// SimSync - motion simulator control suite.
// Bridges game telemetry → motion transforms → SMC3 on Arduino Uno via serial.
// Exposes a local dashboard at http://localhost:7070
package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/exec"
	"os/signal"
	"runtime"
	"sync"
	"syscall"
	"time"

	"github.com/meteorinca/simsync/internal/api"
	"github.com/meteorinca/simsync/internal/bridge"
	"github.com/meteorinca/simsync/internal/motion"
	"github.com/meteorinca/simsync/internal/pid"
	"github.com/meteorinca/simsync/internal/serial"
	"github.com/meteorinca/simsync/internal/session"
	"github.com/meteorinca/simsync/webstatic"
)

func main() {
	// --- CLI flags ---
	serialPort := flag.String("serial", "", "Serial port (e.g. /dev/ttyACM0 on Linux or COM9 on Windows). Empty = auto-detect.")
	baud := flag.Int("baud", 500000, "Serial baud rate")
	udpPort := flag.Int("udp-port", 20777, "UDP telemetry listen port")
	httpPort := flag.Int("port", 7070, "SimSync HTTP/WS port")
	noSerial := flag.Bool("no-serial", false, "Disable serial (dashboard only)")
	noUDP := flag.Bool("no-udp", false, "Disable game bridge (bench / calibration mode)")
	sessionSec := flag.Int("session-sec", 300, "Session ring buffer duration in seconds")
	openBrowser := flag.Bool("open-browser", true, "Open dashboard in browser on start")
	flag.Parse()

	log.Printf("SimSync starting - dashboard will be at http://localhost:%d", *httpPort)

	// --- Shared state ---
	var mu sync.RWMutex
	var currentPID = struct{ Kp, Ki, Kd float32 }{}
	var latestTelemetry bridge.Telemetry
	var jointStates [3]api.JointMsg // index 0 = joint 1
	var armed bool
	var manualMode bool
	var estopTripped bool
	var estopCode int
	var estopReason string
	var udpHz float32
	var udpPacketCount int
	var lastUDPRateCalc = time.Now()

	// --- Services ---
	store := session.New(*sessionSec * 100) // ~100 samples/sec max
	engine := motion.NewEngine(motion.DefaultConfig())
	var lastTelemetryTime time.Time

	// --- Serial port ---
	var serialPort_ *serial.Port
	if !*noSerial {
		serialPort_ = serial.New(*serialPort, *baud, func(jt serial.JointTelemetry) {
			mu.Lock()
			defer mu.Unlock()
			idx := jt.Joint - 1
			if idx >= 0 && idx < 3 {
				jointStates[idx] = api.JointMsg{
					Joint:  jt.Joint,
					Target: jt.Target,
					Actual: jt.Actual,
					Error:  jt.Error,
					Duty:   jt.Duty,
				}
			}
			kp, ki, kd := currentPID.Kp, currentPID.Ki, currentPID.Kd
			store.Add(session.Sample{
				T:      time.Now(),
				Joint:  jt.Joint,
				Target: jt.Target,
				Actual: jt.Actual,
				Error:  jt.Error,
				Duty:   jt.Duty,
				Kp:     kp, Ki: ki, Kd: kd,
			})
		})
		serialPort_.Start()
	}

	// --- UDP bridge ---
	var udpListener *bridge.Listener
	if !*noUDP {
		udpListener = bridge.New(*udpPort)
		go func() {
			log.Printf("[udp] listening on :%d", *udpPort)
			if err := udpListener.Run(func(t bridge.Telemetry) {
				mu.Lock()
				latestTelemetry = t
				lastTelemetryTime = time.Now()
				udpPacketCount++
				if time.Since(lastUDPRateCalc) >= time.Second {
					udpHz = float32(udpPacketCount)
					udpPacketCount = 0
					lastUDPRateCalc = time.Now()
				}
				mu.Unlock()
			}); err != nil {
				log.Printf("[udp] error: %v", err)
			}
		}()
	}

	// --- Motion → Serial sender (60 Hz) ---
	if !*noSerial && !*noUDP {
		go func() {
			ticker := time.NewTicker(16 * time.Millisecond) // ~60 Hz
			var lastDt time.Time
			for range ticker.C {
				mu.Lock()
				if !armed || estopTripped || manualMode {
					mu.Unlock()
					continue
				}
				tel := latestTelemetry
				lastT := lastTelemetryTime

				if time.Since(lastT) > 200*time.Millisecond {
					mu.Unlock()
					continue // no live telemetry - hold position
				}

				now := time.Now()
				dt := float32(16.0 / 1000.0)
				if !lastDt.IsZero() {
					dt = float32(now.Sub(lastDt).Seconds())
				}
				lastDt = now

				targets := engine.Transform(
					tel.GLat, tel.GLong,
					tel.SuspFL, tel.SuspFR, tel.SuspRL, tel.SuspRR,
					dt,
				)
				if err := serialPort_.SendTargets(targets.Pitch, targets.Roll, targets.Heave); err != nil {
					armed = false
					log.Printf("[motion] paused: %v", err)
				}
				mu.Unlock()
			}
		}()
	}

	// --- Step test runner ---
	setPID := func(joint int, kp, ki, kd float32) error {
		if serialPort_ == nil {
			return fmt.Errorf("serial disabled")
		}
		return serialPort_.SetMotorPID(joint, kp, ki, kd)
	}
	setTarget := func(joint int, value float32) error {
		mu.Lock()
		defer mu.Unlock()
		if !armed || estopTripped {
			return fmt.Errorf("resume motion before commanding a target")
		}
		if serialPort_ == nil {
			return fmt.Errorf("serial disabled")
		}
		manualMode = true
		return serialPort_.SetTarget(joint, value)
	}
	stepRunner := pid.NewRunner(
		func(joint int) (float32, error) {
			if serialPort_ == nil {
				return 0, fmt.Errorf("serial disabled")
			}
			return serialPort_.Actual(joint)
		},
		func(j int, kp, ki, kd float32) error {
			mu.Lock()
			defer mu.Unlock()
			if !armed || estopTripped {
				return fmt.Errorf("host output paused")
			}
			return setPID(j, kp, ki, kd)
		}, setTarget,
	)
	stepRunner.CheckTarget = func(joint int, value float32) error {
		mu.RLock()
		defer mu.RUnlock()
		if !armed || estopTripped || serialPort_ == nil {
			return fmt.Errorf("resume host output first")
		}
		return serialPort_.CheckTarget(joint, value)
	}

	stepRunner.OnResult(func(m pid.StepMetrics) {
		log.Printf("[step] kp=%.2f kd=%.2f  rise=%.0fms  overshoot=%.1f%%  settle=%.0fms  sse=%.2f counts",
			m.Kp, m.Kd, m.RiseTimeMs, m.OvershootPct, m.SettleTimeMs, m.SteadyStateErr)
	})

	// --- API Deps ---
	deps := &api.Deps{
		GetSessionSamples: func(n int) interface{} { return store.Last(n) },
		ExportSessionCSV:  func(w http.ResponseWriter) { store.ExportCSV(w) },
		GetPID: func() (float32, float32, float32) {
			if serialPort_ == nil {
				return 0, 0, 0
			}
			return serialPort_.PID()
		},
		SetPID: func(j int, kp, ki, kd float32) error {
			if stepRunner.IsRunning() {
				return fmt.Errorf("test running")
			}
			return setPID(j, kp, ki, kd)
		},
		GetMotors: func() interface{} {
			if serialPort_ == nil {
				return [3]serial.MotorSettings{}
			}
			return serialPort_.Motors()
		},
		ConfigureMotor: func(j int, s serial.MotorSettings) error {
			mu.Lock()
			defer mu.Unlock()
			if armed || stepRunner.IsRunning() {
				return fmt.Errorf("pause before configuring")
			}
			if serialPort_ == nil {
				return fmt.Errorf("serial disabled")
			}
			if err := serialPort_.Configure(j, s); err != nil {
				return err
			}
			cfg := engine.GetConfig()
			switch j {
			case 1:
				cfg.PitchNeutral = s.Center
			case 2:
				cfg.RollNeutral = s.Center
			case 3:
				cfg.HeaveNeutral = s.Center
			}
			engine.SetConfig(cfg)
			return nil
		},
		EnableMotor: func(j int) error {
			mu.Lock()
			defer mu.Unlock()
			if armed || estopTripped || stepRunner.IsRunning() {
				return fmt.Errorf("pause and clear host latch before enabling")
			}
			if serialPort_ == nil {
				return fmt.Errorf("serial disabled")
			}
			return serialPort_.Enable(j)
		},
		JogMotor: func(j, pwm, ms int) error {
			mu.Lock()
			defer mu.Unlock()
			if estopTripped || stepRunner.IsRunning() {
				return fmt.Errorf("clear pause latch and finish step test first")
			}
			if serialPort_ == nil {
				return fmt.Errorf("serial disabled")
			}
			armed = false
			manualMode = true
			return serialPort_.Jog(j, pwm, ms)
		},
		Diagnostics: func(enabled bool) error {
			mu.Lock()
			defer mu.Unlock()
			if serialPort_ == nil {
				return fmt.Errorf("serial disabled")
			}
			if enabled && (estopTripped || stepRunner.IsRunning()) {
				return fmt.Errorf("clear pause latch and finish step test first")
			}
			armed = false
			manualMode = true
			return serialPort_.SetDiagnostics(enabled)
		},
		StopJog: func() error {
			mu.Lock()
			defer mu.Unlock()
			armed = false
			if serialPort_ == nil {
				return fmt.Errorf("serial disabled")
			}
			return serialPort_.StopJog()
		},
		CancelTest:  func() { mu.Lock(); armed = false; mu.Unlock(); stepRunner.Cancel() },
		TestRunning: stepRunner.IsRunning,
		SavePID: func() error {
			mu.Lock()
			defer mu.Unlock()
			if armed || stepRunner.IsRunning() {
				return fmt.Errorf("pause before saving EEPROM")
			}
			if serialPort_ == nil {
				return fmt.Errorf("serial disabled")
			}
			return serialPort_.SavePID()
		},
		Calibrate: func(joint int) error { return fmt.Errorf("set pot alignment and travel limits in SMC3Utils") },
		Autotune:  func(joint int) error { return fmt.Errorf("SMC3 has no on-device autotune; use a host step test") },
		SetTarget: func(j int, v float32) error {
			if stepRunner.IsRunning() {
				return fmt.Errorf("test running")
			}
			return setTarget(j, v)
		},
		Arm: func(state bool) error {
			mu.Lock()
			defer mu.Unlock()
			if state && (estopTripped || serialPort_ == nil || !serialPort_.IsConnected()) {
				return fmt.Errorf("clear the host pause latch and connect SMC3 first")
			}
			if state {
				if stepRunner.IsRunning() {
					return fmt.Errorf("wait for test to finish")
				}
				active := false
				for _, s := range serialPort_.Motors() {
					if s.Active {
						active = true
					}
				}
				if !active {
					return fmt.Errorf("configure and include at least one motor")
				}
				for i, s := range serialPort_.Motors() {
					if s.Active {
						if err := serialPort_.Enable(i + 1); err != nil {
							return err
						}
					}
				}
			}
			armed = state
			if !state {
				stepRunner.Cancel()
				if serialPort_ != nil {
					return serialPort_.StopJog()
				}
			}
			manualMode = false
			return nil
		},
		EStop: func(trip bool) error {
			mu.Lock()
			defer mu.Unlock()
			estopTripped = trip
			stepRunner.Cancel()
			armed = false
			estopReason = "Host output paused; SMC3 still holds position"
			if serialPort_ != nil {
				return serialPort_.StopJog()
			}
			return nil
		},
		ClearEStop: func() error {
			mu.Lock()
			defer mu.Unlock()
			estopTripped = false
			stepRunner.Cancel()
			armed = false
			estopCode = 0
			estopReason = "Host motion paused"
			return nil
		},
		RunStepTest: func(joint int, kp, ki, kd, stepSize float32) error {
			if !*noUDP {
				return fmt.Errorf("restart with --no-udp for bench tests")
			}
			if serialPort_ == nil {
				return fmt.Errorf("serial disabled")
			}
			if err := serialPort_.StepReady(joint); err != nil {
				return err
			}
			return stepRunner.RunSingle(joint, kp, ki, kd, stepSize)
		},
		GetStepResults:  func() interface{} { return stepRunner.Results() },
		GetMotionConfig: func() interface{} { return engine.GetConfig() },
		SetMotionConfig: func(body []byte) error {
			mu.Lock()
			defer mu.Unlock()
			if armed || stepRunner.IsRunning() {
				return fmt.Errorf("pause before changing motion mapping")
			}
			cfg := engine.GetConfig()
			if err := json.Unmarshal(body, &cfg); err != nil {
				return err
			}
			if err := cfg.Validate(); err != nil {
				return err
			}
			engine.SetConfig(cfg)
			return nil
		},
		ControllerStats: func() (interface{}, error) {
			if serialPort_ == nil {
				return nil, fmt.Errorf("serial disabled")
			}
			return serialPort_.Stats()
		},
		SerialOK: func() bool {
			return serialPort_ != nil && serialPort_.IsConnected()
		},
		UDPHz: func() float32 {
			mu.RLock()
			defer mu.RUnlock()
			return udpHz
		},
		Armed: func() bool {
			mu.RLock()
			defer mu.RUnlock()
			return armed
		},
		EStopState: func() (bool, int, string) {
			mu.RLock()
			defer mu.RUnlock()
			return estopTripped, estopCode, estopReason
		},
	}

	// --- WebSocket broadcast loop (60 Hz) ---
	srv := api.New(*httpPort, api.NewHandlers(deps), webstatic.FS())
	go func() {
		ticker := time.NewTicker(16 * time.Millisecond)
		for range ticker.C {
			mu.Lock()
			if serialPort_ != nil {
				if !serialPort_.IsConnected() {
					armed = false
				}
				kp, ki, kd := serialPort_.PID()
				currentPID = struct{ Kp, Ki, Kd float32 }{kp, ki, kd}
			}
			mu.Unlock()
			mu.RLock()
			tel := latestTelemetry
			js := jointStates
			pid_ := currentPID
			arm_ := armed
			es := estopTripped
			ec := estopCode
			er := estopReason
			hz := udpHz
			mu.RUnlock()

			estopped, _, _ := deps.EStopState()
			_ = estopped

			cfg := engine.GetConfig()
			frame := api.WSFrame{
				T: float64(time.Now().UnixMilli()) / 1000.0,
				Telemetry: api.TelemetryMsg{
					SpeedMS:  tel.Speed,
					SpeedKMH: tel.Speed * 3.6,
					GLat:     tel.GLat,
					GLong:    tel.GLong,
					Throttle: tel.Throttle,
					Brake:    tel.Brake,
					Steer:    tel.Steer,
					Gear:     tel.Gear,
					RPM:      tel.RPM,
				},
				Joints: js,
				PID:    api.PIDMsg{Kp: pid_.Kp, Ki: pid_.Ki, Kd: pid_.Kd},
				Motion: api.MotionMsg{
					PitchGain: cfg.PitchGain, RollGain: cfg.RollGain, HeaveGain: cfg.HeaveGain,
					PitchNeutral: cfg.PitchNeutral, RollNeutral: cfg.RollNeutral, HeaveNeutral: cfg.HeaveNeutral,
					PitchLimit: cfg.PitchLimit, RollLimit: cfg.RollLimit, HeaveLimit: cfg.HeaveLimit,
					FilterHz: cfg.FilterHz,
				},
				Armed:       arm_,
				EStop:       es,
				EStopCode:   ec,
				EStopReason: er,
				SerialOK:    deps.SerialOK(),
				UDPHZ:       hz,
			}
			srv.Hub.Broadcast(frame)
		}
	}()

	// --- Auto-open browser ---
	if *openBrowser {
		go func() {
			time.Sleep(600 * time.Millisecond)
			url := fmt.Sprintf("http://localhost:%d", *httpPort)
			var cmd *exec.Cmd
			switch runtime.GOOS {
			case "windows":
				cmd = exec.Command("rundll32", "url.dll,FileProtocolHandler", url)
			case "darwin":
				cmd = exec.Command("open", url)
			default:
				cmd = exec.Command("xdg-open", url)
			}
			cmd.Start()
		}()
	}

	// --- Graceful shutdown ---
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	go func() {
		<-quit
		log.Println("[simsync] shutting down...")
		if udpListener != nil {
			udpListener.Stop()
		}
		if serialPort_ != nil {
			serialPort_.Stop()
		}
		os.Exit(0)
	}()

	if err := srv.Start(); err != nil {
		log.Fatalf("[simsync] server error: %v", err)
	}
}
