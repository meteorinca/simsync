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
	serialPort := flag.String("serial", "", "Serial port (e.g. COM9). Empty = auto-detect.")
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
	setPID := func(kp, ki, kd float32) error {
		if serialPort_ == nil {
			return fmt.Errorf("serial disabled")
		}
		return serialPort_.SetPID(kp, ki, kd)
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
		setPID, setTarget,
	)

	stepRunner.OnResult(func(m pid.StepMetrics) {
		log.Printf("[step] kp=%.2f kd=%.2f  rise=%.0fms  overshoot=%.1f%%  settle=%.0fms  sse=%.2f counts",
			m.Kp, m.Kd, m.RiseTimeMs, m.OvershootPct, m.SettleTimeMs, m.SteadyStateErr)
	})

	// --- PID Sweep helper ---
	linspace := func(min, max float32, n int) []float32 {
		out := make([]float32, n)
		step := (max - min) / float32(n-1)
		for i := range out {
			out[i] = min + float32(i)*step
		}
		return out
	}

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
		SetPID: setPID,
		SavePID: func() error {
			if serialPort_ == nil {
				return fmt.Errorf("serial disabled")
			}
			return serialPort_.SavePID()
		},
		Calibrate: func(joint int) error { return fmt.Errorf("set pot alignment and travel limits in SMC3Utils") },
		Autotune:  func(joint int) error { return fmt.Errorf("SMC3 has no on-device autotune; use a host step test") },
		SetTarget: setTarget,
		Arm: func(state bool) error {
			mu.Lock()
			defer mu.Unlock()
			if state && (estopTripped || serialPort_ == nil || !serialPort_.IsConnected()) {
				return fmt.Errorf("clear the host pause latch and connect SMC3 first")
			}
			armed = state
			manualMode = false
			return nil
		},
		EStop: func(trip bool) error {
			mu.Lock()
			defer mu.Unlock()
			estopTripped = trip
			armed = false
			estopReason = "Host output paused; SMC3 still holds position"
			return nil
		},
		ClearEStop: func() error {
			mu.Lock()
			defer mu.Unlock()
			estopTripped = false
			armed = false
			estopCode = 0
			estopReason = "Host motion paused"
			return nil
		},
		RunStepTest: func(joint int, kp, ki, kd, stepSize float32) error {
			return stepRunner.RunSingle(joint, kp, ki, kd, stepSize)
		},
		RunSweep: func(joint int, kpMin, kpMax, kdMin, kdMax float32, steps int, ki, stepSize float32) {
			stepRunner.RunSweep(joint, linspace(kpMin, kpMax, steps), linspace(kdMin, kdMax, steps), ki, stepSize)
		},
		GetStepResults:  func() interface{} { return stepRunner.Results() },
		GetMotionConfig: func() interface{} { return engine.GetConfig() },
		SetMotionConfig: func(body []byte) error {
			cfg := engine.GetConfig()
			if err := json.Unmarshal(body, &cfg); err != nil {
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
