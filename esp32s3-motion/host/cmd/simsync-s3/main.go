package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"math"
	"net/http"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	"simsync-s3/internal/telemetry"
	"simsync-s3/internal/transport"
	"simsync-s3/internal/washout"
	"simsync-s3/internal/web"
)

type orchestrator struct {
	mu          sync.Mutex
	engine      *washout.Engine
	ctrl        *transport.Controller
	profilePath string
	benchMode   bool
	outputHz    int

	// Latest incoming UDP frame + previous frame for smooth 500 Hz interpolation
	prevInput    washout.Input
	currInput    washout.Input
	lastUDPTime  time.Time
	udpInterval  time.Duration
	udpRateHz    float64
	hasUDP       bool

	// Smooth 500 ms center watchdog ramp when UDP stops
	rampActive   bool
	rampStart    time.Time
	rampFrom     [3]uint16

	// Manual / Bench target & sine test override
	manualActive bool
	manualTarget [3]uint16
	sineActive   bool
	sineMotor    int
	sineCenter   float64
	sineAmp      float64
	sineFreqHz   float64
	sineStart    time.Time
	sineEnd      time.Time

	lastOut      washout.Output
}

func (o *orchestrator) loadProfile() washout.Config {
	cfg := washout.DefaultConfig()
	data, err := os.ReadFile(o.profilePath)
	if err == nil {
		if err := json.Unmarshal(data, &cfg); err == nil {
			log.Printf("[profile] loaded %s (DOF=%d)", o.profilePath, cfg.DOFMode)
			return cfg
		}
	}
	_ = o.saveProfile(cfg)
	return cfg
}

func (o *orchestrator) saveProfile(cfg washout.Config) error {
	data, err := json.MarshalIndent(cfg, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(o.profilePath, data, 0644)
}

func (o *orchestrator) onUDP(f telemetry.Frame) {
	suspAvg := (f.SuspVelFL + f.SuspVelFR + f.SuspVelRL + f.SuspVelRR) * 0.25
	suspLR := ((f.SuspVelFL + f.SuspVelRL) - (f.SuspVelFR + f.SuspVelRR)) * 0.5
	suspFB := ((f.SuspVelFL + f.SuspVelFR) - (f.SuspVelRL + f.SuspVelRR)) * 0.5

	in := washout.Input{
		SurgeG:  f.SurgeG,
		SwayG:   f.SwayG,
		HeaveG:  f.HeaveG,
		Pitch:   f.Pitch,
		Roll:    f.Roll,
		SuspAvg: suspAvg,
		SuspLR:  suspLR,
		SuspFB:  suspFB,
		YawRate: f.YawSlip,
	}

	now := time.Now()
	o.mu.Lock()
	defer o.mu.Unlock()
	if o.hasUDP {
		dt := now.Sub(o.lastUDPTime)
		if dt > 2*time.Millisecond && dt < 250*time.Millisecond {
			o.udpInterval = dt
			instHz := 1.0 / dt.Seconds()
			o.udpRateHz = 0.85*o.udpRateHz + 0.15*instHz
		}
	} else {
		o.udpInterval = 16 * time.Millisecond
		o.udpRateHz = 60
	}
	o.prevInput = o.currInput
	o.currInput = in
	o.lastUDPTime = now
	o.hasUDP = true
	o.rampActive = false
}

func lerp(a, b, t float64) float64 { return a + (b-a)*t }

func (o *orchestrator) activeMaskLocked() uint8 {
	if o.engine.GetConfig().DOFMode >= 3 {
		return 0x07
	}
	return 0x03
}

func (o *orchestrator) tickOutput() {
	now := time.Now()

	o.mu.Lock()
	cfg := o.engine.GetConfig()
	mask := o.activeMaskLocked()

	// 1. Bench sine test mode
	if o.sineActive {
		if now.After(o.sineEnd) {
			o.sineActive = false
			o.manualTarget[o.sineMotor] = uint16(math.Round(o.sineCenter))
		} else {
			elapsed := now.Sub(o.sineStart).Seconds()
			pos := o.sineCenter + o.sineAmp*math.Sin(2.0*math.Pi*o.sineFreqHz*elapsed)
			pos = math.Max(cfg.MinBound12[o.sineMotor], math.Min(cfg.MaxBound12[o.sineMotor], pos))
			o.manualTarget[o.sineMotor] = uint16(math.Round(pos))
		}
		targets := o.manualTarget
		o.lastOut.Target12 = targets
		o.mu.Unlock()
		_ = o.ctrl.SendTargets12(targets, 1<<uint(o.sineMotor))
		return
	}

	// 2. Manual bench target mode
	if o.manualActive || o.benchMode {
		targets := o.manualTarget
		o.lastOut.Target12 = targets
		o.mu.Unlock()
		_ = o.ctrl.SendTargets12(targets, mask)
		return
	}

	// 3. Game UDP mode with interpolation & 500 ms smooth center ramp watchdog
	udpFresh := o.hasUDP && now.Sub(o.lastUDPTime) < 250*time.Millisecond
	if udpFresh {
		// Linear interpolation across the inter-packet interval for silky 500 Hz motion
		alpha := 1.0
		if o.udpInterval > 0 {
			alpha = float64(now.Sub(o.lastUDPTime)) / float64(o.udpInterval)
			if alpha < 0 {
				alpha = 0
			} else if alpha > 1 {
				alpha = 1
			}
		}
		interp := washout.Input{
			SurgeG:  lerp(o.prevInput.SurgeG, o.currInput.SurgeG, alpha),
			SwayG:   lerp(o.prevInput.SwayG, o.currInput.SwayG, alpha),
			HeaveG:  lerp(o.prevInput.HeaveG, o.currInput.HeaveG, alpha),
			Pitch:   lerp(o.prevInput.Pitch, o.currInput.Pitch, alpha),
			Roll:    lerp(o.prevInput.Roll, o.currInput.Roll, alpha),
			SuspAvg: lerp(o.prevInput.SuspAvg, o.currInput.SuspAvg, alpha),
			SuspLR:  lerp(o.prevInput.SuspLR, o.currInput.SuspLR, alpha),
			SuspFB:  lerp(o.prevInput.SuspFB, o.currInput.SuspFB, alpha),
			YawRate: lerp(o.prevInput.YawRate, o.currInput.YawRate, alpha),
		}
		out := o.engine.Step(interp)
		o.lastOut = out
		o.mu.Unlock()
		_ = o.ctrl.SendTargets12(out.Target12, mask)
		return
	}

	// UDP stopped: smoothly ramp back to center over 500 ms instead of jerking!
	if o.hasUDP && !o.rampActive {
		o.rampActive = true
		o.rampStart = now
		o.rampFrom = o.lastOut.Target12
		o.engine.Reset()
	}
	if o.rampActive {
		frac := now.Sub(o.rampStart).Seconds() / 0.500
		if frac >= 1.0 {
			frac = 1.0
			o.rampActive = false
			o.hasUDP = false
		}
		// Smoothstep easing (3t^2 - 2t^3)
		ease := frac * frac * (3.0 - 2.0*frac)
		var targets [3]uint16
		for i := 0; i < 3; i++ {
			targets[i] = uint16(math.Round(lerp(float64(o.rampFrom[i]), cfg.Center12[i], ease)))
		}
		o.lastOut.Target12 = targets
		o.mu.Unlock()
		_ = o.ctrl.SendTargets12(targets, mask)
		return
	}

	o.mu.Unlock()
}

// ---- web.Actions implementation ----

func (o *orchestrator) GetWashout() washout.Config { return o.engine.GetConfig() }

func (o *orchestrator) SetWashout(cfg washout.Config) error {
	o.engine.SetConfig(cfg)
	_ = o.ctrl.SendControl(transport.OpSetActiveDOF, uint16(cfg.DOFMode), 0)
	return o.saveProfile(cfg)
}

func (o *orchestrator) StopAll() error {
	o.mu.Lock()
	o.sineActive = false
	o.mu.Unlock()
	return o.ctrl.SendControl(transport.OpStopAll, 0, 0)
}

func (o *orchestrator) HoldEnable() error {
	o.mu.Lock()
	mask := o.activeMaskLocked()
	snap := o.ctrl.Snapshot()
	for i := 0; i < 3; i++ {
		o.manualTarget[i] = snap.Motors[i].Actual12
	}
	o.mu.Unlock()
	return o.ctrl.SendControl(transport.OpHoldMask, uint16(mask), 0)
}

func (o *orchestrator) CenterAll() error {
	cfg := o.engine.GetConfig()
	o.mu.Lock()
	o.sineActive = false
	o.manualActive = true
	for i := 0; i < 3; i++ {
		o.manualTarget[i] = uint16(math.Round(cfg.Center12[i]))
	}
	o.mu.Unlock()
	return nil
}

func (o *orchestrator) SetBenchTarget(motor int, target12 uint16) error {
	if motor < 0 || motor > 2 {
		return fmt.Errorf("invalid motor")
	}
	cfg := o.engine.GetConfig()
	if float64(target12) < cfg.MinBound12[motor] || float64(target12) > cfg.MaxBound12[motor] {
		return fmt.Errorf("target %d outside host bounds [%.0f..%.0f]", target12, cfg.MinBound12[motor], cfg.MaxBound12[motor])
	}
	o.mu.Lock()
	o.sineActive = false
	o.manualActive = true
	o.manualTarget[motor] = target12
	o.mu.Unlock()
	return nil
}

func (o *orchestrator) RunSineTest(motor int, amp12 uint16, freqHz float64, cycles int) error {
	if motor < 0 || motor > 2 || amp12 < 10 || amp12 > 600 || freqHz <= 0 || freqHz > 4 || cycles < 1 || cycles > 10 {
		return fmt.Errorf("invalid sine test parameters")
	}
	snap := o.ctrl.Snapshot()
	center := float64(snap.Motors[motor].Actual12)
	cfg := o.engine.GetConfig()
	if center-float64(amp12) < cfg.MinBound12[motor] || center+float64(amp12) > cfg.MaxBound12[motor] {
		return fmt.Errorf("sine amplitude exceeds host bounds")
	}
	dur := time.Duration(float64(cycles)/freqHz*1000.0) * time.Millisecond
	o.mu.Lock()
	o.manualActive = true
	o.sineActive = true
	o.sineMotor = motor
	o.sineCenter = center
	o.sineAmp = float64(amp12)
	o.sineFreqHz = freqHz
	o.sineStart = time.Now()
	o.sineEnd = o.sineStart.Add(dur)
	o.mu.Unlock()
	return nil
}

func (o *orchestrator) SetGains(g transport.MotorGains) error    { return o.ctrl.SendGains(g) }
func (o *orchestrator) SetLimits(l transport.MotorLimits) error  { return o.ctrl.SendLimits(l) }
func (o *orchestrator) SetPWMkHz(p12, p3 uint16) error           { return o.ctrl.SendControl(transport.OpSetPWMkHz, p12, p3) }
func (o *orchestrator) SaveNVS() error                           { return o.ctrl.SendControl(transport.OpSaveNVS, 0, 0) }
func (o *orchestrator) ClearEStop() error                        { return o.ctrl.SendControl(transport.OpClearEStop, 0, 0) }
func (o *orchestrator) ResetStats() error                        { return o.ctrl.SendControl(transport.OpResetStats, 0, 0) }

func main() {
	serialPort := flag.String("serial", "", "Serial port (empty = auto-detect ESP32-S3 USB)")
	baud := flag.Int("baud", 921600, "Serial baud rate (ignored by native USB CDC, 500000 for SMC3 Uno)")
	smc3Mode := flag.Bool("smc3", false, "Use legacy 10-bit SMC3 protocol instead of ESP32-S3 Turbo v2 12-bit")
	udpPort := flag.Int("udp-port", 20777, "Dirt Rally 2.0 UDP telemetry port")
	httpPort := flag.Int("port", 7080, "Live tuner web dashboard port")
	outputHz := flag.Int("hz", 500, "Decoupled motion output tick rate (Hz, 200..1000)")
	benchMode := flag.Bool("bench", false, "Bench mode (ignore game UDP, manual/sine control only)")
	profilePath := flag.String("profile", "profile.json", "Path to washout JSON profile")
	flag.Parse()

	if *outputHz < 100 {
		*outputHz = 100
	}
	if *outputHz > 1000 {
		*outputHz = 1000
	}

	orch := &orchestrator{
		profilePath: *profilePath,
		benchMode:   *benchMode,
		outputHz:    *outputHz,
		manualTarget: [3]uint16{2048, 2048, 2048},
	}
	cfg := orch.loadProfile()
	orch.engine = washout.NewEngine(cfg, float64(*outputHz))

	ctrl := transport.NewController(*serialPort, *baud, *smc3Mode, nil)
	orch.ctrl = ctrl
	ctrl.Start()
	defer ctrl.Stop()

	if !*benchMode {
		listener := telemetry.NewListener(*udpPort)
		go func() {
			log.Printf("[udp] listening for Dirt Rally 2.0 telemetry on :%d", *udpPort)
			if err := listener.Run(orch.onUDP); err != nil {
				log.Printf("[udp] error: %v", err)
			}
		}()
		defer listener.Stop()
	}

	srv := web.NewServer(orch)

	// High-frequency decoupled output loop (500 Hz default)
	stopCh := make(chan struct{})
	go func() {
		ticker := time.NewTicker(time.Second / time.Duration(*outputHz))
		defer ticker.Stop()
		for {
			select {
			case <-stopCh:
				return
			case <-ticker.C:
				orch.tickOutput()
			}
		}
	}()

	// 30 Hz WebSocket dashboard broadcaster
	go func() {
		ticker := time.NewTicker(33 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-stopCh:
				return
			case <-ticker.C:
				snap := ctrl.Snapshot()
				orch.mu.Lock()
				udpLive := orch.hasUDP && time.Since(orch.lastUDPTime) < 500*time.Millisecond
				pkt := web.StatePacket{
					Telemetry: snap,
					Running:   snap.DisabledMask != 0x07,
					BenchMode: orch.benchMode,
					UDPRateHz: orch.udpRateHz,
					UDPLive:   udpLive,
					PitchCmd:  orch.lastOut.PitchCmd,
					RollCmd:   orch.lastOut.RollCmd,
					HeaveCmd:  orch.lastOut.HeaveCmd,
					Target12:  orch.lastOut.Target12,
				}
				orch.mu.Unlock()
				srv.Broadcast(pkt)
			}
		}
	}()

	addr := fmt.Sprintf("127.0.0.1:%d", *httpPort)
	httpSrv := &http.Server{Addr: addr, Handler: srv.Handler()}
	go func() {
		log.Printf("[web] SimSync ESP32-S3 Live Tuner listening at http://%s", addr)
		if err := httpSrv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("[web] server error: %v", err)
		}
	}()

	sigCh := make(chan os.Signal, 1)
	signal.Notify(sigCh, os.Interrupt, syscall.SIGTERM)
	<-sigCh
	close(stopCh)
	_ = ctrl.SendControl(transport.OpStopAll, 0, 0)
	_ = httpSrv.Close()
	log.Println("[main] cleanly stopped all motors and exited")
}
