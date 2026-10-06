package serial

import (
	"fmt"
	"math"
	"time"
)

// MotorSettings combines controller readback with session-local output bounds.
type MotorSettings struct {
	Kp           float32 `json:"kp"`
	Ki           float32 `json:"ki"`
	Kd           float32 `json:"kd"`
	PWMMin       int     `json:"pwm_min"`
	PWMMax       int     `json:"pwm_max"`
	PWMRev       int     `json:"pwm_rev"`
	Deadzone     int     `json:"deadzone"`
	Cutoff       int     `json:"cutoff"`
	Clip         int     `json:"clip"`
	Min          float32 `json:"min"`
	Max          float32 `json:"max"`
	Center       float32 `json:"center"`
	Active       bool    `json:"active"`
	Configured   bool    `json:"configured"`
	Disabled     bool    `json:"disabled"`
	Ready        bool    `json:"ready"`
	JogSupported bool    `json:"jog_supported"`
	Jogging      bool    `json:"jogging"`
	Diagnostics  bool    `json:"diagnostics"`
}

// BenchDefaults is the initial Motor 1 profile; enabling still checks feedback.
func BenchDefaults() MotorSettings {
	return MotorSettings{Kp: 1, Ki: 0, Kd: 0.4, PWMMin: 0, PWMMax: 100, PWMRev: 50,
		Cutoff: 23, Clip: 40, Min: 50, Max: 650, Center: 512, Active: true}
}

func (s MotorSettings) Validate() error {
	for _, v := range []float32{s.Kp, s.Ki, s.Kd} {
		if math.IsNaN(float64(v)) || math.IsInf(float64(v), 0) || v < 0 || v > 327.67 {
			return fmt.Errorf("gains must be 0-327.67")
		}
	}
	if s.PWMMin < 0 || s.PWMMax < s.PWMMin || s.PWMMax > 255 || s.PWMRev < 0 || s.PWMRev > s.PWMMax || s.Deadzone < 0 || s.Deadzone > 255 {
		return fmt.Errorf("invalid PWM or deadzone")
	}
	if s.Cutoff < 1 || s.Clip <= s.Cutoff || s.Clip > 255 {
		return fmt.Errorf("require 1 <= cutoff < clip <= 255")
	}
	if !validPosition(s.Min) || !validPosition(s.Max) || !validPosition(s.Center) || s.Min < float32(s.Clip)+4 || s.Max > float32(1023-s.Clip)-4 || s.Min >= s.Center || s.Center >= s.Max {
		return fmt.Errorf("require clip + 4 <= min < center < max <= 1019 - clip")
	}
	for _, v := range []float32{s.Min, s.Max, s.Center} {
		if v != float32(math.Round(float64(v))) {
			return fmt.Errorf("host bounds and center must be whole counts")
		}
	}
	return nil
}

func (p *Port) Motors() [3]MotorSettings {
	p.mu.Lock()
	defer p.mu.Unlock()
	out := p.settings
	for i := range out {
		out[i].Diagnostics = p.diagnostics
		out[i].JogSupported = p.jogSupported
		out[i].Jogging = p.jogActiveLocked() && p.jogMotor == i+1
		out[i].Disabled = p.disabled&(1<<uint(i)) != 0
		out[i].Ready = p.ready && time.Since(p.settingsSeen[i]) < 2*time.Second && p.settingsMask[i] == 63
	}
	return out
}

func (p *Port) Configure(j int, s MotorSettings) error {
	if j < 1 || j > 3 {
		return fmt.Errorf("motor 1-3 required")
	}
	if err := s.Validate(); err != nil {
		return err
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	i := j - 1
	if p.jogActiveLocked() {
		return fmt.Errorf("open-loop jog running")
	}
	if p.settingsMask[i] != 63 || time.Since(p.settingsSeen[i]) > 2*time.Second {
		return fmt.Errorf("waiting for controller settings")
	}
	if time.Since(p.seen[i]) > 250*time.Millisecond || p.joints[i].Actual < s.Min || p.joints[i].Actual > s.Max || p.joints[i].Target < s.Min || p.joints[i].Target > s.Max {
		return fmt.Errorf("actual and target must be inside the proposed range")
	}
	data := pidFrames(j, s.Kp, s.Ki, s.Kd)
	data = append(data, frame(byte('P'+i), uint16(s.PWMMin<<8|s.PWMMax))...)
	data = append(data, frame(byte('S'+i), uint16(s.Cutoff<<8|s.Clip))...)
	data = append(data, frame(byte('V'+i), uint16(s.Deadzone<<8|s.PWMRev))...)
	if err := p.writeLocked(data); err != nil {
		p.settings[i].Configured = false
		return err
	}
	// Only host bounds are committed here. Controller values remain readback.
	p.settings[i].Min, p.settings[i].Max, p.settings[i].Center = s.Min, s.Max, s.Center
	p.settings[i].Active, p.settings[i].Configured = s.Active, true
	return nil
}

func pidFrames(j int, kp, ki, kd float32) []byte {
	var data []byte
	for g, v := range []float32{kp, ki, kd} {
		data = append(data, frame(byte('D'+g*3+j-1), uint16(math.Round(float64(v)*100)))...)
	}
	return data
}

func (p *Port) SetMotorPID(j int, kp, ki, kd float32) error {
	if j < 1 || j > 3 {
		return fmt.Errorf("motor 1-3 required")
	}
	for _, v := range []float32{kp, ki, kd} {
		if math.IsNaN(float64(v)) || math.IsInf(float64(v), 0) || v < 0 || v > 327.67 {
			return fmt.Errorf("invalid gain")
		}
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.jogActiveLocked() {
		return fmt.Errorf("open-loop jog running")
	}
	return p.writeLocked(pidFrames(j, kp, ki, kd))
}

func (p *Port) checkTargetLocked(j int, v float32) error {
	if p.diagnostics {
		return fmt.Errorf("turn jog mode off before closed-loop motion")
	}
	if p.jogActiveLocked() {
		return fmt.Errorf("open-loop jog running")
	}
	if j < 1 || j > 3 || !validPosition(v) {
		return fmt.Errorf("invalid motor or target")
	}
	s := p.settings[j-1]
	if !s.Configured || !s.Active {
		return fmt.Errorf("configure and select motor %d first", j)
	}
	if time.Since(p.seen[j-1]) > 250*time.Millisecond {
		return fmt.Errorf("motor %d feedback stale", j)
	}
	if v < s.Min || v > s.Max {
		return fmt.Errorf("motor %d target outside %.0f-%.0f", j, s.Min, s.Max)
	}
	if p.joints[j-1].Actual < s.Min || p.joints[j-1].Actual > s.Max {
		return fmt.Errorf("motor %d feedback outside host range", j)
	}
	if p.disabled&(1<<uint(j-1)) != 0 {
		return fmt.Errorf("motor %d disabled", j)
	}
	return nil
}

func (p *Port) CheckTarget(j int, v float32) error {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.checkTargetLocked(j, v)
}

func (p *Port) StepReady(j int) error {
	p.mu.Lock()
	defer p.mu.Unlock()
	if j < 1 || j > 3 {
		return fmt.Errorf("motor 1-3 required")
	}
	s := p.joints[j-1]
	if err := p.checkTargetLocked(j, s.Target); err != nil {
		return err
	}
	if math.Abs(float64(s.Actual-s.Target)) > 4 {
		return fmt.Errorf("wait for actual to reach target")
	}
	return nil
}

// Enable uses the existing SMC3 command, explicitly, with a target near feedback.
func (p *Port) Enable(j int) error {
	if j < 1 || j > 3 {
		return fmt.Errorf("motor 1-3 required")
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.diagnostics {
		return fmt.Errorf("turn jog mode off before enabling PID")
	}
	i := j - 1
	s := p.settings[i]
	if p.jogActiveLocked() {
		return fmt.Errorf("open-loop jog running")
	}
	actual := p.joints[i].Actual
	if !s.Configured || !s.Active || time.Since(p.seen[i]) > 250*time.Millisecond || actual < s.Min || actual > s.Max {
		return fmt.Errorf("configure motor and verify feedback inside limits first")
	}
	data := append(frame(byte('A'+i), uint16(actual)), []byte{'[', 'e', 'n', byte('1' + i), ']'}...)
	return p.writeLocked(data)
}
