package serial

import (
	"fmt"
	"math"
	"time"
)

// MotorSettings combines controller readback with session-local output bounds.
type MotorSettings struct {
	LPFHz            float32 `json:"lpf_hz"`
	HPFHz            float32 `json:"hpf_hz"`
	FiltersSupported bool    `json:"filters_supported"`
	FiltersReady     bool    `json:"filters_ready"`
	Solo             bool    `json:"solo"`
	Ks               int     `json:"ks"`
	TimingReady      bool    `json:"timing_ready"`
	Kp               float32 `json:"kp"`
	Ki               float32 `json:"ki"`
	Kd               float32 `json:"kd"`
	PWMMin           int     `json:"pwm_min"`
	PWMMax           int     `json:"pwm_max"`
	PWMRev           int     `json:"pwm_rev"`
	Deadzone         int     `json:"deadzone"`
	Cutoff           int     `json:"cutoff"`
	Clip             int     `json:"clip"`
	Min              float32 `json:"min"`
	Max              float32 `json:"max"`
	Center           float32 `json:"center"`
	Active           bool    `json:"active"`
	Configured       bool    `json:"configured"`
	Disabled         bool    `json:"disabled"`
	Ready            bool    `json:"ready"`
	JogSupported     bool    `json:"jog_supported"`
	Jogging          bool    `json:"jogging"`
	Diagnostics      bool    `json:"diagnostics"`
}

// BenchDefaults is the initial Motor 1 profile; enabling still checks feedback.
func BenchDefaults() MotorSettings {
	return MotorSettings{Ks: 1, Kp: 20, Ki: 0, Kd: 0.4, PWMMin: 0, PWMMax: 100, PWMRev: 50,
		Cutoff: 23, Clip: 40, Min: 50, Max: 650, Center: 338, Active: true}
}

func (s MotorSettings) Validate() error {
	if err := (MotorFilters{Joint: 1, LPFHz: s.LPFHz, HPFHz: s.HPFHz}).Validate(); err != nil {
		return err
	}
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
		out[i].Solo = p.soloMotor == i+1
		out[i].FiltersSupported = p.filtersSupported
		out[i].FiltersReady = p.filtersSupported && p.filterMask[i] == 3 && time.Since(p.filterSeen[i]) < 2*time.Second
		out[i].TimingReady = time.Since(p.ksSeen[i]) < 2*time.Second
		out[i].Diagnostics = p.diagnostics
		out[i].JogSupported = p.jogSupported
		out[i].Jogging = p.jogActiveLocked() && p.jogMotor == i+1
		out[i].Disabled = p.disabled&(1<<uint(i)) != 0
		out[i].Ready = p.ready && p.autoConfigured && time.Since(p.settingsSeen[i]) < 2*time.Second && p.settingsMask[i] == 63
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
	if !p.autoConfigured {
		return fmt.Errorf("waiting for saved settings to restore")
	}
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
	if !p.autoConfigured {
		return fmt.Errorf("waiting for saved settings to restore")
	}
	return p.writeLocked(pidFrames(j, kp, ki, kd))
}

func (p *Port) checkTargetLocked(j int, v float32) error {
	if p.soloMotor != 0 && j != p.soloMotor {
		return fmt.Errorf("solo mode permits only M%d", p.soloMotor)
	}
	if p.motionFault != "" {
		return fmt.Errorf("%s", p.motionFault)
	}
	if !p.autoConfigured {
		return fmt.Errorf("waiting for saved settings to restore")
	}
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
	return p.enableLocked(1 << uint(j-1))
}

// EnableIncluded validates every included channel before sending any enable.
func (p *Port) EnableIncluded() error {
	p.mu.Lock()
	defer p.mu.Unlock()
	var mask byte
	for i, s := range p.settings {
		if s.Active {
			mask |= 1 << uint(i)
		}
	}
	if mask == 0 {
		return fmt.Errorf("configure and include at least one motor")
	}
	if err := p.enableLocked(mask); err != nil {
		return err
	}
	p.soloMotor = 0
	return nil
}

// EnableOnly is session-local; saved inclusion and tuning remain unchanged.
func (p *Port) EnableOnly(j int) error {
	if j < 1 || j > 3 {
		return fmt.Errorf("motor 1-3 required")
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	if !p.jogSupported {
		return fmt.Errorf("updated SMC3 stop support is required for solo mode")
	}
	if p.diagnostics || p.jogActiveLocked() {
		return fmt.Errorf("turn jog mode off first")
	}
	if err := p.writeLocked([]byte("[stp]")); err != nil {
		return err
	}
	p.guardMask = 0
	p.soloMotor = 0
	p.disabled = 7
	if err := p.enableLocked(1 << uint(j-1)); err != nil {
		return err
	}
	p.soloMotor = j
	return nil
}

func (p *Port) enableLocked(mask byte) error {
	if !p.autoConfigured {
		return fmt.Errorf("waiting for saved settings to restore")
	}
	if p.diagnostics {
		return fmt.Errorf("turn jog mode off before enabling PID")
	}
	if p.jogActiveLocked() {
		return fmt.Errorf("open-loop jog running")
	}
	var data []byte
	for i, s := range p.settings {
		if mask&(1<<uint(i)) == 0 {
			continue
		}
		actual := p.joints[i].Actual
		if !s.Configured || !s.Active || time.Since(p.seen[i]) > 250*time.Millisecond || actual < s.Min || actual > s.Max || actual < float32(s.Clip)+4 || actual > float32(1023-s.Clip)-4 {
			return fmt.Errorf("M%d: configure motor and verify fresh feedback inside host and clip limits first", i+1)
		}
		data = append(data, frame(byte('A'+i), uint16(actual))...)
		data = append(data, []byte{'[', 'e', 'n', byte('1' + i), ']'}...)
	}
	if p.holdSupported {
		// Firmware samples exact feedback and arms the entire mask in one command.
		data = frame('h', uint16(mask))
	}
	if err := p.writeLocked(data); err != nil {
		if p.jogSupported {
			_ = p.writeLocked([]byte("[stp]"))
		}
		return err
	}
	p.motionFault = ""
	p.guardMask |= mask
	for i := range p.tracking {
		if mask&(1<<uint(i)) != 0 {
			p.tracking[i] = trackingGuard{}
		}
	}
	return nil
}
