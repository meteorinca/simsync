package serial

import (
	"fmt"
	"time"
)

// ControllerTuning includes shared timer settings; M1 and M2 share Timer 1.
type ControllerTuning struct {
	Joint   int `json:"joint"`
	Ks      int `json:"ks"`
	Divider int `json:"divider"`
	PWM12   int `json:"pwm_12_khz"`
	PWM3    int `json:"pwm_3_khz"`
}

func (s ControllerTuning) Validate() error {
	if s.Joint < 1 || s.Joint > 3 || s.Ks < 1 || s.Ks > 100 || s.Divider < 1 || s.Divider > 10 || s.PWM12 < 5 || s.PWM12 > 35 || (s.PWM3 != 4 && s.PWM3 != 31) {
		return fmt.Errorf("require motor 1-3, Ks 1-100, PID divider 1-10, M1/2 PWM 5-35 kHz and M3 PWM 4 or 31 kHz")
	}
	return nil
}
func (p *Port) TuneController(s ControllerTuning) error {
	if err := s.Validate(); err != nil {
		return err
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	if !p.autoConfigured {
		return fmt.Errorf("waiting for saved settings to restore")
	}
	if p.diagnostics || p.jogActiveLocked() {
		return fmt.Errorf("turn jog mode off before tuning")
	}
	if time.Since(p.tuningSeen) > 2*time.Second || time.Since(p.ksSeen[s.Joint-1]) > 2*time.Second {
		return fmt.Errorf("waiting for timing readback")
	}
	data := frame(byte('M'+s.Joint-1), uint16(s.Ks))
	data = append(data, frame('Z', uint16(s.Divider<<8))...)
	data = append(data, frame('~', uint16(s.PWM12<<8|s.PWM3))...)
	return p.writeLocked(data)
}
