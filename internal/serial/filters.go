package serial

import (
	"fmt"
	"math"
	"time"
)

type MotorFilters struct {
	Joint int     `json:"joint"`
	LPFHz float32 `json:"lpf_hz"`
	HPFHz float32 `json:"hpf_hz"`
}

func (s MotorFilters) Validate() error {
	if s.Joint < 1 || s.Joint > 3 {
		return fmt.Errorf("motor 1-3 required")
	}
	for i, hz := range []float32{s.LPFHz, s.HPFHz} {
		maximum := float32(200)
		if i == 1 {
			maximum = 50
		}
		if math.IsNaN(float64(hz)) || math.IsInf(float64(hz), 0) || hz < 0 || hz > maximum || (hz > 0 && hz < .1) {
			return fmt.Errorf("LPF must be off (0) or 0.1-200 Hz; derivative HPF off (0) or 0.1-50 Hz")
		}
		if math.Abs(float64(hz)*10-math.Round(float64(hz)*10)) > .0001 {
			return fmt.Errorf("filter cutoffs use 0.1 Hz increments")
		}
	}
	if s.LPFHz > 0 && s.HPFHz >= s.LPFHz {
		return fmt.Errorf("HPF must be below LPF when both are on")
	}
	return nil
}

func filterFrames(j int, lpf, hpf float32) []byte {
	return append(frame(byte('!'+j-1), uint16(math.Round(float64(lpf)*10))), frame(byte('$'+j-1), uint16(math.Round(float64(hpf)*10)))...)
}

func (p *Port) SetMotorFilters(s MotorFilters) error {
	if err := s.Validate(); err != nil {
		return err
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	i := s.Joint - 1
	if !p.filtersSupported {
		return fmt.Errorf("upload updated SMC3 firmware for per-motor LPF/HPF")
	}
	if !p.autoConfigured || p.filterMask[i] != 3 || time.Since(p.filterSeen[i]) > 2*time.Second {
		return fmt.Errorf("waiting for motor filter readback")
	}
	if p.diagnostics || p.jogActiveLocked() || p.disabled&(1<<uint(i)) == 0 {
		return fmt.Errorf("pause to disable the motor and turn jog mode off before changing filters")
	}
	return p.writeLocked(filterFrames(s.Joint, s.LPFHz, s.HPFHz))
}
