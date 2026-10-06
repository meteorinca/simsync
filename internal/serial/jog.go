package serial

import (
	"fmt"
	"time"
)

func (p *Port) jogActiveLocked() bool { return time.Now().Before(p.jogUntil) }

// Jog requests a firmware-timed constant PWM pulse, independent of PID gains.
func (p *Port) Jog(j, pwm, durationMS int) error {
	if j < 1 || j > 3 || pwm == 0 || pwm < -100 || pwm > 100 || durationMS < 50 || durationMS > 500 || durationMS%10 != 0 {
		return fmt.Errorf("motor 1-3, signed PWM 1-100, duration 50-500 ms in 10 ms steps required")
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	if !p.diagnostics {
		return fmt.Errorf("enable jog mode first")
	}
	if p.jogActiveLocked() {
		return fmt.Errorf("open-loop jog running")
	}
	return p.jogLocked(j, pwm, durationMS, p.settings[j-1].Min, p.settings[j-1].Max)
}

func (p *Port) jogLocked(j, pwm, durationMS int, low, high float32) error {
	if !p.jogSupported {
		return fmt.Errorf("firmware update required for open-loop jog")
	}
	if time.Now().Before(p.jogUntil) {
		return fmt.Errorf("open-loop jog running")
	}
	i := j - 1
	s := p.settings[i]
	actual := p.joints[i].Actual
	if !s.Configured || !s.Active || time.Since(p.seen[i]) > 250*time.Millisecond || time.Since(p.settingsSeen[i]) > 2*time.Second || p.settingsMask[i] != 63 {
		return fmt.Errorf("configure motor and wait for fresh feedback")
	}
	if actual <= s.Min+4 || actual >= s.Max-4 {
		return fmt.Errorf("feedback must be at least four counts inside host bounds")
	}
	if pwm > s.PWMMax || -pwm > s.PWMMax {
		return fmt.Errorf("jog exceeds motor PWM max %d", s.PWMMax)
	}
	data := append(frame(byte('4'+i), uint16(low)), frame(byte('7'+i), uint16(high))...)
	data = append(data, []byte{'[', byte('j' + i), byte(int8(pwm)), byte(durationMS / 10), ']'}...)
	if err := p.writeLocked(data); err != nil {
		return err
	}
	p.jogUntil = time.Now().Add(time.Duration(durationMS+100) * time.Millisecond)
	p.jogMotor = j
	return nil
}

func (p *Port) StopJog() error {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.diagnostics = false
	if !p.jogSupported {
		return nil
	}
	if err := p.writeLocked([]byte("[stp]")); err != nil {
		return err
	}
	p.jogUntil = time.Time{}
	return nil
}
