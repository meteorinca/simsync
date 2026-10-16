package serial

import (
	"bytes"
	"testing"
	"time"
)

func restoreForTest(p *Port, c *capturePort) {
	for i := range p.settings {
		p.settingsMask[i] = 63
		p.settingsSeen[i] = time.Now()
	}
	for n := 0; n < 6; n++ {
		before := len(c.data)
		p.restoreStartupSettings()
		for offset := before; offset+5 <= len(c.data); offset += 5 {
			f := c.data[offset : offset+5]
			if f[1] >= 'D' && f[1] <= 'X' || f[1] >= '!' && f[1] <= '&' {
				p.parseFrame(f)
			}
		}
	}
}

func TestStartupRestoresPerMotorFiltersOnlyWithCapableFirmware(t *testing.T) {
	p := New("", 0, nil)
	c := &capturePort{}
	connected(p, c)
	p.autoConfigured = false
	p.startup[1].LPFHz, p.startup[1].HPFHz = 80, 2
	// An old controller must not become configured while dropping saved filters.
	restoreForTest(p, c)
	if p.autoConfigured || p.startupApplied[1] {
		t.Fatal("old firmware silently dropped required filters")
	}
	p.parseFrame(frame('q', 7))
	restoreForTest(p, c)
	if !p.autoConfigured || p.Motors()[1].LPFHz != 80 || p.Motors()[1].HPFHz != 2 {
		t.Fatal(p.Motors())
	}
	if p.Motors()[0].LPFHz != 0 || !bytes.Contains(c.data, filterFrames(2, 80, 2)) {
		t.Fatal("per-motor filter restore mismatch", c.data)
	}
	for offset := 0; offset+5 <= len(c.data); offset += 5 {
		if id := c.data[offset+1]; id == 'e' || id == 'h' || id >= 'A' && id <= 'C' {
			t.Fatal("filter restore must not enable or move motors")
		}
	}
}
func TestStartupRestoresSavedProfileWithoutMotion(t *testing.T) {
	p := New("", 0, nil)
	c := &capturePort{}
	connected(p, c)
	motors := p.startup
	motors[0].Kp = 23.5
	motors[0].Clip = 60
	motors[0].Min = 70
	motors[0].Ks = 4
	motors[1].Active = true
	motors[1].Center = 350
	timing := ControllerTuning{Joint: 1, Ks: 4, Divider: 2, PWM12: 20, PWM3: 4}
	if err := p.SetStartupProfile(motors, timing); err != nil {
		t.Fatal(err)
	}
	p.autoConfigured = false
	p.jogSupported = true
	p.disabled = 7
	p.settings[0] = MotorSettings{Ki: .4, PWMMax: 100, PWMRev: 200}
	if p.Enable(1) == nil || p.SetMotorPID(1, 20, 0, .4) == nil || p.SavePID() == nil {
		t.Fatal("accepted command before settings restoration")
	}
	restoreForTest(p, c)
	s := p.Motors()[0]
	if err := s.Validate(); err != nil {
		t.Fatal(err)
	}
	if !s.Configured || !s.Active || s.Center != 338 || s.Kp != 23.5 || s.Clip != 60 || s.Min != 70 || s.Ks != 4 {
		t.Fatal(s)
	}
	if !bytes.HasPrefix(c.data, []byte("[stp]")) || !bytes.Contains(c.data, frame('Z', 2<<8)) || !bytes.Contains(c.data, frame('~', 20<<8|4)) {
		t.Fatal(c.data)
	}
	for offset := 0; offset < len(c.data); offset += 5 {
		id := c.data[offset+1]
		if id >= 'A' && id <= 'C' || id == 'e' || id == 's' && string(c.data[offset:offset+5]) != "[stp]" {
			t.Fatal("startup commanded motion or EEPROM", c.data)
		}
	}
	before := len(c.data)
	p.restoreStartupSettings()
	if len(c.data) != before {
		t.Fatal("startup overwrote later tuning")
	}
	// Change the desired profile and simulate reconnect. Retain inclusion/bounds.
	motors[0].Kp = 27
	if err := p.SetStartupProfile(motors, timing); err != nil {
		t.Fatal(err)
	}
	p.autoConfigured = false
	p.startupApplied = [3]bool{}
	p.startupStopped = false
	p.startupTimerApplied = false
	restoreForTest(p, c)
	if p.Motors()[0].Kp != 27 || !p.Motors()[1].Active || p.Motors()[1].Center != 350 {
		t.Fatal("reconnect lost settings")
	}
	p.joints[0].Actual = 700
	if p.Enable(1) == nil {
		t.Fatal("enabled outside saved bounds")
	}
}
