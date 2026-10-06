package serial

import (
	"bytes"
	"testing"
	"time"
)

func TestBenchStartupReplacesLegacySettings(t *testing.T) {
	p := New("", 0, nil)
	c := &capturePort{}
	connected(p, c)
	p.settings[0] = MotorSettings{Ki: 0.4, PWMMax: 100, PWMRev: 200}
	p.settingsMask[0] = 63
	p.settingsSeen[0] = time.Now()
	p.joints[0].Actual = 512
	p.disabled = 7
	p.autoConfigureBench()
	s := p.Motors()[0]
	if err := s.Validate(); err != nil {
		t.Fatal(err)
	}
	if !s.Configured || !s.Active || s.Min != 50 || s.Max != 650 || s.Ki != 0 || s.PWMMax != 100 || s.PWMRev != 50 {
		t.Fatal(s)
	}
	if !bytes.Contains(c.data, frame('G', 0)) || !bytes.Contains(c.data, frame('V', 50)) {
		t.Fatalf("missing corrected controller writes: %v", c.data)
	}
	before := len(c.data)
	p.autoConfigureBench()
	if len(c.data) != before {
		t.Fatal("startup overwrites later tuning")
	}
	if err := p.Enable(1); err != nil {
		t.Fatal(err)
	}
	if !bytes.HasSuffix(c.data, []byte("[en1]")) {
		t.Fatal("missing enable")
	}
	p.joints[0].Actual = 700
	if p.Enable(1) == nil {
		t.Fatal("enabled outside requested bounds")
	}
}
