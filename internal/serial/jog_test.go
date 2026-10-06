package serial

import (
	"bytes"
	"testing"
	"time"
)

func jogPort() (*Port, *capturePort) {
	p := New("", 0, nil)
	c := &capturePort{}
	connected(p, c)
	p.jogSupported = true
	p.diagnostics = true
	p.settings[0] = MotorSettings{Configured: true, Active: true, Min: 480, Max: 544, Center: 512, PWMMax: 40}
	p.settingsMask[0] = 63
	p.settingsSeen[0] = time.Now()
	p.joints[0].Actual = 512
	return p, c
}

func TestJogSignedPulseAndExclusion(t *testing.T) {
	p, c := jogPort()
	if err := p.Jog(1, -40, 150); err != nil {
		t.Fatal(err)
	}
	want := append(append(frame('4', 480), frame('7', 544)...), []byte{'[', 'j', 216, 15, ']'}...)
	if !bytes.Equal(c.data, want) {
		t.Fatalf("%v", c.data)
	}
	c.data = nil
	if p.Jog(1, 40, 150) == nil || p.SetTarget(1, 512) == nil || p.SetMotorPID(1, 1, 0, 0) == nil || p.Enable(1) == nil || p.SavePID() == nil {
		t.Fatal("accepted command during jog")
	}
	if len(c.data) != 0 {
		t.Fatal("wrote concurrent command")
	}
	if err := p.StopJog(); err != nil {
		t.Fatal(err)
	}
	if string(c.data) != "[stp]" {
		t.Fatal(c.data)
	}
}

func TestJogRejectsUnsupportedStaleAndLimits(t *testing.T) {
	for _, tc := range []struct{ pwm, ms int }{{0, 150}, {101, 150}, {41, 150}, {-41, 150}, {40, 501}, {40, 149}, {40, 40}} {
		p, c := jogPort()
		if p.Jog(1, tc.pwm, tc.ms) == nil || len(c.data) != 0 {
			t.Fatal(tc)
		}
	}
	for _, change := range []func(*Port){func(p *Port) { p.jogSupported = false }, func(p *Port) { p.seen[0] = time.Time{} }, func(p *Port) { p.joints[0].Actual = 540 }, func(p *Port) { p.settings[0].Configured = false }} {
		p, c := jogPort()
		change(p)
		if p.Jog(1, 40, 150) == nil || len(c.data) != 0 {
			t.Fatal("invalid jog accepted")
		}
	}
}

func TestJogCapability(t *testing.T) {
	p := New("", 0, nil)
	p.parseFrame(frame('q', 1))
	if !p.Motors()[0].JogSupported {
		t.Fatal("missing capability")
	}
}

func TestJogModeToggleAndPIDExclusion(t *testing.T) {
	p, c := jogPort()
	p.diagnostics = false
	if p.Jog(1, 40, 150) == nil {
		t.Fatal("jog accepted without toggle")
	}
	if err := p.SetDiagnostics(true); err != nil {
		t.Fatal(err)
	}
	if string(c.data) != "[stp]" {
		t.Fatal("toggle did not stop motors")
	}
	if p.Enable(1) == nil || p.SetTarget(1, 512) == nil {
		t.Fatal("PID allowed in jog mode")
	}
	if err := p.Jog(1, 40, 150); err != nil {
		t.Fatal(err)
	}
	if err := p.SetDiagnostics(false); err != nil {
		t.Fatal(err)
	}
	if p.Diagnostics() || p.jogActiveLocked() {
		t.Fatal("jog mode remained active")
	}
	if p.Jog(1, 40, 150) == nil {
		t.Fatal("jog accepted after toggle off")
	}
}
