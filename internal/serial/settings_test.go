package serial

import (
	"bytes"
	"testing"
	"time"
)

func TestMotorBoundsAndSelection(t *testing.T) {
	p := New("", 0, nil)
	c := &capturePort{}
	connected(p, c)
	p.settings[0].Min = 480
	p.settings[0].Max = 544
	p.joints[0].Actual = 512
	p.settings[1].Active = false
	p.settings[2].Active = false
	if err := p.SendTargets(520, 0, 1023); err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(c.data, frame('A', 520)) {
		t.Fatalf("unexpected frames %v", c.data)
	}
	c.data = nil
	for _, v := range []float32{479, 545} {
		if p.SetTarget(1, v) == nil {
			t.Fatal("accepted out of range target")
		}
	}
	if p.SetTarget(2, 512) == nil {
		t.Fatal("accepted inactive motor")
	}
	p.seen[0] = time.Now().Add(-time.Second)
	if p.SetTarget(1, 512) == nil {
		t.Fatal("accepted stale feedback")
	}
	if len(c.data) != 0 {
		t.Fatal("invalid commands wrote bytes")
	}
}

func TestPerMotorGainsAndReadback(t *testing.T) {
	p := New("", 0, nil)
	c := &capturePort{}
	connected(p, c)
	if err := p.SetMotorPID(2, 1, 0, 0.4); err != nil {
		t.Fatal(err)
	}
	want := append(append(frame('E', 100), frame('H', 0)...), frame('K', 40)...)
	if !bytes.Equal(c.data, want) {
		t.Fatal(c.data)
	}
	for _, id := range []byte{'E', 'H', 'K', 'Q', 'T', 'W'} {
		p.parseFrame(frame(id, 100))
	}
	s := p.Motors()
	if !s[1].Ready || s[0].Ready || s[1].Kp != 1 {
		t.Fatal(s)
	}
}

func TestConfigureAndEnable(t *testing.T) {
	p := New("", 0, nil)
	c := &capturePort{}
	connected(p, c)
	p.joints[0] = JointTelemetry{Actual: 512, Target: 512}
	p.settingsMask[0] = 63
	p.settingsSeen[0] = time.Now()
	s := MotorSettings{Kp: 1, PWMMax: 40, Cutoff: 30, Clip: 50, Min: 480, Max: 544, Center: 512, Active: true}
	if err := p.Configure(1, s); err != nil {
		t.Fatal(err)
	}
	if len(c.data) != 30 || !bytes.Equal(c.data[15:20], frame('P', 40)) || !bytes.Equal(c.data[20:25], frame('S', 30<<8|50)) {
		t.Fatal(c.data)
	}
	c.data = nil
	p.disabled = 1
	if err := p.Enable(1); err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(c.data, append(frame('A', 512), []byte("[en1]")...)) {
		t.Fatal(c.data)
	}
	c.data = nil
	s.Min = 520
	if p.Configure(1, s) == nil || len(c.data) != 0 {
		t.Fatal("invalid configuration accepted")
	}
}
