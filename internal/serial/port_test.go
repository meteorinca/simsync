package serial

import (
	"bytes"
	goserial "go.bug.st/serial"
	"math"
	"testing"
	"time"
)

type capturePort struct {
	goserial.Port
	data  []byte
	chunk int
}

func (p *capturePort) Write(b []byte) (int, error) {
	n := len(b)
	if p.chunk > 0 && n > p.chunk {
		n = p.chunk
	}
	p.data = append(p.data, b[:n]...)
	return n, nil
}
func connected(p *Port, c *capturePort) { p.port = c; p.ready = true; p.lastReply = time.Now() }
func TestNativeTargetsAndPartialWrites(t *testing.T) {
	p := New("COM1", 0, nil)
	c := &capturePort{chunk: 2}
	connected(p, c)
	if err := p.SendTargets(0, 512, 1023); err != nil {
		t.Fatal(err)
	}
	want := []byte{'[', 'A', 0, 0, ']', '[', 'B', 2, 0, ']', '[', 'C', 3, 255, ']'}
	if !bytes.Equal(c.data, want) {
		t.Fatalf("got %v want %v", c.data, want)
	}
	for _, v := range []float32{-1, 1024, float32(math.NaN()), float32(math.Inf(1))} {
		if p.SetTarget(1, v) == nil {
			t.Fatalf("accepted %v", v)
		}
	}
}
func TestFeedbackBinaryPayload(t *testing.T) {
	var got JointTelemetry
	p := New("", 0, func(j JointTelemetry) { got = j })
	p.parseFrame([]byte{'[', 'v', 0, 70, ']'})
	p.parseFrame([]byte{'[', 'a', 0x21, 200, ']'})
	// Delimiter bytes are valid payload bytes, not frame boundaries.
	p.parseFrame([]byte{'[', 'A', '[', ']', ']'})
	if got.Actual != 364 || got.Target != 372 || got.Error != 8 || got.Duty != 200 {
		t.Fatalf("feedback: %+v", got)
	}
	if p.disabled != 1 {
		t.Fatalf("disabled mask %d", p.disabled)
	}
	if v, err := p.Actual(1); err != nil || v != 364 {
		t.Fatalf("actual %v %v", v, err)
	}
	p.seen[0] = time.Now().Add(-time.Second)
	if _, err := p.Actual(1); err == nil {
		t.Fatal("accepted stale feedback")
	}
}
func TestGainsAndSave(t *testing.T) {
	p := New("", 0, nil)
	c := &capturePort{}
	connected(p, c)
	if err := p.SetPID(4.25, 0.1, 1.5); err != nil {
		t.Fatal(err)
	}
	if len(c.data) != 45 || !bytes.Equal(c.data[:5], []byte{'[', 'D', 1, 169, ']'}) || !bytes.Equal(c.data[15:20], []byte{'[', 'G', 0, 10, ']'}) {
		t.Fatalf("PID frames %v", c.data)
	}
	if err := p.SavePID(); err != nil {
		t.Fatal(err)
	}
	if string(c.data[45:]) != "[sav]" {
		t.Fatal("wrong EEPROM command")
	}
	p.parseFrame([]byte{'[', 'D', 1, 169, ']'})
	kp, _, _ := p.PID()
	if kp != 4.25 {
		t.Fatal(kp)
	}
	p.ready = false
	if p.SetTarget(2, 512) == nil {
		t.Fatal("command before handshake")
	}
}
