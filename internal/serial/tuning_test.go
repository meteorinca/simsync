package serial

import (
	"bytes"
	"testing"
	"time"
)

func TestTimingReadbackAndWrites(t *testing.T) {
	p := New("", 0, nil)
	c := &capturePort{}
	connected(p, c)
	p.parseFrame(frame('N', 5))
	p.parseFrame(frame('~', 25<<8|31))
	p.parseFrame(frame('b', 3<<12))
	if m := p.Motors()[1]; m.Ks != 5 || !m.TimingReady {
		t.Fatal(m)
	}
	s := ControllerTuning{Joint: 2, Ks: 7, Divider: 2, PWM12: 20, PWM3: 4}
	if err := p.TuneController(s); err != nil {
		t.Fatal(err)
	}
	want := append(append(frame('N', 7), frame('Z', 2<<8)...), frame('~', 20<<8|4)...)
	if !bytes.Equal(c.data, want) {
		t.Fatal(c.data)
	}
	c.data = nil
	for _, invalid := range []ControllerTuning{{Joint: 0, Ks: 7, Divider: 2, PWM12: 20, PWM3: 4}, {Joint: 2, Ks: 0, Divider: 2, PWM12: 20, PWM3: 4}, {Joint: 2, Ks: 7, Divider: 0, PWM12: 20, PWM3: 4}, {Joint: 2, Ks: 7, Divider: 2, PWM12: 0, PWM3: 4}, {Joint: 2, Ks: 7, Divider: 2, PWM12: 20, PWM3: 0}} {
		if p.TuneController(invalid) == nil {
			t.Fatal("invalid settings accepted", invalid)
		}
	}
	p.tuningSeen = time.Now().Add(-3 * time.Second)
	if p.TuneController(s) == nil || len(c.data) != 0 {
		t.Fatal("stale or invalid timing wrote bytes")
	}
}
