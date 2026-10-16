package serial

import (
	"bytes"
	"testing"
	"time"
)

func TestTrackingGuardApproachNoiseAndDivergence(t *testing.T) {
	var g trackingGuard
	now := time.Now()
	for i, actual := range []float32{500, 512, 520, 516, 524, 520} {
		if g.update(520, actual, now.Add(time.Duration(i)*time.Second)) {
			t.Fatal("normal approach and four-count noise must not trip")
		}
	}
	if g.update(520, 548, now.Add(7*time.Second)) {
		t.Fatal("first divergent sample must start the observation window")
	}
	if !g.update(520, 552, now.Add(7100*time.Millisecond)) {
		t.Fatal("sustained divergence must trip")
	}
	if g.update(600, 552, now.Add(8*time.Second)) {
		t.Fatal("new target must start a new observation")
	}
}

func TestRunawayDisablesAllMotorsAndLatchesReason(t *testing.T) {
	p := New("", 0, nil)
	c := &capturePort{}
	connected(p, c)
	p.jogSupported = true
	p.joints[1].Actual = 512
	if err := p.Enable(2); err != nil {
		t.Fatal(err)
	}
	c.data = nil
	p.disabled = 0
	p.parseFrame([]byte{'[', 'B', 128, 128, ']'})
	p.parseFrame([]byte{'[', 'B', 134, 128, ']'})
	p.tracking[1].awaySince = time.Now().Add(-150 * time.Millisecond)
	p.parseFrame([]byte{'[', 'B', 136, 128, ']'})
	if !bytes.Equal(c.data, []byte("[stp]")) || p.MotionFault() == "" || p.disabled != 7 {
		t.Fatal("runaway must stop all outputs and report the motor", c.data, p.MotionFault())
	}
	if p.CheckTarget(2, 512) == nil {
		t.Fatal("must block further targets until fault is cleared")
	}
	p.ClearMotionFault()
	if p.MotionFault() != "" {
		t.Fatal("explicit clear must allow a new setup attempt")
	}
}
