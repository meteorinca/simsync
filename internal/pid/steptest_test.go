package pid

import (
	"fmt"
	"testing"
	"time"
)

func TestAnalyzeDirectionsAndUnsettled(t *testing.T) {
	start := time.Now()
	for _, direction := range []float32{1, -1} {
		var samples []StepSample
		for i := 0; i < 100; i++ {
			v := float32(512)
			if i >= 10 {
				v += direction * 4
			}
			if i >= 20 {
				v += direction * 8
			}
			samples = append(samples, StepSample{T: start.Add(time.Duration(i) * 10 * time.Millisecond), Actual: v})
		}
		m := Analyze(samples, 512, 512+direction*12, start, 1, 0, 0.4)
		if !m.Success || m.RiseTimeMs != 100 || m.SettleTimeMs != 200 {
			t.Fatalf("%+v", m)
		}
		for i := range samples {
			samples[i].Actual = 512
		}
		m = Analyze(samples, 512, 512+direction*12, start, 1, 0, 0.4)
		if m.Success || m.RiseTimeMs != -1 || m.SettleTimeMs != -1 {
			t.Fatalf("stationary response scored as settled: %+v", m)
		}
	}
}

func TestPreflightBeforeGains(t *testing.T) {
	writes := 0
	r := NewRunner(func(int) (float32, error) { return 512, nil }, func(int, float32, float32, float32) error { writes++; return nil }, func(int, float32) error { writes++; return nil })
	r.CheckTarget = func(int, float32) error { return fmt.Errorf("outside limits") }
	if r.RunSingle(1, 1, 0, 0, 8) == nil || writes != 0 {
		t.Fatal("preflight wrote to controller")
	}
}

func TestCancelDoesNotReturnToBaseline(t *testing.T) {
	writes := 0
	var r *Runner
	r = NewRunner(func(int) (float32, error) { return 512, nil }, func(int, float32, float32, float32) error { return nil }, func(int, float32) error { writes++; r.Cancel(); return nil })
	if r.RunSingle(1, 1, 0, 0, 8) == nil || writes != 1 || r.IsRunning() {
		t.Fatal("cancel did not stop test")
	}
}
