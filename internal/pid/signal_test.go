package pid

import (
	"fmt"
	"math"
	"testing"
)

func TestSignalPreflightAndCancellation(t *testing.T) {
	for _, mode := range []string{"sine", "triangle", "square"} {
		writes := 0
		var r *Runner
		r = NewRunner(func(int) (float32, error) { return 512, nil }, nil, func(int, float32) error { writes++; r.Cancel(); return nil })
		r.CheckTarget = func(_ int, v float32) error {
			if v > 520 {
				return fmt.Errorf("outside host bounds")
			}
			return nil
		}
		if r.RunSignal(1, mode, 12, 1) == nil || writes != 0 {
			t.Fatal("endpoint preflight allowed movement", mode)
		}
		if r.RunSignal(1, mode, 8, 1) == nil || writes != 1 || r.IsRunning() {
			t.Fatal("cancel caused extra movement", mode, writes)
		}
	}
}
func TestSignalValidationAndShape(t *testing.T) {
	writes := 0
	r := NewRunner(func(int) (float32, error) { return 512, nil }, nil, func(int, float32) error { writes++; return nil })
	for _, tc := range []struct {
		mode              string
		amplitude, period float32
	}{{"invalid", 8, 1}, {"sine", 21, 1}, {"sine", 0, 1}, {"sine", 8, 0}, {"sine", 8, 4}, {"sine", float32(math.NaN()), 1}} {
		if r.RunSignal(1, tc.mode, tc.amplitude, tc.period) == nil {
			t.Fatal(tc)
		}
	}
	if writes != 0 {
		t.Fatal("invalid signal wrote targets")
	}
	for _, mode := range []string{"sine", "triangle", "square"} {
		for i := 0; i < 100; i++ {
			v := SignalValue(mode, float64(i)/100)
			if math.Abs(v) > 1.00001 {
				t.Fatal(mode, v)
			}
		}
		if math.Abs(SignalValue(mode, .25)-1) > 0.00001 || math.Abs(SignalValue(mode, .75)+1) > 0.00001 {
			t.Fatal(mode)
		}
	}
}
