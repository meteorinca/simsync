package washout

import (
	"math"
	"testing"
)

func TestBiquadHighPassWashesOutConstantG(t *testing.T) {
	var hp Biquad
	hp.ConfigureHighPass(1.0, 500.0)

	// Step input of +1.0 G: initial sample should pass strongly, then wash back near 0 after ~2s
	first := hp.Step(1.0)
	if first < 0.85 {
		t.Fatalf("expected strong onset kick, got %.4f", first)
	}
	var last float64
	for i := 0; i < 1500; i++ {
		last = hp.Step(1.0)
	}
	if math.Abs(last) > 0.02 {
		t.Fatalf("expected HPF washout near 0 after 3s, got %.4f", last)
	}
}

func TestBiquadLowPassPreservesSustainedTilt(t *testing.T) {
	var lp Biquad
	lp.ConfigureLowPass(2.0, 500.0)

	var last float64
	for i := 0; i < 1000; i++ {
		last = lp.Step(1.0)
	}
	if math.Abs(last-1.0) > 0.01 {
		t.Fatalf("expected LPF sustained DC gain ~1.0, got %.4f", last)
	}
}

func TestCrankLinearizeMonotonicAndSymmetric(t *testing.T) {
	prev := -2.0
	for i := -10; i <= 10; i++ {
		u := float64(i) / 10.0
		got := crankLinearize(u, 60.0)
		if got <= prev {
			t.Fatalf("non-monotonic at u=%.1f: prev=%.4f got=%.4f", u, prev, got)
		}
		prev = got
		sym := crankLinearize(-u, 60.0)
		if math.Abs(got+sym) > 1e-9 {
			t.Fatalf("asymmetric at u=%.1f: %.6f vs %.6f", u, got, sym)
		}
	}
	// At mid-stroke (u=0.5), asin compensation requires LESS angle than linear (since sin rises fastest near 0)
	mid := crankLinearize(0.5, 60.0)
	if mid >= 0.5 || mid <= 0.3 {
		t.Fatalf("unexpected asin mid-stroke angle %.4f", mid)
	}
}

func TestEngine2DOFAnd3DOFMixing(t *testing.T) {
	cfg := DefaultConfig()
	cfg.DOFMode = 2
	cfg.OutputLPHz = 0 // disable smoothing for exact steady-state check
	cfg.SurgeHPGain = 0
	cfg.SurgeLPGain = 1.0
	cfg.SwayHPGain = 0
	cfg.SwayLPGain = 1.0
	eng := NewEngine(cfg, 500.0)

	// Sustained lateral right turn (positive SwayG) -> M1 goes UP, M2 goes DOWN (opposite signs)
	var out Output
	for i := 0; i < 800; i++ {
		out = eng.Step(Input{SwayG: 1.2})
	}
	if !(out.Target12[0] > uint16(cfg.Center12[0]+100) && out.Target12[1] < uint16(cfg.Center12[1]-100)) {
		t.Fatalf("expected opposite M1/M2 roll response, got M1=%d M2=%d", out.Target12[0], out.Target12[1])
	}
	if out.Target12[2] != uint16(cfg.Center12[2]) {
		t.Fatalf("expected M3 centered in 2DOF mode, got %d", out.Target12[2])
	}

	// Switch to 3DOF heave/pitch platform: braking (negative SurgeG) pitches front M1/M2 UP and rear M3 DOWN
	cfg.DOFMode = 3
	cfg.Motor3Role = "heave"
	eng.SetConfig(cfg)
	eng.Reset()
	for i := 0; i < 800; i++ {
		out = eng.Step(Input{SurgeG: -1.2})
	}
	if !(out.Target12[0] > uint16(cfg.Center12[0]+100) &&
		out.Target12[1] > uint16(cfg.Center12[1]+100) &&
		out.Target12[2] < uint16(cfg.Center12[2]-100)) {
		t.Fatalf("expected 3DOF pitch mix (M1/M2 up, M3 down), got %v", out.Target12)
	}
}
