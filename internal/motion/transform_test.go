package motion

import "testing"

func TestActuatorMix(t *testing.T) {
	cfg := DefaultConfig()
	cfg.FilterHz = 0
	e := NewEngine(cfg)
	tests := []struct {
		name            string
		lat, long, susp float32
		want            Targets
	}{
		{"neutral", 0, 0, 0, Targets{338, 338, 338}},
		{"roll", 1, 0, 0, Targets{348, 328, 338}},
		{"pitch", 0, 1, 0, Targets{326, 326, 350}},
		{"heave", 0, 0, 1, Targets{346, 346, 346}},
		{"axis limits", 100, -100, 100, Targets{398, 348, 333}},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			got := e.Transform(tt.lat, tt.long, tt.susp, tt.susp, tt.susp, tt.susp, 0.016)
			if got != tt.want {
				t.Fatalf("got %+v want %+v", got, tt.want)
			}
		})
	}
	cfg.PitchNeutral = 1020
	cfg.RollNeutral = 2
	e.SetConfig(cfg)
	got := e.Transform(100, 0, 0, 0, 0, 0, 0.016)
	if got.Pitch != 1023 || got.Roll != 0 {
		t.Fatalf("ADC clamp %+v", got)
	}
}
