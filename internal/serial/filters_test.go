package serial

import (
	"bytes"
	"math"
	"testing"
	"time"
)

func TestFiltersRequireCapabilityPauseAndFreshReadback(t *testing.T) {
	p := New("", 0, nil)
	c := &capturePort{}
	connected(p, c)
	s := MotorFilters{Joint: 2, LPFHz: 80, HPFHz: 2}
	if p.SetMotorFilters(s) == nil {
		t.Fatal("old firmware cannot accept filters")
	}
	p.parseFrame(frame('q', 7))
	p.parseFrame(frame('"', 600))
	p.parseFrame(frame('%', 10))
	m := p.Motors()[1]
	if !m.FiltersSupported || !m.FiltersReady || m.LPFHz != 60 || m.HPFHz != 1 || p.Motors()[0].FiltersReady {
		t.Fatal(m)
	}
	if p.SetMotorFilters(s) == nil {
		t.Fatal("must not change filters on an enabled motor")
	}
	p.disabled = 7
	if err := p.SetMotorFilters(s); err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(c.data, append(frame('"', 800), frame('%', 20)...)) {
		t.Fatal(c.data)
	}
	if p.Motors()[1].LPFHz != 60 {
		t.Fatal("writes must not manufacture readback")
	}
	c.data = nil
	p.filterSeen[1] = time.Now().Add(-3 * time.Second)
	if p.SetMotorFilters(s) == nil || len(c.data) != 0 {
		t.Fatal("stale filters accepted")
	}
}

func TestFilterValidation(t *testing.T) {
	for _, s := range []MotorFilters{{Joint: 0}, {Joint: 1, LPFHz: -1}, {Joint: 1, LPFHz: 201}, {Joint: 1, HPFHz: 51}, {Joint: 1, LPFHz: float32(math.NaN())}, {Joint: 1, HPFHz: float32(math.Inf(1))}, {Joint: 1, LPFHz: .05}, {Joint: 1, LPFHz: 1.15}, {Joint: 1, LPFHz: 1, HPFHz: 1}} {
		if s.Validate() == nil {
			t.Fatal("invalid filters accepted", s)
		}
	}
	if err := (MotorFilters{Joint: 3, LPFHz: 0, HPFHz: 0}).Validate(); err != nil {
		t.Fatal(err)
	}
}

func TestSoloStopsOthersAndRestrictsTargetsWithoutChangingProfile(t *testing.T) {
	p := New("", 0, nil)
	c := &capturePort{}
	connected(p, c)
	p.jogSupported = true
	p.holdSupported = true
	p.joints[0].Actual, p.joints[1].Actual, p.joints[2].Actual = 512, 508, 512
	before := p.settings
	if err := p.EnableOnly(2); err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(c.data, append([]byte("[stp]"), frame('h', 2)...)) {
		t.Fatal(c.data)
	}
	if p.settings != before || !p.Motors()[1].Solo || p.Motors()[0].Solo {
		t.Fatal("solo changed saved motor settings")
	}
	p.disabled = 0 // Even misleading status must not authorize another motor.
	if p.SetTarget(1, 512) == nil {
		t.Fatal("solo allowed a different motor")
	}
	if err := p.SetTarget(2, 512); err != nil {
		t.Fatal(err)
	}
	if err := p.StopJog(); err != nil {
		t.Fatal(err)
	}
	if p.soloMotor != 0 {
		t.Fatal("pause must leave solo mode")
	}
}
