package pid

import (
	"fmt"
	"math"
	"time"
)

// SignalValue starts and ends sine/triangle at baseline. Square alternates
// the endpoints; its full excursion is twice the requested amplitude.
func SignalValue(mode string, phase float64) float64 {
	phase -= math.Floor(phase)
	switch mode {
	case "sine":
		return math.Sin(2 * math.Pi * phase)
	case "triangle":
		return 2 / math.Pi * math.Asin(math.Sin(2*math.Pi*phase))
	case "square":
		if phase < .5 {
			return 1
		}
		return -1
	}
	return 0
}

// RunSignal shares the step runner's exclusion/cancellation state. It never
// changes gains, and checks both endpoints before any movement. Runs are finite.
func (r *Runner) RunSignal(joint int, mode string, amplitude, period float32) error {
	if joint < 1 || joint > 3 || (mode != "sine" && mode != "triangle" && mode != "square") || math.IsNaN(float64(amplitude)) || amplitude < 1 || amplitude > 20 || math.IsNaN(float64(period)) || period < .5 || period > 3 {
		return fmt.Errorf("require motor 1-3, sine/triangle/square, amplitude 1-20 counts, period 0.5-3 seconds")
	}
	r.mu.Lock()
	if r.running {
		r.mu.Unlock()
		return fmt.Errorf("test already running")
	}
	r.running = true
	r.cancel = make(chan struct{})
	r.mu.Unlock()
	defer func() { r.mu.Lock(); r.running = false; r.mu.Unlock() }()
	baseline, err := r.GetAngle(joint)
	if err != nil {
		return err
	}
	if r.CheckTarget == nil {
		return fmt.Errorf("target validation required")
	}
	for _, v := range []float32{baseline - amplitude, baseline + amplitude} {
		if err := r.CheckTarget(joint, v); err != nil {
			return err
		}
	}
	start := time.Now()
	// Whole cycles, with a maximum of three seconds per run.
	duration := time.Duration(math.Floor(3/float64(period)) * float64(period) * float64(time.Second))
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	for time.Since(start) < duration {
		if r.cancelled() {
			return fmt.Errorf("signal cancelled; no return movement")
		}
		if _, err := r.GetAngle(joint); err != nil {
			return err
		}
		value := baseline + amplitude*float32(SignalValue(mode, time.Since(start).Seconds()/float64(period)))
		if err := r.CheckTarget(joint, value); err != nil {
			return err
		}
		if err := r.SetTarget(joint, value); err != nil {
			return err
		}
		select {
		case <-r.cancel:
			return fmt.Errorf("signal cancelled; no return movement")
		case <-ticker.C:
		}
	}
	if r.cancelled() {
		return fmt.Errorf("signal cancelled; no return movement")
	}
	return r.SetTarget(joint, baseline)
}
