// Package pid provides host-side automated step test and gain analysis tools.
package pid

import (
	"fmt"
	"log"
	"math"
	"sort"
	"sync"
	"time"
)

// StepMetrics holds the result of a single step response measurement.
type StepMetrics struct {
	Kp             float32
	Ki             float32
	Kd             float32
	StepSize       float32 // ADC counts
	StartAngle     float32
	TargetAngle    float32
	RiseTimeMs     float32 // time from 10% to 90% of step
	OvershootPct   float32 // peak overshoot as % of step size
	SettleTimeMs   float32 // time to stay within 2% of target
	SteadyStateErr float32 // mean error over final 500 ms
	Success        bool
	Note           string
}

// StepSample is one recorded point during a step test.
type StepSample struct {
	T      time.Time
	Actual float32
	Duty   int16
}

// AngleGetter is a function that returns the current actual angle for a joint.
type AngleGetter func(joint int) (float32, error)

// PIDSetter is a function that pushes new gains to the SMC3.
type PIDSetter func(kp, ki, kd float32) error

// TargetSetter is a function that commands a new angle target.
type TargetSetter func(joint int, angle float32) error

// Runner orchestrates automated step tests.
type Runner struct {
	mu       sync.Mutex
	running  bool
	results  []StepMetrics
	onResult func(StepMetrics)

	GetAngle  AngleGetter
	SetPID    PIDSetter
	SetTarget TargetSetter
}

// NewRunner creates a Runner. Callbacks must be set before calling Run*.
func NewRunner(get AngleGetter, setPID PIDSetter, setTarget TargetSetter) *Runner {
	return &Runner{
		GetAngle:  get,
		SetPID:    setPID,
		SetTarget: setTarget,
	}
}

// OnResult registers a callback invoked after each step completes.
func (r *Runner) OnResult(fn func(StepMetrics)) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.onResult = fn
}

// Results returns a copy of all recorded step metrics, newest first.
func (r *Runner) Results() []StepMetrics {
	r.mu.Lock()
	defer r.mu.Unlock()
	out := make([]StepMetrics, len(r.results))
	copy(out, r.results)
	sort.Slice(out, func(i, j int) bool {
		return out[i].OvershootPct < out[j].OvershootPct
	})
	return out
}

// IsRunning returns true if a test is currently active.
func (r *Runner) IsRunning() bool {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.running
}

// RunSingle performs one step test with the given PID gains on joint.
// stepSize is in ADC counts (positive).
func (r *Runner) RunSingle(joint int, kp, ki, kd, stepSize float32) error {
	r.mu.Lock()
	if r.running {
		r.mu.Unlock()
		return fmt.Errorf("step test already running")
	}
	r.running = true
	r.mu.Unlock()

	defer func() {
		r.mu.Lock()
		r.running = false
		r.mu.Unlock()
	}()

	return r.runStep(joint, kp, ki, kd, stepSize)
}

// RunSweep performs a grid sweep of Kp and Kd values, running one step test
// per combination. Results are reported via OnResult after each step.
func (r *Runner) RunSweep(joint int, kpVals, kdVals []float32, ki, stepSize float32) {
	r.mu.Lock()
	if r.running {
		r.mu.Unlock()
		return
	}
	r.running = true
	r.mu.Unlock()

	go func() {
		defer func() {
			r.mu.Lock()
			r.running = false
			r.mu.Unlock()
		}()
		for _, kp := range kpVals {
			for _, kd := range kdVals {
				if err := r.runStep(joint, kp, ki, kd, stepSize); err != nil {
					log.Printf("[pid sweep] error at kp=%.2f kd=%.2f: %v", kp, kd, err)
				}
				time.Sleep(800 * time.Millisecond) // settle between tests
			}
		}
	}()
}

func (r *Runner) runStep(joint int, kp, ki, kd, stepSize float32) error {
	// Apply gains
	if stepSize <= 0 || math.IsNaN(float64(stepSize)) || math.IsInf(float64(stepSize), 0) {
		return fmt.Errorf("step must be a finite positive count")
	}
	if err := r.SetPID(kp, ki, kd); err != nil {
		return fmt.Errorf("SetPID: %w", err)
	}
	time.Sleep(100 * time.Millisecond)

	// Read baseline angle
	baseline, err := r.GetAngle(joint)
	if err != nil {
		return fmt.Errorf("GetAngle baseline: %w", err)
	}

	target := baseline + stepSize
	if target > 1023 {
		return fmt.Errorf("step target exceeds 1023 counts")
	}

	// Command step
	stepTime := time.Now()
	if err := r.SetTarget(joint, target); err != nil {
		return fmt.Errorf("SetTarget: %w", err)
	}

	// Record response for up to 3 seconds
	var samples []StepSample
	deadline := time.Now().Add(3 * time.Second)
	for time.Now().Before(deadline) {
		actual, err := r.GetAngle(joint)
		if err != nil {
			return fmt.Errorf("feedback lost during step: %w", err)
		}
		if err == nil {
			samples = append(samples, StepSample{T: time.Now(), Actual: actual})
		}
		time.Sleep(10 * time.Millisecond)
	}

	metrics := Analyze(samples, baseline, target, stepTime, kp, ki, kd)
	r.mu.Lock()
	r.results = append(r.results, metrics)
	cb := r.onResult
	r.mu.Unlock()

	if cb != nil {
		cb(metrics)
	}

	// Return to baseline
	_ = r.SetTarget(joint, baseline)
	time.Sleep(500 * time.Millisecond)

	return nil
}

// Analyze computes performance metrics from a recorded step response.
func Analyze(samples []StepSample, baseline, target float32, stepTime time.Time, kp, ki, kd float32) StepMetrics {
	m := StepMetrics{
		Kp: kp, Ki: ki, Kd: kd,
		StepSize:   target - baseline,
		StartAngle: baseline, TargetAngle: target,
	}

	if len(samples) < 5 {
		m.Note = "insufficient samples"
		return m
	}

	step := target - baseline
	if step == 0 {
		m.Note = "zero step size"
		return m
	}

	// Find rise time (10% -> 90% of step)
	lo := baseline + step*0.10
	hi := baseline + step*0.90
	var t10, t90 float64
	found10, found90 := false, false
	peakErr := float32(0)

	for _, s := range samples {
		dt := float64(s.T.Sub(stepTime).Milliseconds())
		if !found10 && s.Actual >= lo {
			t10 = dt
			found10 = true
		}
		if !found90 && s.Actual >= hi {
			t90 = dt
			found90 = true
		}
		overshoot := s.Actual - target
		if overshoot > peakErr {
			peakErr = overshoot
		}
	}
	if found10 && found90 {
		m.RiseTimeMs = float32(t90 - t10)
	}
	m.OvershootPct = (peakErr / float32(math.Abs(float64(step)))) * 100

	// Settling time: first time actual stays within ±2% of step for 200 ms
	tolerance := float32(math.Abs(float64(step))) * 0.02
	settleWindow := 20 // 20 samples @ 10 ms = 200 ms
	for i := 0; i < len(samples)-settleWindow; i++ {
		allIn := true
		for j := i; j < i+settleWindow; j++ {
			if float32(math.Abs(float64(samples[j].Actual-target))) > tolerance {
				allIn = false
				break
			}
		}
		if allIn {
			m.SettleTimeMs = float32(samples[i].T.Sub(stepTime).Milliseconds())
			break
		}
	}

	// Steady-state error: mean |error| over last 500 ms (50 samples)
	tail := samples
	if len(tail) > 50 {
		tail = tail[len(tail)-50:]
	}
	var errSum float32
	for _, s := range tail {
		errSum += float32(math.Abs(float64(s.Actual - target)))
	}
	m.SteadyStateErr = errSum / float32(len(tail))
	m.Success = true
	return m
}
