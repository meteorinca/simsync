// Package pid provides host-side automated step test and gain analysis tools.
package pid

import (
	"fmt"
	"log"
	"math"
	"sync"
	"time"
)

// StepMetrics holds the result of a single step response measurement.
type StepMetrics struct {
	Joint          int
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
type PIDSetter func(joint int, kp, ki, kd float32) error

// TargetSetter is a function that commands a new angle target.
type TargetSetter func(joint int, angle float32) error

// Runner orchestrates automated step tests.
type Runner struct {
	mu          sync.Mutex
	running     bool
	results     []StepMetrics
	onResult    func(StepMetrics)
	cancel      chan struct{}
	CheckTarget TargetSetter

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
	for i, j := 0, len(out)-1; i < j; i, j = i+1, j-1 {
		out[i], out[j] = out[j], out[i]
	}
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
	r.cancel = make(chan struct{})
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
	r.cancel = make(chan struct{})
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
					return
				}
				time.Sleep(800 * time.Millisecond) // settle between tests
			}
		}
	}()
}

func (r *Runner) Cancel() {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.running && r.cancel != nil {
		select {
		case <-r.cancel:
		default:
			close(r.cancel)
		}
	}
}

func (r *Runner) cancelled() bool {
	r.mu.Lock()
	defer r.mu.Unlock()
	select {
	case <-r.cancel:
		return true
	default:
		return false
	}
}

func (r *Runner) runStep(joint int, kp, ki, kd, stepSize float32) error {
	// Apply gains
	if stepSize == 0 || math.Abs(float64(stepSize)) > 40 || math.IsNaN(float64(stepSize)) || math.IsInf(float64(stepSize), 0) {
		return fmt.Errorf("step must be between -40 and 40, excluding zero")
	}

	// Read baseline angle
	baseline, err := r.GetAngle(joint)
	if err != nil {
		return fmt.Errorf("GetAngle baseline: %w", err)
	}

	target := baseline + stepSize
	if target < 0 || target > 1023 {
		return fmt.Errorf("step target outside ADC range")
	}
	if r.CheckTarget != nil {
		if err := r.CheckTarget(joint, baseline); err != nil {
			return err
		}
		if err := r.CheckTarget(joint, target); err != nil {
			return err
		}
	}
	if r.cancelled() {
		return fmt.Errorf("test cancelled")
	}
	if err := r.SetPID(joint, kp, ki, kd); err != nil {
		return err
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
		if r.cancelled() {
			return fmt.Errorf("test cancelled; controller holds last target")
		}
		if r.CheckTarget != nil {
			if err := r.CheckTarget(joint, target); err != nil {
				return err
			}
		}
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
	metrics.Joint = joint
	r.mu.Lock()
	r.results = append(r.results, metrics)
	cb := r.onResult
	r.mu.Unlock()

	if cb != nil {
		cb(metrics)
	}

	// Return to baseline
	if r.cancelled() {
		return fmt.Errorf("test cancelled; controller holds last target")
	}
	if err := r.SetTarget(joint, baseline); err != nil {
		return fmt.Errorf("return to baseline: %w", err)
	}
	time.Sleep(500 * time.Millisecond)

	return nil
}

// Analyze computes performance metrics from a recorded step response.
func Analyze(samples []StepSample, baseline, target float32, stepTime time.Time, kp, ki, kd float32) StepMetrics {
	m := StepMetrics{
		RiseTimeMs: -1, SettleTimeMs: -1,
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
	direction := float32(1)
	if step < 0 {
		direction = -1
	}
	lo := float32(math.Abs(float64(step))) * 0.10
	hi := float32(math.Abs(float64(step))) * 0.90
	var t10, t90 float64
	found10, found90 := false, false
	peakErr := float32(0)

	for _, s := range samples {
		dt := float64(s.T.Sub(stepTime).Milliseconds())
		progress := (s.Actual - baseline) * direction
		if !found10 && progress >= lo {
			t10 = dt
			found10 = true
		}
		if !found90 && progress >= hi {
			t90 = dt
			found90 = true
		}
		overshoot := (s.Actual - target) * direction
		if overshoot > peakErr {
			peakErr = overshoot
		}
	}
	if found10 && found90 {
		m.RiseTimeMs = float32(t90 - t10)
	}
	m.OvershootPct = (peakErr / float32(math.Abs(float64(step)))) * 100

	// Settling time: first time actual stays within ±2% of step for 200 ms
	tolerance := max(float32(4), float32(math.Abs(float64(step)))*0.02)
	lastOutside := -1
	for i, s := range samples {
		if float32(math.Abs(float64(s.Actual-target))) > tolerance {
			lastOutside = i
		}
	}
	first := lastOutside + 1
	if first < len(samples) && samples[len(samples)-1].T.Sub(samples[first].T) >= 200*time.Millisecond {
		m.SettleTimeMs = float32(samples[first].T.Sub(stepTime).Milliseconds())
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
	m.Success = found90 && m.SettleTimeMs >= 0
	if !m.Success {
		m.Note = "target not reached or not settled"
	}
	return m
}
