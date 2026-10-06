// Package motion transforms raw telemetry G-forces and velocity data
// into 3-DOF actuator position targets with configurable gains and limits.
package motion

import (
	"fmt"
	"math"
	"sync"
)

func (c Config) Validate() error {
	for _, v := range []float32{c.PitchGain, c.RollGain, c.HeaveGain, c.PitchLimit, c.RollLimit, c.HeaveLimit, c.FilterHz} {
		if math.IsNaN(float64(v)) || math.IsInf(float64(v), 0) || v < 0 || v > 1023 {
			return fmt.Errorf("invalid motion gain, limit or filter")
		}
	}
	for _, v := range []float32{c.PitchNeutral, c.RollNeutral, c.HeaveNeutral} {
		if math.IsNaN(float64(v)) || math.IsInf(float64(v), 0) || v < 0 || v > 1023 {
			return fmt.Errorf("invalid motor center")
		}
	}
	if c.FilterHz > 50 {
		return fmt.Errorf("filter must be 0-50 Hz")
	}
	return nil
}

// Config holds all gain and limit parameters, adjustable at runtime.
type Config struct {
	// Gains: how many ADC counts of actuator movement per unit of input
	PitchGain float32 // counts per (m/s^2 normalized)
	RollGain  float32
	HeaveGain float32

	// Neutral positions (ADC counts) - where the actuator sits at rest
	PitchNeutral float32
	RollNeutral  float32
	HeaveNeutral float32

	// Travel limits from neutral (ADC counts)
	PitchLimit float32
	RollLimit  float32
	HeaveLimit float32

	// Low-pass filter cutoff frequency (Hz); 0 = no filtering
	FilterHz float32
}

// DefaultConfig returns a safe starting point for a 3DOF rig.
func DefaultConfig() Config {
	return Config{
		PitchGain: 12.0, RollGain: 10.0, HeaveGain: 8.0,
		PitchNeutral: 512.0, RollNeutral: 512.0, HeaveNeutral: 512.0,
		PitchLimit: 20.0, RollLimit: 25.0, HeaveLimit: 15.0,
		FilterHz: 15.0,
	}
}

// Targets holds the three computed actuator positions in ADC counts.
type Targets struct {
	Pitch float32 // Motor 1, front left
	Roll  float32 // Motor 2, front right
	Heave float32 // Motor 3, rear
}

// Engine transforms telemetry into actuator targets.
type Engine struct {
	mu  sync.RWMutex
	cfg Config
	lp  *lowPass
}

// NewEngine creates an Engine with the given initial config.
func NewEngine(cfg Config) *Engine {
	return &Engine{
		cfg: cfg,
		lp:  newLowPass(cfg.FilterHz),
	}
}

// SetConfig atomically replaces the engine config.
func (e *Engine) SetConfig(cfg Config) {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.cfg = cfg
	e.lp.SetCutoff(cfg.FilterHz)
}

// GetConfig returns a snapshot of the current config.
func (e *Engine) GetConfig() Config {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.cfg
}

// Transform converts raw G-force + suspension data into joint position targets.
// dt is the elapsed time in seconds since the last call.
func (e *Engine) Transform(gLat, gLong, suspFL, suspFR, suspRL, suspRR float32, dt float32) Targets {
	e.mu.Lock()
	defer e.mu.Unlock()
	cfg := e.cfg

	// Pitch: longitudinal G - positive G (braking) tilts nose down
	rawPitch := -gLong * cfg.PitchGain
	// Roll: lateral G - positive G (right turn) tilts left
	rawRoll := gLat * cfg.RollGain
	// Heave: mean suspension travel
	meanSusp := (suspFL + suspFR + suspRL + suspRR) / 4.0
	rawHeave := meanSusp * cfg.HeaveGain

	// Apply low-pass filter
	fp := e.lp.Apply(rawPitch, rawRoll, rawHeave, dt)

	return Targets{
		Pitch: clamp(cfg.PitchNeutral+clamp(fp[0], -cfg.PitchLimit, cfg.PitchLimit)+clamp(fp[1], -cfg.RollLimit, cfg.RollLimit)+clamp(fp[2], -cfg.HeaveLimit, cfg.HeaveLimit), 0, 1023),
		Roll:  clamp(cfg.RollNeutral+clamp(fp[0], -cfg.PitchLimit, cfg.PitchLimit)-clamp(fp[1], -cfg.RollLimit, cfg.RollLimit)+clamp(fp[2], -cfg.HeaveLimit, cfg.HeaveLimit), 0, 1023),
		Heave: clamp(cfg.HeaveNeutral-clamp(fp[0], -cfg.PitchLimit, cfg.PitchLimit)+clamp(fp[2], -cfg.HeaveLimit, cfg.HeaveLimit), 0, 1023),
	}
}

func clamp(v, lo, hi float32) float32 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
