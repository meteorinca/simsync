package washout

import (
	"math"
	"sync"
)

// Input holds normalized/raw vehicle dynamics extracted from game UDP telemetry.
type Input struct {
	SurgeG  float64 // Longitudinal G (positive = accel, negative = braking)
	SwayG   float64 // Lateral G (positive = right turn)
	HeaveG  float64 // Vertical acceleration / bump energy
	Pitch   float64 // Chassis pitch angle (radians, nose up positive)
	Roll    float64 // Chassis roll angle (radians, right down positive)
	SuspAvg float64 // Mean suspension velocity/travel cue
	SuspLR  float64 // Left-Right suspension differential (bump roll texture)
	SuspFB  float64 // Front-Rear suspension differential (bump pitch texture)
	YawRate float64 // Optional traction loss / yaw slip cue for 3DOF rear axis
}

// Config holds live-tunable classical washout & geometry parameters.
type Config struct {
	DOFMode int `json:"dof_mode"` // 2 = 2DOF (M1+M2), 3 = 3DOF (M1+M2+M3)
	// Optional 3rd motor role when DOFMode == 3: "heave" (3-point platform) or "traction" (rear traction loss)
	Motor3Role string `json:"motor3_role"`

	// Input scaling (telemetry units that map to ~1.0 before soft-knee saturation)
	SurgeMaxG float64 `json:"surge_max_g"`
	SwayMaxG  float64 `json:"sway_max_g"`
	HeaveMax  float64 `json:"heave_max"`
	SoftKnee  float64 `json:"soft_knee"` // tanh curve stiffness (0.5..3.0, 1.2 default)

	// Classical washout channel gains (0..1 contribution into axis mix)
	SurgeHPGain float64 `json:"surge_hp_gain"` // Sharp braking/accel kick
	SurgeLPGain float64 `json:"surge_lp_gain"` // Sustained tilt coordination
	SwayHPGain  float64 `json:"sway_hp_gain"`  // Sharp cornering snap
	SwayLPGain  float64 `json:"sway_lp_gain"`  // Sustained lateral lean
	HeaveGain   float64 `json:"heave_gain"`    // Gravel / landing jolts
	BumpRoll    float64 `json:"bump_roll"`     // Left-right suspension bump texture
	BumpPitch   float64 `json:"bump_pitch"`    // Front-rear suspension pitch texture
	PitchAngle  float64 `json:"pitch_angle"`   // Direct hill/slope pitch contribution
	RollAngle   float64 `json:"roll_angle"`    // Direct camber/bank roll contribution
	TractionGain float64 `json:"traction_gain"`

	// Washout filter frequencies (Hz)
	SurgeHPHz  float64 `json:"surge_hp_hz"`  // e.g. 0.6 Hz
	SurgeLPHz  float64 `json:"surge_lp_hz"`  // e.g. 1.2 Hz
	SwayHPHz   float64 `json:"sway_hp_hz"`   // e.g. 0.6 Hz
	SwayLPHz   float64 `json:"sway_lp_hz"`   // e.g. 1.2 Hz
	HeaveHPHz  float64 `json:"heave_hp_hz"`  // e.g. 1.5 Hz
	HeaveLPHz  float64 `json:"heave_lp_hz"`  // e.g. 18.0 Hz
	OutputLPHz float64 `json:"output_lp_hz"` // Final smoothing LPF (e.g. 25 Hz)

	// Axis mix weights & master gain
	MasterGain float64 `json:"master_gain"` // 0..1 overall intensity
	PitchMix   float64 `json:"pitch_mix"`   // 0..1
	RollMix    float64 `json:"roll_mix"`    // 0..1
	HeaveMix   float64 `json:"heave_mix"`   // 0..1

	// Crank-arm geometry linearization (asin compensation for rotary lever arms)
	CrankLinearize bool    `json:"crank_linearize"`
	CrankMaxDeg    float64 `json:"crank_max_deg"` // Typical ±45 to ±60 degrees

	// Output 12-bit motor centers & safe travel amplitude (0..4095 counts)
	Center12    [3]float64 `json:"center_12"`    // Default 2048 (or 1352 matching SimSync 338*4)
	Travel12    [3]float64 `json:"travel_12"`    // Max excursion ±counts from center
	MinBound12  [3]float64 `json:"min_bound_12"` // Hard host software min (0..4095)
	MaxBound12  [3]float64 `json:"max_bound_12"` // Hard host software max (0..4095)
}

// DefaultConfig returns a crisp, safe starting tune for a 2DOF (upgradeable to 3DOF) rally rig.
func DefaultConfig() Config {
	return Config{
		DOFMode:        2,
		Motor3Role:     "heave",
		SurgeMaxG:      1.8,
		SwayMaxG:       1.8,
		HeaveMax:       1.5,
		SoftKnee:       1.25,
		SurgeHPGain:    0.55,
		SurgeLPGain:    0.45,
		SwayHPGain:     0.55,
		SwayLPGain:     0.45,
		HeaveGain:      0.40,
		BumpRoll:       0.20,
		BumpPitch:      0.20,
		PitchAngle:     0.15,
		RollAngle:      0.15,
		TractionGain:   0.60,
		SurgeHPHz:      0.65,
		SurgeLPHz:      1.20,
		SwayHPHz:       0.65,
		SwayLPHz:       1.20,
		HeaveHPHz:      1.50,
		HeaveLPHz:      20.0,
		OutputLPHz:     28.0,
		MasterGain:     0.65,
		PitchMix:       0.75,
		RollMix:        0.75,
		HeaveMix:       0.50,
		CrankLinearize: true,
		CrankMaxDeg:    50.0,
		Center12:       [3]float64{2048, 2048, 2048},
		Travel12:       [3]float64{900, 900, 900},
		MinBound12:     [3]float64{240, 240, 240},
		MaxBound12:     [3]float64{3855, 3855, 3855},
	}
}

// Output holds the computed normalized (-1..+1) and 12-bit (0..4095) motor targets.
type Output struct {
	Norm     [3]float64 // -1.0 .. +1.0 after crank linearization
	Target12 [3]uint16  // 0 .. 4095 clamped to host bounds
	PitchCmd float64
	RollCmd  float64
	HeaveCmd float64
}

// Engine runs the classical washout filter bank at a fixed output tick rate (e.g. 500 Hz).
type Engine struct {
	mu       sync.RWMutex
	cfg      Config
	sampleHz float64

	surgeHP  Biquad
	surgeLP  Biquad
	swayHP   Biquad
	swayLP   Biquad
	heaveHP  Biquad
	heaveLP  Biquad
	outLP    [3]Biquad
}

func NewEngine(cfg Config, sampleHz float64) *Engine {
	e := &Engine{cfg: cfg, sampleHz: sampleHz}
	e.reconfigureLocked()
	return e
}

func (e *Engine) SetConfig(cfg Config) {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.cfg = cfg
	e.reconfigureLocked()
}

func (e *Engine) GetConfig() Config {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.cfg
}

func (e *Engine) Reset() {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.surgeHP.Reset()
	e.surgeLP.Reset()
	e.swayHP.Reset()
	e.swayLP.Reset()
	e.heaveHP.Reset()
	e.heaveLP.Reset()
	for i := range e.outLP {
		e.outLP[i].Reset()
	}
}

func (e *Engine) reconfigureLocked() {
	c := e.cfg
	fs := e.sampleHz
	e.surgeHP.ConfigureHighPass(c.SurgeHPHz, fs)
	e.surgeLP.ConfigureLowPass(c.SurgeLPHz, fs)
	e.swayHP.ConfigureHighPass(c.SwayHPHz, fs)
	e.swayLP.ConfigureLowPass(c.SwayLPHz, fs)
	e.heaveHP.ConfigureHighPass(c.HeaveHPHz, fs)
	e.heaveLP.ConfigureLowPass(c.HeaveLPHz, fs)
	for i := range e.outLP {
		e.outLP[i].ConfigureLowPass(c.OutputLPHz, fs)
	}
}

// softKnee smoothly saturates x into (-1, +1) using tanh so massive jumps never slam stops.
func softKnee(x, stiffness float64) float64 {
	if stiffness <= 0.05 {
		return clampF(x, -1, 1)
	}
	return math.Tanh(x * stiffness) / math.Tanh(stiffness)
}

// crankLinearize compensates for circular crank-arm geometry: vertical height y = R*sin(theta),
// so desired linear height u in [-1..1] maps to rotary angle theta = asin(u * sin(maxRad)) / maxRad.
func crankLinearize(u, maxDeg float64) float64 {
	u = clampF(u, -1, 1)
	if maxDeg < 5 {
		maxDeg = 5
	}
	if maxDeg > 85 {
		maxDeg = 85
	}
	maxRad := maxDeg * math.Pi / 180.0
	sinMax := math.Sin(maxRad)
	return math.Asin(clampF(u*sinMax, -1, 1)) / maxRad
}

// Step computes one washout + axis-mix + geometry-linearized frame (zero heap allocations).
func (e *Engine) Step(in Input) Output {
	e.mu.Lock()
	defer e.mu.Unlock()
	c := e.cfg

	// 1. Normalize & soft-knee saturate raw G-forces
	surgeNorm := 0.0
	if c.SurgeMaxG > 0.01 {
		surgeNorm = softKnee(in.SurgeG/c.SurgeMaxG, c.SoftKnee)
	}
	swayNorm := 0.0
	if c.SwayMaxG > 0.01 {
		swayNorm = softKnee(in.SwayG/c.SwayMaxG, c.SoftKnee)
	}
	heaveNorm := 0.0
	if c.HeaveMax > 0.01 {
		heaveNorm = softKnee((in.HeaveG+in.SuspAvg)/c.HeaveMax, c.SoftKnee)
	}

	// 2. Classical Washout Filter Bank
	//    HPF = crisp onset kick that washes back to center; LPF = sustained tilt coordination
	surgeCue := e.surgeHP.Step(surgeNorm)*c.SurgeHPGain + e.surgeLP.Step(surgeNorm)*c.SurgeLPGain
	swayCue := e.swayHP.Step(swayNorm)*c.SwayHPGain + e.swayLP.Step(swayNorm)*c.SwayLPGain
	heaveCue := e.heaveLP.Step(e.heaveHP.Step(heaveNorm)) * c.HeaveGain

	// Combine pitch & roll cues (braking = negative surge -> nose pitches down; right turn = lean left)
	pitchCmd := softKnee(
		(-surgeCue+in.Pitch*c.PitchAngle+in.SuspFB*c.BumpPitch)*c.PitchMix,
		c.SoftKnee,
	)
	rollCmd := softKnee(
		(swayCue+in.Roll*c.RollAngle+in.SuspLR*c.BumpRoll)*c.RollMix,
		c.SoftKnee,
	)
	heaveCmd := softKnee(heaveCue*c.HeaveMix, c.SoftKnee)

	// 3. 2DOF / 3DOF Mixing Matrix
	var raw [3]float64
	if c.DOFMode <= 2 {
		// Classic 2DOF seat mover: M1 = Pitch + Roll (+ subtle heave jolt), M2 = Pitch - Roll (+ subtle heave jolt)
		raw[0] = pitchCmd + rollCmd + heaveCmd*0.5
		raw[1] = pitchCmd - rollCmd + heaveCmd*0.5
		raw[2] = 0
	} else if c.Motor3Role == "traction" {
		// 2DOF seat mover + 3rd motor dedicated to Rear Traction Loss (yaw slip)
		raw[0] = pitchCmd + rollCmd + heaveCmd*0.4
		raw[1] = pitchCmd - rollCmd + heaveCmd*0.4
		raw[2] = softKnee(in.YawRate*c.TractionGain, c.SoftKnee)
	} else {
		// Full 3DOF 3-point platform: M1 Front-Left, M2 Front-Right, M3 Rear
		raw[0] = pitchCmd + rollCmd + heaveCmd
		raw[1] = pitchCmd - rollCmd + heaveCmd
		raw[2] = -pitchCmd + heaveCmd
	}

	var out Output
	out.PitchCmd = pitchCmd
	out.RollCmd = rollCmd
	out.HeaveCmd = heaveCmd

	master := clampF(c.MasterGain, 0, 1.5)
	for i := 0; i < 3; i++ {
		u := softKnee(raw[i]*master, c.SoftKnee)
		u = e.outLP[i].Step(u)
		if c.CrankLinearize {
			u = crankLinearize(u, c.CrankMaxDeg)
		}
		u = clampF(u, -1, 1)
		out.Norm[i] = u

		pos := c.Center12[i] + u*c.Travel12[i]
		pos = clampF(pos, c.MinBound12[i], c.MaxBound12[i])
		pos = clampF(pos, 0, 4095)
		out.Target12[i] = uint16(math.Round(pos))
	}

	return out
}

func clampF(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
