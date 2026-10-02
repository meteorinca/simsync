package motion

import "math"

// lowPass is a simple one-pole IIR filter applied per-axis.
type lowPass struct {
	cutoff float32
	prev   [3]float32
}

func newLowPass(cutoffHz float32) *lowPass {
	return &lowPass{cutoff: cutoffHz}
}

func (lp *lowPass) SetCutoff(hz float32) {
	lp.cutoff = hz
}

// Apply filters three axes given elapsed time dt (seconds).
// Returns filtered [pitch, roll, heave].
func (lp *lowPass) Apply(pitch, roll, heave float32, dt float32) [3]float32 {
	if lp.cutoff <= 0 || dt <= 0 {
		lp.prev = [3]float32{pitch, roll, heave}
		return lp.prev
	}
	// RC = 1 / (2*pi*fc)
	rc := float32(1.0 / (2.0 * math.Pi * float64(lp.cutoff)))
	alpha := dt / (rc + dt)

	in := [3]float32{pitch, roll, heave}
	for i := range in {
		lp.prev[i] = lp.prev[i] + alpha*(in[i]-lp.prev[i])
	}
	return lp.prev
}
