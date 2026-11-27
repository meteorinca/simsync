package washout

import "math"

// Biquad is a zero-allocation 2nd-order IIR filter (Direct Form II Transposed).
type Biquad struct {
	b0, b1, b2 float64
	a1, a2     float64
	z1, z2     float64
	active     bool
}

// ConfigureLowPass sets up a 2nd-order Butterworth low-pass filter (Q = 1/sqrt(2)).
func (b *Biquad) ConfigureLowPass(cutoffHz, sampleHz float64) {
	if cutoffHz <= 0 || sampleHz <= 0 || cutoffHz >= sampleHz*0.45 {
		b.active = false
		return
	}
	w0 := 2.0 * math.Pi * cutoffHz / sampleHz
	cosw0 := math.Cos(w0)
	sinw0 := math.Sin(w0)
	alpha := sinw0 / math.Sqrt2

	a0 := 1.0 + alpha
	b.b0 = ((1.0 - cosw0) * 0.5) / a0
	b.b1 = (1.0 - cosw0) / a0
	b.b2 = b.b0
	b.a1 = (-2.0 * cosw0) / a0
	b.a2 = (1.0 - alpha) / a0
	b.active = true
}

// ConfigureHighPass sets up a 2nd-order Butterworth high-pass washout filter.
func (b *Biquad) ConfigureHighPass(cutoffHz, sampleHz float64) {
	if cutoffHz <= 0 || sampleHz <= 0 || cutoffHz >= sampleHz*0.45 {
		b.active = false
		return
	}
	w0 := 2.0 * math.Pi * cutoffHz / sampleHz
	cosw0 := math.Cos(w0)
	sinw0 := math.Sin(w0)
	alpha := sinw0 / math.Sqrt2

	a0 := 1.0 + alpha
	b.b0 = ((1.0 + cosw0) * 0.5) / a0
	b.b1 = -(1.0 + cosw0) / a0
	b.b2 = b.b0
	b.a1 = (-2.0 * cosw0) / a0
	b.a2 = (1.0 - alpha) / a0
	b.active = true
}

// ConfigureBandPass sets up a constant-0dB-peak-gain band-pass filter for heave/gravel cues.
func (b *Biquad) ConfigureBandPass(centerHz, q, sampleHz float64) {
	if centerHz <= 0 || q <= 0 || sampleHz <= 0 || centerHz >= sampleHz*0.45 {
		b.active = false
		return
	}
	w0 := 2.0 * math.Pi * centerHz / sampleHz
	cosw0 := math.Cos(w0)
	sinw0 := math.Sin(w0)
	alpha := sinw0 / (2.0 * q)

	a0 := 1.0 + alpha
	b.b0 = alpha / a0
	b.b1 = 0
	b.b2 = -alpha / a0
	b.a1 = (-2.0 * cosw0) / a0
	b.a2 = (1.0 - alpha) / a0
	b.active = true
}

// Reset zeroes internal delay lines (safe on pause/session reset).
func (b *Biquad) Reset() {
	b.z1 = 0
	b.z2 = 0
}

// Step processes one sample without heap allocation.
func (b *Biquad) Step(x float64) float64 {
	if !b.active {
		return x
	}
	y := b.b0*x + b.z1
	b.z1 = b.b1*x - b.a1*y + b.z2
	b.z2 = b.b2*x - b.a2*y
	return y
}
