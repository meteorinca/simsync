package serial

import (
	"fmt"
	"math"
	"time"
)

// Guard a stationary target against sustained motion away from the closest
// measured position. It does not diagnose polarity or replace travel cutoffs.
type trackingGuard struct {
	target, closest float32
	valid           bool
	awaySince       time.Time
}

func (g *trackingGuard) update(target, actual float32, now time.Time) bool {
	err := float32(math.Abs(float64(target - actual)))
	if !g.valid || g.target != target {
		*g = trackingGuard{target: target, closest: err, valid: true}
		return false
	}
	if err < g.closest {
		g.closest = err
	}
	if err <= g.closest+16 {
		g.awaySince = time.Time{}
		return false
	}
	if g.awaySince.IsZero() {
		g.awaySince = now
	}
	return now.Sub(g.awaySince) >= 100*time.Millisecond
}

func (p *Port) guardTrackingLocked(i int, jt JointTelemetry) {
	if !p.jogSupported || p.guardMask&(1<<uint(i)) == 0 || p.disabled&(1<<uint(i)) != 0 || p.diagnostics || p.jogActiveLocked() {
		return
	}
	if p.tracking[i].update(jt.Target, jt.Actual, time.Now()) {
		p.motionFault = fmt.Sprintf("M%d moved away from its target; check motor/feedback direction and gains before enabling again", i+1)
		if err := p.writeLocked([]byte("[stp]")); err != nil {
			p.motionFault += "; stop command failed: " + err.Error()
		}
		p.guardMask = 0
		p.soloMotor = 0
		p.disabled = 7
	}
}

func (p *Port) MotionFault() string {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.motionFault
}

func (p *Port) ClearMotionFault() {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.motionFault = ""
}
