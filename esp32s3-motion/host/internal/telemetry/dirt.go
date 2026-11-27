package telemetry

import (
	"encoding/binary"
	"math"
	"net"
	"time"
)

// Frame holds one zero-allocation parsed Dirt Rally 2.0 (extradata=3, 264B or 256B) telemetry packet.
type Frame struct {
	Timestamp time.Time
	Speed     float64 // m/s
	RPM       float64
	Gear      int
	Throttle  float64
	Brake     float64
	Steer     float64

	// Vehicle G-forces & attitude
	SurgeG float64 // Longitudinal G (positive = accel)
	SwayG  float64 // Lateral G (positive = right)
	HeaveG float64 // Vertical G derived from world VelY delta
	Pitch  float64 // Radians
	Roll   float64 // Radians

	// Suspension travel & velocities (4 corners: RL, RR, FL, FR in Dirt Rally 2.0)
	SuspRL, SuspRR, SuspFL, SuspFR       float64
	SuspVelRL, SuspVelRR, SuspVelFL, SuspVelFR float64
	YawSlip                              float64 // Estimated lateral slip / traction loss cue
}

// Listener listens for UDP telemetry with zero per-packet heap allocations.
type Listener struct {
	port     int
	conn     *net.UDPConn
	prevVelY float64
	prevTime time.Time
	hasPrev  bool
}

func NewListener(port int) *Listener {
	return &Listener{port: port}
}

func (l *Listener) Run(onFrame func(Frame)) error {
	addr := &net.UDPAddr{Port: l.port, IP: net.IPv4zero}
	conn, err := net.ListenUDP("udp", addr)
	if err != nil {
		return err
	}
	l.conn = conn
	defer conn.Close()

	// Pre-allocated receive buffer (no per-packet heap allocation)
	var buf [512]byte
	for {
		n, _, err := conn.ReadFromUDP(buf[:])
		if err != nil {
			return nil
		}
		if n < 256 {
			continue
		}
		f := l.parseCodemasters(buf[:n])
		onFrame(f)
	}
}

func (l *Listener) Stop() {
	if l.conn != nil {
		_ = l.conn.Close()
	}
}

func readF32(b []byte, idx int) float64 {
	off := idx * 4
	if off+4 > len(b) {
		return 0
	}
	v := float64(math.Float32frombits(binary.LittleEndian.Uint32(b[off : off+4])))
	if math.IsNaN(v) || math.IsInf(v, 0) {
		return 0
	}
	return v
}

// parseCodemasters decodes Dirt Rally 2.0 / Codemasters extradata="3" (264 bytes = 66 float32)
// and standard 256-byte packets.
func (l *Listener) parseCodemasters(b []byte) Frame {
	now := time.Now()
	velX := readF32(b, 8)
	velY := readF32(b, 9)
	velZ := readF32(b, 10)

	rollX := readF32(b, 11)
	rollY := readF32(b, 12)
	rollZ := readF32(b, 13)
	fwdX := readF32(b, 14)
	fwdY := readF32(b, 15)
	fwdZ := readF32(b, 16)

	// Extract chassis pitch & roll angles from orientation unit vectors
	pitch := math.Asin(clamp(fwdY, -1, 1))
	roll := math.Asin(clamp(rollY, -1, 1))

	// Compute vertical acceleration (HeaveG) from VelY derivative
	heaveG := 0.0
	if l.hasPrev {
		dt := now.Sub(l.prevTime).Seconds()
		if dt > 0.001 && dt < 0.2 {
			heaveG = ((velY - l.prevVelY) / dt) / 9.81
		}
	}
	l.prevVelY = velY
	l.prevTime = now
	l.hasPrev = true

	// Estimate lateral velocity (traction loss / drift angle) by projecting world velocity onto right vector
	latVel := velX*rollX + velY*rollY + velZ*rollZ
	fwdVel := velX*fwdX + velY*fwdY + velZ*fwdZ
	yawSlip := 0.0
	if math.Abs(fwdVel) > 2.0 {
		yawSlip = math.Atan2(latVel, math.Abs(fwdVel))
	}

	// Codemasters / Dirt Rally 2.0 extradata=3 field indices:
	//  [17..20] Suspension position (RL, RR, FL, FR)
	//  [21..24] Suspension velocity (RL, RR, FL, FR)
	//  [29] Throttle, [30] Steer, [31] Brake, [33] Gear, [34] GLat, [35] GLong, [37] RPM
	return Frame{
		Timestamp: now,
		Speed:     readF32(b, 7),
		Throttle:  clamp(readF32(b, 29), 0, 1),
		Steer:     clamp(readF32(b, 30), -1, 1),
		Brake:     clamp(readF32(b, 31), 0, 1),
		Gear:      int(readF32(b, 33)),
		SwayG:     readF32(b, 34),
		SurgeG:    readF32(b, 35),
		HeaveG:    clamp(heaveG, -5, 5),
		RPM:       readF32(b, 37) * 10.0,
		Pitch:     pitch,
		Roll:      roll,
		SuspRL:    readF32(b, 17),
		SuspRR:    readF32(b, 18),
		SuspFL:    readF32(b, 19),
		SuspFR:    readF32(b, 20),
		SuspVelRL: readF32(b, 21),
		SuspVelRR: readF32(b, 22),
		SuspVelFL: readF32(b, 23),
		SuspVelFR: readF32(b, 24),
		YawSlip:   clamp(yawSlip, -1.2, 1.2),
	}
}

func clamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
