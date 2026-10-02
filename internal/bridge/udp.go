// Package bridge listens for UDP telemetry from a racing simulator
// and emits parsed Telemetry frames at up to 120 Hz.
package bridge

import (
	"encoding/binary"
	"math"
	"net"
	"time"
)

// Telemetry holds one parsed frame of vehicle dynamics data.
// Field names are simulation-agnostic; the caller maps them to DOF targets.
type Telemetry struct {
	Timestamp time.Time
	Speed     float32 // m/s

	VelX, VelY, VelZ     float32 // world velocity vector
	RollX, RollY, RollZ  float32 // roll / right vector
	FwdX, FwdY, FwdZ    float32 // forward / pitch vector

	Throttle float32 // 0..1
	Brake    float32 // 0..1
	Steer    float32 // -1..1
	Clutch   float32 // 0..1
	Gear     int

	RPM   float32
	GLat  float32 // lateral g-force  (positive = right)
	GLong float32 // longitudinal g-force (positive = acceleration)

	// Suspension travel (normalized, not all sources provide this)
	SuspFL, SuspFR, SuspRL, SuspRR float32
}

// Handler is called each time a valid telemetry packet is received.
type Handler func(t Telemetry)

// Listener is a UDP telemetry receiver.
type Listener struct {
	port int
	conn *net.UDPConn
}

// New creates a Listener bound to the given port.
func New(port int) *Listener {
	return &Listener{port: port}
}

// Run blocks and continuously receives UDP packets, calling handler for each
// valid frame. Call Stop to unblock.
func (l *Listener) Run(handler Handler) error {
	addr := &net.UDPAddr{Port: l.port, IP: net.ParseIP("0.0.0.0")}
	conn, err := net.ListenUDP("udp", addr)
	if err != nil {
		return err
	}
	l.conn = conn
	defer conn.Close()

	buf := make([]byte, 1024)
	for {
		n, _, err := conn.ReadFromUDP(buf)
		if err != nil {
			return nil // stopped via Stop()
		}
		if n < 256 {
			continue
		}
		t := parse256(buf[:256])
		handler(t)
	}
}

// Stop closes the underlying UDP connection, unblocking Run.
func (l *Listener) Stop() {
	if l.conn != nil {
		l.conn.Close()
	}
}

func clampF(v, lo, hi float32) float32 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

// parse256 decodes 64 little-endian float32 values from a 256-byte packet.
func parse256(data []byte) Telemetry {
	f := func(i int) float32 {
		bits := binary.LittleEndian.Uint32(data[i*4 : i*4+4])
		return math.Float32frombits(bits)
	}
	return Telemetry{
		Timestamp: time.Now(),
		Speed:     f(7),

		VelX: f(8), VelY: f(9), VelZ: f(10),
		RollX: f(11), RollY: f(12), RollZ: f(13),
		FwdX: f(14), FwdY: f(15), FwdZ: f(16),

		Throttle: clampF(f(29), 0, 1),
		Brake:    clampF(f(31), 0, 1),
		Steer:    clampF(f(30), -1, 1),
		Clutch:   clampF(f(32), 0, 1),
		Gear:     int(f(33)),

		RPM:   f(37),
		GLat:  f(34),
		GLong: f(35),

		SuspFL: f(49), SuspFR: f(50),
		SuspRL: f(51), SuspRR: f(52),
	}
}
