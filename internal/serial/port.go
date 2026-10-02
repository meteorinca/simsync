// Package serial implements the supplied SMC3 Uno protocol.
package serial

import (
	"encoding/binary"
	"fmt"
	goserial "go.bug.st/serial"
	"io"
	"log"
	"math"
	"sync"
	"time"
)

type JointTelemetry struct {
	Joint                 int
	Actual, Target, Error float32
	Duty                  int16
}
type TelemetryHandler func(JointTelemetry)
type Port struct {
	mu        sync.Mutex
	portName  string
	baud      int
	port      goserial.Port
	handler   TelemetryHandler
	stopCh    chan struct{}
	once      sync.Once
	ready     bool
	lastReply time.Time
	joints    [3]JointTelemetry
	seen      [3]time.Time
	gains     [3]float32
	disabled  byte
}

func New(name string, baud int, handler TelemetryHandler) *Port {
	if baud == 0 {
		baud = 500000
	}
	return &Port{portName: name, baud: baud, handler: handler, stopCh: make(chan struct{})}
}
func (p *Port) Start() { go p.loop() }
func (p *Port) Stop() {
	p.once.Do(func() {
		close(p.stopCh)
		p.mu.Lock()
		defer p.mu.Unlock()
		if p.port != nil {
			p.port.Close()
		}
		p.ready = false
	})
}
func frame(id byte, v uint16) []byte { return []byte{'[', id, byte(v >> 8), byte(v), ']'} }
func validPosition(v float32) bool {
	return !math.IsNaN(float64(v)) && !math.IsInf(float64(v), 0) && v >= 0 && v <= 1023
}
func (p *Port) writeLocked(data []byte) error {
	if p.port == nil || !p.ready || time.Since(p.lastReply) > time.Second {
		return fmt.Errorf("SMC3 disconnected or feedback stale")
	}
	for len(data) > 0 {
		n, err := p.port.Write(data)
		if err != nil {
			return err
		}
		if n == 0 {
			return io.ErrShortWrite
		}
		data = data[n:]
	}
	return nil
}
func (p *Port) SendTargets(a, b, c float32) error {
	p.mu.Lock()
	defer p.mu.Unlock()
	var data []byte
	for i, v := range []float32{a, b, c} {
		if !validPosition(v) {
			return fmt.Errorf("target must be 0-1023 ADC counts")
		}
		if p.disabled&(1<<uint(i)) != 0 {
			return fmt.Errorf("SMC3 motor %d disabled by firmware", i+1)
		}
		data = append(data, frame(byte('A'+i), uint16(math.Round(float64(v))))...)
	}
	return p.writeLocked(data)
}
func (p *Port) SetTarget(j int, v float32) error {
	if j < 1 || j > 3 || !validPosition(v) {
		return fmt.Errorf("motor 1-3 and target 0-1023 ADC counts required")
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.disabled&(1<<uint(j-1)) != 0 {
		return fmt.Errorf("motor %d disabled by SMC3; check limits in SMC3Utils", j)
	}
	return p.writeLocked(frame(byte('A'+j-1), uint16(math.Round(float64(v)))))
}
func (p *Port) SetPID(kp, ki, kd float32) error {
	vals := []float32{kp, ki, kd}
	var data []byte
	for g, v := range vals {
		if math.IsNaN(float64(v)) || math.IsInf(float64(v), 0) || v < 0 || v > 327.67 {
			return fmt.Errorf("PID gains must be 0-327.67")
		}
		for m := 0; m < 3; m++ {
			data = append(data, frame(byte('D'+g*3+m), uint16(math.Round(float64(v)*100)))...)
		}
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	if err := p.writeLocked(data); err != nil {
		return err
	}
	copy(p.gains[:], vals)
	return nil
}
func (p *Port) PID() (float32, float32, float32) {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.gains[0], p.gains[1], p.gains[2]
}
func (p *Port) SavePID() error {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.writeLocked([]byte("[sav]"))
}
func (p *Port) Actual(j int) (float32, error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if j < 1 || j > 3 || !p.ready || time.Since(p.seen[j-1]) > 250*time.Millisecond {
		return 0, fmt.Errorf("fresh motor feedback unavailable")
	}
	return p.joints[j-1].Actual, nil
}
func (p *Port) IsConnected() bool {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.ready && p.port != nil && time.Since(p.lastReply) < time.Second
}
func (p *Port) PortName() string { return p.portName }
func (p *Port) Stats() (interface{}, error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if !p.ready || time.Since(p.lastReply) > time.Second {
		return nil, fmt.Errorf("SMC3 disconnected")
	}
	return map[string]interface{}{"controller": "SMC3 Uno", "baud": p.baud, "disabled_mask": p.disabled, "units": "ADC counts"}, nil
}
func (p *Port) retry() bool {
	select {
	case <-p.stopCh:
		return false
	case <-time.After(time.Second):
		return true
	}
}
func (p *Port) loop() {
	for {
		select {
		case <-p.stopCh:
			return
		default:
		}
		name := p.portName
		if name == "" {
			var err error
			name, err = ScanForController()
			if err != nil {
				log.Printf("[serial] %v", err)
				if !p.retry() {
					return
				}
				continue
			}
		}
		sp, err := goserial.Open(name, &goserial.Mode{BaudRate: p.baud})
		if err != nil {
			log.Printf("[serial] %v", err)
			if !p.retry() {
				return
			}
			continue
		}
		sp.SetReadTimeout(100 * time.Millisecond)
		// An Uno may reset on open. Wait for its bootloader.
		select {
		case <-p.stopCh:
			sp.Close()
			return
		case <-time.After(2 * time.Second):
		}
		p.mu.Lock()
		p.port = sp
		p.ready = false
		p.seen = [3]time.Time{}
		p.mu.Unlock()
		log.Printf("[serial] probing SMC3 on %s at %d baud", name, p.baud)
		p.readLoop(sp)
		p.mu.Lock()
		p.ready = false
		p.port = nil
		p.mu.Unlock()
		sp.Close()
		if !p.retry() {
			return
		}
	}
}
func (p *Port) readLoop(sp goserial.Port) {
	var pending []byte
	buf := make([]byte, 256)
	lastPoll := time.Time{}
	started := time.Now()
	for {
		select {
		case <-p.stopCh:
			return
		default:
		}
		if time.Since(lastPoll) >= 20*time.Millisecond {
			p.mu.Lock()
			_, err := sp.Write([]byte("[ver][rdA][rda][rdB][rdb][rdC][rdc][rdD][rdG][rdJ]"))
			p.mu.Unlock()
			if err != nil {
				return
			}
			lastPoll = time.Now()
		}
		n, err := sp.Read(buf)
		if err != nil {
			return
		}
		pending = append(pending, buf[:n]...)
		for len(pending) >= 5 {
			if pending[0] != '[' || pending[4] != ']' {
				pending = pending[1:]
				continue
			}
			p.parseFrame(pending[:5])
			pending = pending[5:]
		}
		p.mu.Lock()
		stale := time.Since(p.lastReply) > time.Second
		p.mu.Unlock()
		if time.Since(started) > 3*time.Second && stale {
			return
		}
	}
}
func (p *Port) parseFrame(f []byte) {
	p.mu.Lock()
	id := f[1]
	if id == 'v' && binary.BigEndian.Uint16(f[2:4]) == 70 {
		p.ready = true
	}
	if id == 'v' || (id >= 'A' && id <= 'C') || (id >= 'a' && id <= 'c') {
		p.lastReply = time.Now()
	}
	var jt JointTelemetry
	emit := false
	if id >= 'A' && id <= 'C' {
		i := int(id - 'A')
		jt = p.joints[i]
		jt.Joint = i + 1
		jt.Actual = float32(f[2]) * 4
		jt.Target = float32(f[3]) * 4
		jt.Error = jt.Target - jt.Actual
		p.joints[i] = jt
		p.seen[i] = time.Now()
		emit = true
	} else if id >= 'a' && id <= 'c' {
		i := int(id - 'a')
		p.joints[i].Duty = int16(f[3])
		p.disabled = f[2] & 7
	} else {
		for i, c := range []byte{'D', 'G', 'J'} {
			if id == c {
				p.gains[i] = float32(binary.BigEndian.Uint16(f[2:4])) / 100
			}
		}
	}
	p.mu.Unlock()
	if emit && p.handler != nil {
		p.handler(jt)
	}
}
