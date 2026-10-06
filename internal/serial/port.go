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
	mu             sync.Mutex
	portName       string
	baud           int
	port           goserial.Port
	handler        TelemetryHandler
	stopCh         chan struct{}
	once           sync.Once
	ready          bool
	lastReply      time.Time
	joints         [3]JointTelemetry
	seen           [3]time.Time
	gains          [3]float32
	disabled       byte
	settings       [3]MotorSettings
	settingsSeen   [3]time.Time
	settingsMask   [3]byte
	autoConfigured bool
	jogSupported   bool
	jogUntil       time.Time
	jogMotor       int
	diagnostics    bool
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
			if p.jogSupported {
				_ = p.writeLocked([]byte("[stp]"))
			}
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
	if p.jogActiveLocked() {
		return fmt.Errorf("open-loop jog running")
	}
	var data []byte
	for i, v := range []float32{a, b, c} {
		if !p.settings[i].Active {
			continue
		}
		if err := p.checkTargetLocked(i+1, v); err != nil {
			return err
		}
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
	if err := p.checkTargetLocked(j, v); err != nil {
		return err
	}
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
	if p.jogActiveLocked() {
		return fmt.Errorf("open-loop jog running")
	}
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
	if p.jogActiveLocked() {
		return fmt.Errorf("open-loop jog running")
	}
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
			log.Printf("[serial] auto-selected Arduino USB serial candidate on %s; awaiting SMC3 confirmation", name)
		}
		sp, err := goserial.Open(name, &goserial.Mode{BaudRate: p.baud})
		if err != nil {
			log.Printf("[serial] open Arduino candidate %s failed: %v; retrying", name, err)
			if !p.retry() {
				return
			}
			continue
		}
		sp.SetReadTimeout(5 * time.Millisecond)
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
		p.settingsMask = [3]byte{}
		p.autoConfigured = false
		p.jogSupported = false
		p.diagnostics = false
		p.jogUntil = time.Time{}
		for i := range p.settings {
			p.settings[i].Configured = false
			p.settings[i].Active = false
		}
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
	settingsQueries := []string{"[ver]", "[cap]", "[rdD]", "[rdE]", "[rdF]", "[rdG]", "[rdH]", "[rdI]", "[rdJ]", "[rdK]", "[rdL]", "[rdP]", "[rdQ]", "[rdR]", "[rdS]", "[rdT]", "[rdU]", "[rdV]", "[rdW]", "[rdX]"}
	settingsCursor := 0
	started := time.Now()
	for {
		select {
		case <-p.stopCh:
			return
		default:
		}
		if time.Since(lastPoll) >= 20*time.Millisecond {
			p.mu.Lock()
			query := "[rdA][rda][rdB][rdb][rdC][rdc]"
			// Keep requests below the Uno's 64-byte serial receive buffer.
			for n := 0; n < 3; n++ {
				query += settingsQueries[settingsCursor]
				settingsCursor = (settingsCursor + 1) % len(settingsQueries)
			}
			_, err := sp.Write([]byte(query))
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
		p.autoConfigureBench()
		p.mu.Lock()
		stale := time.Since(p.lastReply) > time.Second
		p.mu.Unlock()
		if time.Since(started) > 3*time.Second && stale {
			return
		}
	}
}

// autoConfigureBench makes the first controller channel immediately usable for
// an unmounted bench rig. The startup profile replaces legacy EEPROM settings
// in RAM once per connection, with Ki zero and the requested 50-650 bounds.
func (p *Port) autoConfigureBench() {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.autoConfigured || !p.ready || p.settingsMask[0] != 63 || time.Since(p.settingsSeen[0]) > 2*time.Second || time.Since(p.seen[0]) > 250*time.Millisecond {
		return
	}
	s := BenchDefaults()
	if err := s.Validate(); err != nil {
		return
	}
	data := pidFrames(1, s.Kp, s.Ki, s.Kd)
	data = append(data, frame('P', uint16(s.PWMMin<<8|s.PWMMax))...)
	data = append(data, frame('S', uint16(s.Cutoff<<8|s.Clip))...)
	data = append(data, frame('V', uint16(s.Deadzone<<8|s.PWMRev))...)
	if err := p.writeLocked(data); err != nil {
		return
	}
	p.settings[0] = s
	p.settings[0].Configured = true
	p.autoConfigured = true
}
func (p *Port) parseFrame(f []byte) {
	p.mu.Lock()
	id := f[1]
	if id == 'q' {
		p.jogSupported = binary.BigEndian.Uint16(f[2:4])&1 != 0
	}
	if id >= 'D' && id <= 'L' {
		i, g := int(id-'D')%3, int(id-'D')/3
		v := float32(binary.BigEndian.Uint16(f[2:4])) / 100
		switch g {
		case 0:
			p.settings[i].Kp = v
		case 1:
			p.settings[i].Ki = v
		case 2:
			p.settings[i].Kd = v
		}
		p.settingsMask[i] |= 1 << uint(g)
		p.settingsSeen[i] = time.Now()
	}
	if id >= 'P' && id <= 'X' {
		i, g := int(id-'P')%3, int(id-'P')/3
		s := &p.settings[i]
		switch g {
		case 0:
			s.PWMMin = int(f[2])
			s.PWMMax = int(f[3])
		case 1:
			s.Cutoff = int(f[2])
			s.Clip = int(f[3])
		case 2:
			s.Deadzone = int(f[2])
			s.PWMRev = int(f[3])
		}
		p.settingsMask[i] |= 1 << uint(g+3)
		p.settingsSeen[i] = time.Now()
	}
	if id == 'v' && binary.BigEndian.Uint16(f[2:4]) == 70 {
		if !p.ready {
			log.Printf("[serial] confirmed Arduino SMC3 firmware 0.70 at %d baud", p.baud)
		}
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
