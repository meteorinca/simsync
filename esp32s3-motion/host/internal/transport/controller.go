package transport

import (
	"encoding/binary"
	"fmt"
	"log"
	"math"
	"strings"
	"sync"
	"time"

	goserial "go.bug.st/serial"
	"go.bug.st/serial/enumerator"
)

const (
	TurboSync0 = 0xAA
	TurboSync1 = 0x55

	CmdTargets12 = 0x01
	CmdSetGains  = 0x02
	CmdSetLimits = 0x03
	CmdControl   = 0x04
	CmdGetConfig = 0x05

	OpStopAll      = 0x00
	OpEnableMask   = 0x01
	OpHoldMask     = 0x02
	OpClearEStop   = 0x03
	OpSaveNVS      = 0x04
	OpSetStreamHz  = 0x05
	OpSetPWMkHz    = 0x06
	OpSetActiveDOF = 0x07
	OpResetStats   = 0x08

	PktTelemetry = 0x81
	PktMotorCfg  = 0x82
	PktGlobalCfg = 0x83
	PktAck       = 0x84
)

// MotorState holds full 12-bit resolution live telemetry & noise metrics for one motor.
type MotorState struct {
	Actual12     uint16  `json:"actual_12"`
	Target12     uint16  `json:"target_12"`
	Duty         int16   `json:"duty"`           // Signed -1000..+1000
	NoiseP2P12   uint16  `json:"noise_p2p_12"`   // Peak-to-peak ADC spread in 12-bit counts
	NoiseRMS12   float64 `json:"noise_rms_12"`   // Smoothed RMS noise in 12-bit counts
	ADCSamples   uint16  `json:"adc_samples"`    // GDMA oversamples averaged per PID tick
	Disabled     bool    `json:"disabled"`
}

// Telemetry holds the full ESP32-S3 live snapshot.
type Telemetry struct {
	Connected    bool          `json:"connected"`
	Port         string        `json:"port"`
	Protocol     string        `json:"protocol"` // "turbo-v2" or "smc3-10bit"
	Motors       [3]MotorState `json:"motors"`
	DisabledMask uint8         `json:"disabled_mask"`
	FaultFlags   uint8         `json:"fault_flags"`
	ActiveDOF    uint8         `json:"active_dof"`
	PIDDivider   uint8         `json:"pid_divider"`
	LoopExecUs   uint16        `json:"loop_exec_us"`
	LoopMaxUs    uint16        `json:"loop_max_us"`
	LoopJitterUs uint16        `json:"loop_jitter_us"`
	LoopCount    uint32        `json:"loop_count"`
	UpdatedAt    time.Time     `json:"updated_at"`
}

// MotorGains holds per-motor PID + feedforward + biquad filter settings.
type MotorGains struct {
	Motor int     `json:"motor"` // 0..2
	Kp    float32 `json:"kp"`
	Ki    float32 `json:"ki"`
	Kd    float32 `json:"kd"`
	Kff   float32 `json:"kff"`
	LPFHz float32 `json:"lpf_hz"`
	HPFHz float32 `json:"hpf_hz"`
	Ks    uint16  `json:"ks"`
}

// MotorLimits holds per-motor travel, PWM, and polarity settings (12-bit counts).
type MotorLimits struct {
	Motor       int    `json:"motor"` // 0..2
	CutoffMin12 uint16 `json:"cutoff_min_12"`
	CutoffMax12 uint16 `json:"cutoff_max_12"`
	ClipMin12   uint16 `json:"clip_min_12"`
	ClipMax12   uint16 `json:"clip_max_12"`
	Deadzone12  uint16 `json:"deadzone_12"`
	PWMMin      uint16 `json:"pwm_min"`
	PWMMax      uint16 `json:"pwm_max"`
	PWMRev      uint16 `json:"pwm_rev"`
	InvertPot   bool   `json:"invert_pot"`
	InvertDir   bool   `json:"invert_dir"`
}

// CRC8 computes SMBus CRC-8 (poly 0x07), matching ESP32-S3 firmware.
func CRC8(data []byte) byte {
	var crc byte
	for _, b := range data {
		crc ^= b
		for i := 0; i < 8; i++ {
			if crc&0x80 != 0 {
				crc = (crc << 1) ^ 0x07
			} else {
				crc <<= 1
			}
		}
	}
	return crc
}

// BuildTurboPacket encodes a framed Turbo v2 binary packet.
func BuildTurboPacket(pktType byte, payload []byte) []byte {
	pkt := make([]byte, 5+len(payload))
	pkt[0] = TurboSync0
	pkt[1] = TurboSync1
	pkt[2] = pktType
	pkt[3] = byte(len(payload))
	copy(pkt[4:], payload)
	pkt[4+len(payload)] = CRC8(pkt[2 : 4+len(payload)])
	return pkt
}

// BuildSMC3Target encodes a legacy 5-byte SMC3 10-bit position frame.
func BuildSMC3Target(motorIdx int, target12 uint16) []byte {
	t10 := target12 >> 2
	if t10 > 1023 {
		t10 = 1023
	}
	return []byte{'[', byte('A' + motorIdx), byte(t10 >> 8), byte(t10 & 0xFF), ']'}
}

// AutoDetectPort finds an ESP32-S3 native USB Serial/JTAG port (VID 303A, PID 1001) first,
// falling back to a single USB serial device if unambiguous.
func AutoDetectPort() (string, error) {
	ports, err := enumerator.GetDetailedPortsList()
	if err != nil {
		return "", err
	}
	var usbCandidates []string
	for _, p := range ports {
		if !p.IsUSB {
			continue
		}
		vid := strings.ToUpper(p.VID)
		pid := strings.ToUpper(p.PID)
		// Espressif ESP32-S3 Native USB Serial/JTAG or TinyUSB CDC
		if vid == "303A" && (pid == "1001" || pid == "4001" || pid == "0002") {
			return p.Name, nil
		}
		usbCandidates = append(usbCandidates, p.Name)
	}
	if len(usbCandidates) == 1 {
		return usbCandidates[0], nil
	}
	if len(usbCandidates) == 0 {
		return "", fmt.Errorf("no USB serial controller detected")
	}
	return "", fmt.Errorf("multiple USB serial ports found (%s); specify --serial", strings.Join(usbCandidates, ", "))
}

// Controller manages the native USB connection to the ESP32-S3 (or SMC3 fallback).
type Controller struct {
	mu       sync.Mutex
	portName string
	baud     int
	smc3Mode bool
	sp       goserial.Port
	stopCh   chan struct{}
	once     sync.Once

	telem    Telemetry
	onTelem  func(Telemetry)
	txBuf    [32]byte // pre-allocated buffer for zero-alloc high-rate target writes
}

func NewController(portName string, baud int, smc3Mode bool, onTelem func(Telemetry)) *Controller {
	if baud <= 0 {
		baud = 921600
	}
	proto := "turbo-v2"
	if smc3Mode {
		proto = "smc3-10bit"
	}
	return &Controller{
		portName: portName,
		baud:     baud,
		smc3Mode: smc3Mode,
		stopCh:   make(chan struct{}),
		onTelem:  onTelem,
		telem: Telemetry{
			Protocol:     proto,
			DisabledMask: 0x07,
			ActiveDOF:    2,
		},
	}
}

func (c *Controller) Start() {
	go c.runLoop()
}

func (c *Controller) Stop() {
	c.once.Do(func() {
		close(c.stopCh)
		c.mu.Lock()
		defer c.mu.Unlock()
		if c.sp != nil {
			_ = c.sendStopLocked()
			_ = c.sp.Close()
			c.sp = nil
		}
		c.telem.Connected = false
	})
}

func (c *Controller) Snapshot() Telemetry {
	c.mu.Lock()
	defer c.mu.Unlock()
	t := c.telem
	if t.Connected && time.Since(t.UpdatedAt) > 2*time.Second && !c.smc3Mode {
		// Still keep port state visible
	}
	return t
}

func (c *Controller) writeLocked(data []byte) error {
	if c.sp == nil {
		return fmt.Errorf("controller not connected")
	}
	for len(data) > 0 {
		n, err := c.sp.Write(data)
		if err != nil {
			return err
		}
		data = data[n:]
	}
	return nil
}

// SendTargets12 streams all 3 motor 12-bit positions in a single 12-byte zero-alloc Turbo v2 frame
// (or 3x 5-byte SMC3 frames in fallback mode).
func (c *Controller) SendTargets12(t [3]uint16, mask uint8) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.sp == nil {
		return nil
	}
	if c.smc3Mode {
		n := 0
		for i := 0; i < 3; i++ {
			if mask&(1<<uint(i)) == 0 {
				continue
			}
			t10 := t[i] >> 2
			if t10 > 1023 {
				t10 = 1023
			}
			c.txBuf[n] = '['
			c.txBuf[n+1] = byte('A' + i)
			c.txBuf[n+2] = byte(t10 >> 8)
			c.txBuf[n+3] = byte(t10 & 0xFF)
			c.txBuf[n+4] = ']'
			n += 5
		}
		if n == 0 {
			return nil
		}
		return c.writeLocked(c.txBuf[:n])
	}

	// Zero-allocation Turbo v2 packet (12 bytes total)
	c.txBuf[0] = TurboSync0
	c.txBuf[1] = TurboSync1
	c.txBuf[2] = CmdTargets12
	c.txBuf[3] = 7
	binary.LittleEndian.PutUint16(c.txBuf[4:6], t[0])
	binary.LittleEndian.PutUint16(c.txBuf[6:8], t[1])
	binary.LittleEndian.PutUint16(c.txBuf[8:10], t[2])
	c.txBuf[10] = mask
	c.txBuf[11] = CRC8(c.txBuf[2:11])
	return c.writeLocked(c.txBuf[:12])
}

func (c *Controller) SendControl(op byte, arg1, arg2 uint16) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.smc3Mode {
		switch op {
		case OpStopAll:
			return c.writeLocked([]byte("[stp]"))
		case OpEnableMask:
			return c.writeLocked([]byte("[ena]"))
		case OpHoldMask:
			return c.writeLocked([]byte{'[', 'h', 0, byte(arg1), ']'})
		case OpSaveNVS:
			return c.writeLocked([]byte("[sav]"))
		}
		return nil
	}
	var pl [5]byte
	pl[0] = op
	binary.LittleEndian.PutUint16(pl[1:3], arg1)
	binary.LittleEndian.PutUint16(pl[3:5], arg2)
	return c.writeLocked(BuildTurboPacket(CmdControl, pl[:]))
}

func (c *Controller) sendStopLocked() error {
	if c.smc3Mode {
		return c.writeLocked([]byte("[stp]"))
	}
	var pl [5]byte
	pl[0] = OpStopAll
	return c.writeLocked(BuildTurboPacket(CmdControl, pl[:]))
}

func (c *Controller) SendGains(g MotorGains) error {
	if g.Motor < 0 || g.Motor > 2 {
		return fmt.Errorf("invalid motor %d", g.Motor)
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.smc3Mode {
		var out []byte
		m := byte(g.Motor)
		out = append(out, '[', 'D'+m, byte(uint16(g.Kp*100)>>8), byte(uint16(g.Kp*100)), ']')
		out = append(out, '[', 'G'+m, byte(uint16(g.Ki*100)>>8), byte(uint16(g.Ki*100)), ']')
		out = append(out, '[', 'J'+m, byte(uint16(g.Kd*100)>>8), byte(uint16(g.Kd*100)), ']')
		out = append(out, '[', 'M'+m, byte(g.Ks>>8), byte(g.Ks), ']')
		return c.writeLocked(out)
	}
	var pl [27]byte
	pl[0] = byte(g.Motor)
	binary.LittleEndian.PutUint32(pl[1:5], math.Float32bits(g.Kp))
	binary.LittleEndian.PutUint32(pl[5:9], math.Float32bits(g.Ki))
	binary.LittleEndian.PutUint32(pl[9:13], math.Float32bits(g.Kd))
	binary.LittleEndian.PutUint32(pl[13:17], math.Float32bits(g.Kff))
	binary.LittleEndian.PutUint32(pl[17:21], math.Float32bits(g.LPFHz))
	binary.LittleEndian.PutUint32(pl[21:25], math.Float32bits(g.HPFHz))
	binary.LittleEndian.PutUint16(pl[25:27], g.Ks)
	return c.writeLocked(BuildTurboPacket(CmdSetGains, pl[:]))
}

func (c *Controller) SendLimits(l MotorLimits) error {
	if l.Motor < 0 || l.Motor > 2 {
		return fmt.Errorf("invalid motor %d", l.Motor)
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	var pl [19]byte
	pl[0] = byte(l.Motor)
	binary.LittleEndian.PutUint16(pl[1:3], l.CutoffMin12)
	binary.LittleEndian.PutUint16(pl[3:5], l.CutoffMax12)
	binary.LittleEndian.PutUint16(pl[5:7], l.ClipMin12)
	binary.LittleEndian.PutUint16(pl[7:9], l.ClipMax12)
	binary.LittleEndian.PutUint16(pl[9:11], l.Deadzone12)
	binary.LittleEndian.PutUint16(pl[11:13], l.PWMMin)
	binary.LittleEndian.PutUint16(pl[13:15], l.PWMMax)
	binary.LittleEndian.PutUint16(pl[15:17], l.PWMRev)
	if l.InvertPot {
		pl[17] = 1
	}
	if l.InvertDir {
		pl[18] = 1
	}
	return c.writeLocked(BuildTurboPacket(CmdSetLimits, pl[:]))
}

func (c *Controller) runLoop() {
	for {
		select {
		case <-c.stopCh:
			return
		default:
		}

		portName := c.portName
		if portName == "" {
			detected, err := AutoDetectPort()
			if err != nil {
				c.sleepRetry()
				continue
			}
			portName = detected
		}

		sp, err := goserial.Open(portName, &goserial.Mode{BaudRate: c.baud})
		if err != nil {
			log.Printf("[serial] waiting for %s: %v", portName, err)
			c.sleepRetry()
			continue
		}
		_ = sp.SetReadTimeout(100 * time.Millisecond)

		c.mu.Lock()
		c.sp = sp
		c.telem.Connected = true
		c.telem.Port = portName
		c.mu.Unlock()

		log.Printf("[serial] connected to %s (%s)", portName, c.telem.Protocol)

		// Enable 100 Hz Turbo v2 binary telemetry stream from ESP32-S3
		if !c.smc3Mode {
			_ = c.SendControl(OpSetStreamHz, 100, 0)
		}

		c.readSession(sp)

		c.mu.Lock()
		if c.sp == sp {
			_ = sp.Close()
			c.sp = nil
		}
		c.telem.Connected = false
		c.mu.Unlock()
		log.Printf("[serial] disconnected from %s; retrying...", portName)
		c.sleepRetry()
	}
}

func (c *Controller) sleepRetry() {
	select {
	case <-c.stopCh:
	case <-time.After(1 * time.Second):
	}
}

// ParseTurboTelemetry decodes a 50-byte TURBO_PKT_TELEMETRY payload.
func ParseTurboTelemetry(pl []byte, dst *Telemetry) bool {
	if len(pl) != 50 {
		return false
	}
	off := 0
	for i := 0; i < 3; i++ {
		dst.Motors[i].Actual12 = binary.LittleEndian.Uint16(pl[off : off+2])
		dst.Motors[i].Target12 = binary.LittleEndian.Uint16(pl[off+2 : off+4])
		dst.Motors[i].Duty = int16(binary.LittleEndian.Uint16(pl[off+4 : off+6]))
		dst.Motors[i].NoiseP2P12 = binary.LittleEndian.Uint16(pl[off+6 : off+8])
		dst.Motors[i].NoiseRMS12 = float64(binary.LittleEndian.Uint16(pl[off+8:off+10])) * 0.01
		dst.Motors[i].ADCSamples = binary.LittleEndian.Uint16(pl[off+10 : off+12])
		off += 12
	}
	dst.DisabledMask = pl[off]
	dst.FaultFlags = pl[off+1]
	dst.ActiveDOF = pl[off+2]
	dst.PIDDivider = pl[off+3]
	dst.LoopExecUs = binary.LittleEndian.Uint16(pl[off+4 : off+6])
	dst.LoopMaxUs = binary.LittleEndian.Uint16(pl[off+6 : off+8])
	dst.LoopJitterUs = binary.LittleEndian.Uint16(pl[off+8 : off+10])
	dst.LoopCount = binary.LittleEndian.Uint32(pl[off+10 : off+14])
	for i := 0; i < 3; i++ {
		dst.Motors[i].Disabled = (dst.DisabledMask & (1 << uint(i))) != 0
	}
	dst.UpdatedAt = time.Now()
	return true
}

func (c *Controller) readSession(sp goserial.Port) {
	var buf [256]byte
	state := 0
	var pktType, pktLen byte
	var pl [64]byte
	plIdx := 0

	for {
		select {
		case <-c.stopCh:
			return
		default:
		}
		n, err := sp.Read(buf[:])
		if err != nil {
			return
		}
		for i := 0; i < n; i++ {
			b := buf[i]
			switch state {
			case 0:
				if b == TurboSync0 {
					state = 1
				}
			case 1:
				if b == TurboSync1 {
					state = 2
				} else {
					state = 0
				}
			case 2:
				pktType = b
				state = 3
			case 3:
				pktLen = b
				if pktLen > 64 {
					state = 0
				} else if pktLen == 0 {
					state = 5
				} else {
					plIdx = 0
					state = 4
				}
			case 4:
				pl[plIdx] = b
				plIdx++
				if plIdx >= int(pktLen) {
					state = 5
				}
			case 5:
				var chk [66]byte
				chk[0] = pktType
				chk[1] = pktLen
				copy(chk[2:], pl[:pktLen])
				if b == CRC8(chk[:2+pktLen]) && pktType == PktTelemetry {
					c.mu.Lock()
					ok := ParseTurboTelemetry(pl[:pktLen], &c.telem)
					snap := c.telem
					c.mu.Unlock()
					if ok && c.onTelem != nil {
						c.onTelem(snap)
					}
				}
				state = 0
			}
		}
	}
}
