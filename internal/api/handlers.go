package api

import (
	"encoding/json"
	"fmt"
	"github.com/meteorinca/simsync/internal/serial"
	"net/http"
	"strconv"
	"time"
)

// Deps are the live service references handlers need.
type Deps struct {
	GetSessionSamples func(n int) interface{}
	ExportSessionCSV  func(w http.ResponseWriter)
	GetPID            func() (kp, ki, kd float32)
	SetPID            func(joint int, kp, ki, kd float32) error
	GetMotors         func() interface{}
	ConfigureMotor    func(int, serial.MotorSettings) error
	EnableMotor       func(int) error
	JogMotor          func(int, int, int) error
	StopJog           func() error
	Diagnostics       func(bool) error
	CancelTest        func()
	TestRunning       func() bool
	SavePID           func() error
	SetTarget         func(joint int, angle float32) error
	Arm               func(state bool) error
	EStop             func(trip bool) error
	ClearEStop        func() error
	RunStepTest       func(joint int, kp, ki, kd, stepSize float32) error
	GetStepResults    func() interface{}
	GetMotionConfig   func() interface{}
	SetMotionConfig   func(body []byte) error
	ControllerStats   func() (interface{}, error)
	Calibrate         func(joint int) error
	Autotune          func(joint int) error
	SerialOK          func() bool
	UDPHz             func() float32
	Armed             func() bool
	EStopState        func() (bool, int, string)
}

// Handlers holds the HTTP handler functions bound to Deps.
type Handlers struct {
	d *Deps
}

// NewHandlers creates Handlers from the given Deps.
func NewHandlers(d *Deps) *Handlers {
	return &Handlers{d: d}
}

func writeJSON(w http.ResponseWriter, v interface{}) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(v)
}

func writeErr(w http.ResponseWriter, code int, msg string) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(code)
	json.NewEncoder(w).Encode(map[string]string{"error": msg})
}

// Status returns SimSync operational status.
func (h *Handlers) Status(w http.ResponseWriter, r *http.Request) {
	estopped, estopCode, estopReason := h.d.EStopState()
	writeJSON(w, map[string]interface{}{
		"serial_ok":    h.d.SerialOK(),
		"udp_hz":       h.d.UDPHz(),
		"armed":        h.d.Armed(),
		"estop":        estopped,
		"estop_code":   estopCode,
		"estop_reason": estopReason,
		"time":         time.Now().UnixMilli(),
	})
}

// Session returns recent session samples.
func (h *Handlers) Session(w http.ResponseWriter, r *http.Request) {
	n := 500
	if s := r.URL.Query().Get("n"); s != "" {
		if v, err := strconv.Atoi(s); err == nil && v > 0 {
			n = v
		}
	}
	writeJSON(w, h.d.GetSessionSamples(n))
}

// SessionExport streams session data as a CSV download.
func (h *Handlers) SessionExport(w http.ResponseWriter, r *http.Request) {
	ts := time.Now().Format("20060102_150405")
	w.Header().Set("Content-Type", "text/csv")
	w.Header().Set("Content-Disposition", fmt.Sprintf(`attachment; filename="simsync_session_%s.csv"`, ts))
	h.d.ExportSessionCSV(w)
}

// PID handles GET (read current gains) and POST (update gains).
func (h *Handlers) PID(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodGet {
		kp, ki, kd := h.d.GetPID()
		writeJSON(w, map[string]float32{"kp": kp, "ki": ki, "kd": kd})
		return
	}
	kp, ki, kd, err := parsePIDParams(r)
	if err != nil {
		writeErr(w, http.StatusBadRequest, err.Error())
		return
	}
	joint, _ := strconv.Atoi(r.URL.Query().Get("joint"))
	if err := h.d.SetPID(joint, kp, ki, kd); err != nil {
		writeErr(w, http.StatusInternalServerError, err.Error())
		return
	}
	writeJSON(w, map[string]interface{}{"ok": true, "kp": kp, "ki": ki, "kd": kd})
}

// PIDSave persists current PID gains to SMC3 EEPROM.
func (h *Handlers) PIDSave(w http.ResponseWriter, r *http.Request) {
	if err := h.d.SavePID(); err != nil {
		writeErr(w, http.StatusInternalServerError, err.Error())
		return
	}
	writeJSON(w, map[string]bool{"ok": true})
}

// Target commands a joint angle.
func (h *Handlers) Target(w http.ResponseWriter, r *http.Request) {
	q := r.URL.Query()
	joint, _ := strconv.Atoi(q.Get("joint"))
	angle64, err := strconv.ParseFloat(q.Get("angle"), 32)
	if err != nil || joint < 1 || joint > 3 {
		writeErr(w, http.StatusBadRequest, "joint (1-3) and angle required")
		return
	}
	if err := h.d.SetTarget(joint, float32(angle64)); err != nil {
		writeErr(w, http.StatusInternalServerError, err.Error())
		return
	}
	writeJSON(w, map[string]interface{}{"ok": true, "joint": joint, "angle": float32(angle64)})
}

// Arm handles motor arm/disarm.
func (h *Handlers) Arm(w http.ResponseWriter, r *http.Request) {
	state := r.URL.Query().Get("state") == "1"
	if err := h.d.Arm(state); err != nil {
		writeErr(w, http.StatusInternalServerError, err.Error())
		return
	}
	writeJSON(w, map[string]interface{}{"ok": true, "armed": state})
}

// EStop trips or clears the software emergency stop.
func (h *Handlers) EStop(w http.ResponseWriter, r *http.Request) {
	trip := r.URL.Query().Get("state") == "1"
	if err := h.d.EStop(trip); err != nil {
		writeErr(w, http.StatusInternalServerError, err.Error())
		return
	}
	writeJSON(w, map[string]interface{}{"ok": true, "estop": trip})
}

// EStopClear clears the E-stop latch.
func (h *Handlers) EStopClear(w http.ResponseWriter, r *http.Request) {
	if err := h.d.ClearEStop(); err != nil {
		writeErr(w, http.StatusInternalServerError, err.Error())
		return
	}
	writeJSON(w, map[string]bool{"ok": true})
}

// StepTest launches a single step test.
func (h *Handlers) StepTest(w http.ResponseWriter, r *http.Request) {
	q := r.URL.Query()
	joint, _ := strconv.Atoi(q.Get("joint"))
	if joint < 1 || joint > 3 {
		writeErr(w, 400, "motor 1-3 required")
		return
	}
	kp, ki, kd, err := parsePIDParams(r)
	if err != nil {
		writeErr(w, http.StatusBadRequest, err.Error())
		return
	}
	v, err := strconv.ParseFloat(q.Get("step"), 32)
	if err != nil {
		writeErr(w, 400, "valid step required")
		return
	}
	stepSize := float32(v)
	if err := h.d.RunStepTest(joint, kp, ki, kd, stepSize); err != nil {
		writeErr(w, http.StatusConflict, err.Error())
		return
	}
	writeJSON(w, map[string]interface{}{"ok": true, "joint": joint, "kp": kp, "ki": ki, "kd": kd, "step": stepSize})
}

// Sweep preserves the old route without exposing unbounded automated tuning.
func (h *Handlers) Sweep(w http.ResponseWriter, r *http.Request) {
	writeErr(w, http.StatusGone, "use individual bounded step tests")
}

func (h *Handlers) Motors(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodGet {
		writeJSON(w, h.d.GetMotors())
		return
	}
	var body struct {
		Joint    int                  `json:"joint"`
		Settings serial.MotorSettings `json:"settings"`
	}
	if err := json.NewDecoder(http.MaxBytesReader(w, r.Body, 4096)).Decode(&body); err != nil {
		writeErr(w, 400, "invalid settings")
		return
	}
	if err := h.d.ConfigureMotor(body.Joint, body.Settings); err != nil {
		writeErr(w, 409, err.Error())
		return
	}
	writeJSON(w, map[string]bool{"ok": true})
}

func (h *Handlers) MotorEnable(w http.ResponseWriter, r *http.Request) {
	j, _ := strconv.Atoi(r.URL.Query().Get("joint"))
	if err := h.d.EnableMotor(j); err != nil {
		writeErr(w, 409, err.Error())
		return
	}
	writeJSON(w, map[string]bool{"ok": true})
}

func (h *Handlers) Jog(w http.ResponseWriter, r *http.Request) {
	var b struct {
		Joint    int `json:"joint"`
		PWM      int `json:"pwm"`
		Duration int `json:"duration_ms"`
	}
	if err := json.NewDecoder(http.MaxBytesReader(w, r.Body, 1024)).Decode(&b); err != nil {
		writeErr(w, 400, "invalid jog request")
		return
	}
	if err := h.d.JogMotor(b.Joint, b.PWM, b.Duration); err != nil {
		writeErr(w, 409, err.Error())
		return
	}
	writeJSON(w, map[string]bool{"ok": true})
}

func (h *Handlers) JogStop(w http.ResponseWriter, r *http.Request) {
	if err := h.d.StopJog(); err != nil {
		writeErr(w, 409, err.Error())
		return
	}
	writeJSON(w, map[string]bool{"ok": true})
}

func (h *Handlers) Diagnostics(w http.ResponseWriter, r *http.Request) {
	var b struct {
		Enabled bool `json:"enabled"`
	}
	if err := json.NewDecoder(http.MaxBytesReader(w, r.Body, 1024)).Decode(&b); err != nil {
		writeErr(w, 400, "invalid diagnostics request")
		return
	}
	if err := h.d.Diagnostics(b.Enabled); err != nil {
		writeErr(w, 409, err.Error())
		return
	}
	writeJSON(w, map[string]bool{"ok": true})
}

func (h *Handlers) TestStatus(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, map[string]bool{"running": h.d.TestRunning()})
}

func (h *Handlers) TestCancel(w http.ResponseWriter, r *http.Request) {
	h.d.CancelTest()
	writeJSON(w, map[string]bool{"ok": true})
}

// StepResults returns all recorded step test results.
func (h *Handlers) StepResults(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, h.d.GetStepResults())
}

// MotionConfig handles GET/POST of motion engine parameters.
func (h *Handlers) MotionConfig(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodGet {
		writeJSON(w, h.d.GetMotionConfig())
		return
	}
	dec := json.NewDecoder(r.Body)
	var body map[string]interface{}
	if err := dec.Decode(&body); err != nil {
		writeErr(w, http.StatusBadRequest, err.Error())
		return
	}
	data, _ := json.Marshal(body)
	if err := h.d.SetMotionConfig(data); err != nil {
		writeErr(w, http.StatusInternalServerError, err.Error())
		return
	}
	writeJSON(w, map[string]bool{"ok": true})
}

// ControllerStats returns SMC3 controller status.
func (h *Handlers) ControllerStats(w http.ResponseWriter, r *http.Request) {
	stats, err := h.d.ControllerStats()
	if err != nil {
		writeErr(w, http.StatusServiceUnavailable, err.Error())
		return
	}
	writeJSON(w, stats)
}

// Calibrate zeros a joint.
func (h *Handlers) Calibrate(w http.ResponseWriter, r *http.Request) {
	joint, _ := strconv.Atoi(r.URL.Query().Get("joint"))
	if joint < 1 || joint > 3 {
		writeErr(w, http.StatusBadRequest, "joint 1-3 required")
		return
	}
	if err := h.d.Calibrate(joint); err != nil {
		writeErr(w, http.StatusInternalServerError, err.Error())
		return
	}
	writeJSON(w, map[string]bool{"ok": true})
}

// Autotune triggers on-device autotune for a joint.
func (h *Handlers) Autotune(w http.ResponseWriter, r *http.Request) {
	joint, _ := strconv.Atoi(r.URL.Query().Get("joint"))
	if joint < 1 || joint > 3 {
		joint = 1
	}
	if err := h.d.Autotune(joint); err != nil {
		writeErr(w, http.StatusInternalServerError, err.Error())
		return
	}
	writeJSON(w, map[string]bool{"ok": true})
}

// --- helpers ---

func parsePIDParams(r *http.Request) (kp, ki, kd float32, err error) {
	q := r.URL.Query()
	kp64, e1 := strconv.ParseFloat(q.Get("kp"), 32)
	ki64, e2 := strconv.ParseFloat(q.Get("ki"), 32)
	kd64, e3 := strconv.ParseFloat(q.Get("kd"), 32)
	if e1 != nil || e2 != nil || e3 != nil {
		// Try JSON body
		var body struct{ Kp, Ki, Kd float32 }
		if err2 := json.NewDecoder(r.Body).Decode(&body); err2 == nil {
			return body.Kp, body.Ki, body.Kd, nil
		}
		return 0, 0, 0, fmt.Errorf("kp, ki, kd required (query or JSON body)")
	}
	return float32(kp64), float32(ki64), float32(kd64), nil
}
