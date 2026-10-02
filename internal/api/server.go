// Package api provides the SimSync HTTP REST API and WebSocket broadcast hub.
package api

import (
	"encoding/json"
	"fmt"
	"io/fs"
	"log"
	"net/http"
	"sync"
	"time"

	"github.com/gorilla/websocket"
)

// WSFrame is the JSON envelope pushed to every connected browser at ~60 Hz.
type WSFrame struct {
	T           float64      `json:"t"`
	Telemetry   TelemetryMsg `json:"telemetry"`
	Joints      [3]JointMsg  `json:"joints"`
	PID         PIDMsg       `json:"pid"`
	Motion      MotionMsg    `json:"motion"`
	Armed       bool         `json:"armed"`
	EStop       bool         `json:"estop"`
	EStopCode   int          `json:"estop_code"`
	EStopReason string       `json:"estop_reason"`
	SerialOK    bool         `json:"serial_ok"`
	UDPHZ       float32      `json:"udp_hz"`
}

type TelemetryMsg struct {
	SpeedMS  float32 `json:"speed_ms"`
	SpeedKMH float32 `json:"speed_kmh"`
	GLat     float32 `json:"g_lat"`
	GLong    float32 `json:"g_long"`
	Throttle float32 `json:"throttle"`
	Brake    float32 `json:"brake"`
	Steer    float32 `json:"steer"`
	Gear     int     `json:"gear"`
	RPM      float32 `json:"rpm"`
}

type JointMsg struct {
	Joint  int     `json:"j"`
	Target float32 `json:"target"`
	Actual float32 `json:"actual"`
	Error  float32 `json:"error"`
	Duty   int16   `json:"duty"`
}

type PIDMsg struct {
	Kp float32 `json:"kp"`
	Ki float32 `json:"ki"`
	Kd float32 `json:"kd"`
}

type MotionMsg struct {
	PitchGain    float32 `json:"pitch_gain"`
	RollGain     float32 `json:"roll_gain"`
	HeaveGain    float32 `json:"heave_gain"`
	PitchNeutral float32 `json:"pitch_neutral"`
	RollNeutral  float32 `json:"roll_neutral"`
	HeaveNeutral float32 `json:"heave_neutral"`
	PitchLimit   float32 `json:"pitch_limit"`
	RollLimit    float32 `json:"roll_limit"`
	HeaveLimit   float32 `json:"heave_limit"`
	FilterHz     float32 `json:"filter_hz"`
}

// Hub manages all WebSocket connections and broadcasts frames.
type Hub struct {
	mu       sync.RWMutex
	clients  map[*websocket.Conn]struct{}
	upgrader websocket.Upgrader
}

// NewHub creates a Hub.
func NewHub() *Hub {
	return &Hub{
		clients: make(map[*websocket.Conn]struct{}),
		upgrader: websocket.Upgrader{
			CheckOrigin: func(r *http.Request) bool { return true },
		},
	}
}

// ServeWS upgrades an HTTP connection to WebSocket.
func (h *Hub) ServeWS(w http.ResponseWriter, r *http.Request) {
	conn, err := h.upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Printf("[ws] upgrade failed: %v", err)
		return
	}
	h.mu.Lock()
	h.clients[conn] = struct{}{}
	h.mu.Unlock()

	for {
		if _, _, err := conn.ReadMessage(); err != nil {
			break
		}
	}
	h.mu.Lock()
	delete(h.clients, conn)
	h.mu.Unlock()
	conn.Close()
}

// Broadcast sends frame to all connected clients.
func (h *Hub) Broadcast(frame WSFrame) {
	data, err := json.Marshal(frame)
	if err != nil {
		return
	}
	h.mu.RLock()
	defer h.mu.RUnlock()
	for conn := range h.clients {
		conn.SetWriteDeadline(time.Now().Add(200 * time.Millisecond))
		if err := conn.WriteMessage(websocket.TextMessage, data); err != nil {
			conn.Close()
		}
	}
}

// Server holds handler state and the embedded web FS.
type Server struct {
	Hub        *Hub
	Handlers   *Handlers
	WebFS      fs.FS
	listenAddr string
}

// New creates a Server. webFS should be the embedded web/ directory.
func New(port int, handlers *Handlers, webFS fs.FS) *Server {
	return &Server{
		Hub:        NewHub(),
		Handlers:   handlers,
		WebFS:      webFS,
		listenAddr: fmt.Sprintf(":%d", port),
	}
}

// Start registers routes and listens. Blocks until the server stops.
func (s *Server) Start() error {
	mux := http.NewServeMux()

	mux.HandleFunc("/ws", s.Hub.ServeWS)

	mux.HandleFunc("/api/status", s.Handlers.Status)
	mux.HandleFunc("/api/session", s.Handlers.Session)
	mux.HandleFunc("/api/session/export", s.Handlers.SessionExport)
	mux.HandleFunc("/api/pid", s.Handlers.PID)
	mux.HandleFunc("/api/pid/save", s.Handlers.PIDSave)
	mux.HandleFunc("/api/target", s.Handlers.Target)
	mux.HandleFunc("/api/arm", s.Handlers.Arm)
	mux.HandleFunc("/api/estop", s.Handlers.EStop)
	mux.HandleFunc("/api/estop/clear", s.Handlers.EStopClear)
	mux.HandleFunc("/api/steptest", s.Handlers.StepTest)
	mux.HandleFunc("/api/steptest/results", s.Handlers.StepResults)
	mux.HandleFunc("/api/sweep", s.Handlers.Sweep)
	mux.HandleFunc("/api/motion/config", s.Handlers.MotionConfig)
	mux.HandleFunc("/api/controller/stats", s.Handlers.ControllerStats)
	mux.HandleFunc("/api/calibrate", s.Handlers.Calibrate)
	mux.HandleFunc("/api/autotune", s.Handlers.Autotune)

	mux.Handle("/", http.FileServer(http.FS(s.WebFS)))

	log.Printf("[api] SimSync dashboard at http://localhost%s", s.listenAddr)
	return http.ListenAndServe(s.listenAddr, mux)
}
