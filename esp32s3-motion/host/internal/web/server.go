package web

import (
	_ "embed"
	"encoding/json"
	"net/http"
	"sync"

	"github.com/gorilla/websocket"
	"simsync-s3/internal/transport"
	"simsync-s3/internal/washout"
)

//go:embed index.html
var indexHTML []byte

// StatePacket is pushed to connected dashboard browsers over WebSocket at ~30 Hz.
type StatePacket struct {
	Telemetry   transport.Telemetry `json:"telemetry"`
	Running     bool                `json:"running"`
	BenchMode   bool                `json:"bench_mode"`
	UDPRateHz   float64             `json:"udp_rate_hz"`
	UDPLive     bool                `json:"udp_live"`
	PitchCmd    float64             `json:"pitch_cmd"`
	RollCmd     float64             `json:"roll_cmd"`
	HeaveCmd    float64             `json:"heave_cmd"`
	Target12    [3]uint16           `json:"target_12"`
}

// Actions bridges HTTP/WS UI calls into the main motion orchestrator.
type Actions interface {
	GetWashout() washout.Config
	SetWashout(cfg washout.Config) error
	StopAll() error
	HoldEnable() error
	CenterAll() error
	SetBenchTarget(motor int, target12 uint16) error
	RunSineTest(motor int, amp12 uint16, freqHz float64, cycles int) error
	SetGains(g transport.MotorGains) error
	SetLimits(l transport.MotorLimits) error
	SetPWMkHz(pwm12, pwm3 uint16) error
	SaveNVS() error
	ClearEStop() error
	ResetStats() error
}

type Server struct {
	actions  Actions
	upgrader websocket.Upgrader

	mu      sync.Mutex
	clients map[*websocket.Conn]struct{}
}

func NewServer(actions Actions) *Server {
	return &Server{
		actions: actions,
		upgrader: websocket.Upgrader{
			CheckOrigin: func(r *http.Request) bool { return true },
		},
		clients: make(map[*websocket.Conn]struct{}),
	}
}

func (s *Server) Broadcast(pkt StatePacket) {
	data, err := json.Marshal(pkt)
	if err != nil {
		return
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	for conn := range s.clients {
		if err := conn.WriteMessage(websocket.TextMessage, data); err != nil {
			_ = conn.Close()
			delete(s.clients, conn)
		}
	}
}

func (s *Server) Handler() http.Handler {
	mux := http.NewServeMux()
	mux.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		_, _ = w.Write(indexHTML)
	})

	mux.HandleFunc("/ws", func(w http.ResponseWriter, r *http.Request) {
		conn, err := s.upgrader.Upgrade(w, r, nil)
		if err != nil {
			return
		}
		s.mu.Lock()
		s.clients[conn] = struct{}{}
		s.mu.Unlock()
		go func() {
			defer func() {
				s.mu.Lock()
				delete(s.clients, conn)
				s.mu.Unlock()
				_ = conn.Close()
			}()
			for {
				if _, _, err := conn.ReadMessage(); err != nil {
					return
				}
			}
		}()
	})

	mux.HandleFunc("/api/washout", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "application/json")
		if r.Method == http.MethodGet {
			_ = json.NewEncoder(w).Encode(s.actions.GetWashout())
			return
		}
		if r.Method == http.MethodPost {
			var cfg washout.Config
			if err := json.NewDecoder(r.Body).Decode(&cfg); err != nil {
				http.Error(w, err.Error(), http.StatusBadRequest)
				return
			}
			if err := s.actions.SetWashout(cfg); err != nil {
				http.Error(w, err.Error(), http.StatusBadRequest)
				return
			}
			_ = json.NewEncoder(w).Encode(map[string]bool{"ok": true})
		}
	})

	mux.HandleFunc("/api/stop", postAction(s.actions.StopAll))
	mux.HandleFunc("/api/enable", postAction(s.actions.HoldEnable))
	mux.HandleFunc("/api/center", postAction(s.actions.CenterAll))
	mux.HandleFunc("/api/save-nvs", postAction(s.actions.SaveNVS))
	mux.HandleFunc("/api/clear-estop", postAction(s.actions.ClearEStop))
	mux.HandleFunc("/api/reset-stats", postAction(s.actions.ResetStats))

	mux.HandleFunc("/api/gains", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "POST required", http.StatusMethodNotAllowed)
			return
		}
		var g transport.MotorGains
		if err := json.NewDecoder(r.Body).Decode(&g); err != nil {
			http.Error(w, err.Error(), http.StatusBadRequest)
			return
		}
		if err := s.actions.SetGains(g); err != nil {
			http.Error(w, err.Error(), http.StatusBadRequest)
			return
		}
		w.Header().Set("Content-Type", "application/json")
		_, _ = w.Write([]byte(`{"ok":true}`))
	})

	mux.HandleFunc("/api/target", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "POST required", http.StatusMethodNotAllowed)
			return
		}
		var req struct {
			Motor    int    `json:"motor"`
			Target12 uint16 `json:"target_12"`
		}
		if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
			http.Error(w, err.Error(), http.StatusBadRequest)
			return
		}
		if err := s.actions.SetBenchTarget(req.Motor, req.Target12); err != nil {
			http.Error(w, err.Error(), http.StatusBadRequest)
			return
		}
		w.Header().Set("Content-Type", "application/json")
		_, _ = w.Write([]byte(`{"ok":true}`))
	})

	mux.HandleFunc("/api/sine", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "POST required", http.StatusMethodNotAllowed)
			return
		}
		var req struct {
			Motor  int     `json:"motor"`
			Amp12  uint16  `json:"amp_12"`
			FreqHz float64 `json:"freq_hz"`
			Cycles int     `json:"cycles"`
		}
		if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
			http.Error(w, err.Error(), http.StatusBadRequest)
			return
		}
		if err := s.actions.RunSineTest(req.Motor, req.Amp12, req.FreqHz, req.Cycles); err != nil {
			http.Error(w, err.Error(), http.StatusBadRequest)
			return
		}
		w.Header().Set("Content-Type", "application/json")
		_, _ = w.Write([]byte(`{"ok":true}`))
	})

	return mux
}

func postAction(fn func() error) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "POST required", http.StatusMethodNotAllowed)
			return
		}
		if err := fn(); err != nil {
			http.Error(w, err.Error(), http.StatusBadRequest)
			return
		}
		w.Header().Set("Content-Type", "application/json")
		_, _ = w.Write([]byte(`{"ok":true}`))
	}
}
