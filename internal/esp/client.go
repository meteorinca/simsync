// Package esp provides an HTTP REST client for the ESP32-S3 MotionSimBot API.
package esp

import (
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"time"
)

// Client talks to the ESP32 HTTP REST API over WiFi.
type Client struct {
	BaseURL    string
	httpClient *http.Client
}

// New creates a Client targeting baseURL (e.g. http://motionsimbot1.local).
func New(baseURL string) *Client {
	return &Client{BaseURL: baseURL, httpClient: &http.Client{Timeout: 3 * time.Second}}
}

func (c *Client) Arm(state bool) error {
	v := 0
	if state {
		v = 1
	}
	return c.get(fmt.Sprintf("/api/arm?state=%d", v), nil)
}

func (c *Client) EStop(trip bool) error {
	v := 0
	if trip {
		v = 1
	}
	return c.get(fmt.Sprintf("/api/estop?state=%d", v), nil)
}

func (c *Client) ClearEStop() error { return c.get("/api/estop/clear", nil) }

func (c *Client) SetTarget(joint int, angle float32) error {
	return c.get(fmt.Sprintf("/api/target?joint=%d&angle=%.2f", joint, angle), nil)
}

func (c *Client) SetPID(kp, ki, kd float32) error {
	return c.get(fmt.Sprintf("/api/pid?kp=%.3f&ki=%.3f&kd=%.3f", kp, ki, kd), nil)
}

func (c *Client) SavePID() error  { return c.get("/api/pid/save", nil) }
func (c *Client) Autotune(joint int) error { return c.get(fmt.Sprintf("/api/autotune?joint=%d", joint), nil) }
func (c *Client) Calibrate(joint int) error { return c.get(fmt.Sprintf("/api/calibrate?joint=%d", joint), nil) }

// AngleResponse mirrors /api/angle JSON.
type AngleResponse struct {
	Joint  int
	Actual float32
	Target float32
	Error  float32
	Duty   int16
	Raw    int
}

func (c *Client) GetAngle(joint int) (*AngleResponse, error) {
	var r AngleResponse
	if err := c.get(fmt.Sprintf("/api/angle?joint=%d", joint), &r); err != nil {
		return nil, err
	}
	return &r, nil
}

// PIDStats is the pid sub-object from /api/stats.
type PIDStats struct {
	Kp float32
	Ki float32
	Kd float32
}

// StatsResponse mirrors /api/stats JSON (abbreviated, no struct tags needed — parsed via map).
type StatsResponse struct {
	M1          JointState
	M2          JointState
	M3          JointState
	PID         PIDStats
	EStop       bool
	EStopCode   int
	EStopReason string
	Armed       bool
	PktRateHz   float32
	RSSI        int
	IP          string
	Version     string
}

// JointState holds per-joint data from /api/stats.
type JointState struct {
	Target float32
	Actual float32
	Error  float32
	Duty   int16
}

func (c *Client) Stats() (*StatsResponse, error) {
	resp, err := c.httpClient.Get(c.BaseURL + "/api/stats")
	if err != nil {
		return nil, err
	}
	defer resp.Body.Close()
	// Parse into a raw map to avoid struct tag issues
	var raw map[string]interface{}
	if err := json.NewDecoder(resp.Body).Decode(&raw); err != nil {
		return nil, err
	}
	s := &StatsResponse{}
	if m, ok := raw["pid"].(map[string]interface{}); ok {
		s.PID.Kp = float32v(m["kp"])
		s.PID.Ki = float32v(m["ki"])
		s.PID.Kd = float32v(m["kd"])
	}
	s.EStop, _ = raw["estop"].(bool)
	s.Armed, _ = raw["armed"].(bool)
	if code, ok := raw["estop_code"].(float64); ok {
		s.EStopCode = int(code)
	}
	s.EStopReason, _ = raw["estop_reason"].(string)
	s.PktRateHz = float32v(raw["pkt_rate_hz"])
	s.IP, _ = raw["ip"].(string)
	s.Version, _ = raw["version"].(string)
	return s, nil
}

func (c *Client) Available() bool { return c.get("/api/stats", nil) == nil }

func (c *Client) get(path string, out interface{}) error {
	resp, err := c.httpClient.Get(c.BaseURL + path)
	if err != nil {
		return err
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		body, _ := io.ReadAll(resp.Body)
		return fmt.Errorf("ESP32 HTTP %d: %s", resp.StatusCode, string(body))
	}
	if out != nil {
		return json.NewDecoder(resp.Body).Decode(out)
	}
	io.Copy(io.Discard, resp.Body)
	return nil
}

func float32v(v interface{}) float32 {
	if f, ok := v.(float64); ok {
		return float32(f)
	}
	return 0
}