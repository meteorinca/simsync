package api

import (
	"github.com/meteorinca/simsync/internal/serial"
	"net/http/httptest"
	"strings"
	"testing"
	"testing/fstest"
)

func TestCommandsRequirePostAndHeader(t *testing.T) {
	calls := 0
	s := New(7070, NewHandlers(&Deps{Arm: func(bool) error { calls++; return nil }}), fstest.MapFS{})
	for _, tc := range []struct {
		method, header string
		code           int
	}{{"GET", "", 405}, {"POST", "", 405}, {"POST", "1", 200}} {
		r := httptest.NewRequest(tc.method, "/api/arm?state=1", nil)
		if tc.header != "" {
			r.Header.Set("X-SimSync", tc.header)
		}
		w := httptest.NewRecorder()
		s.Handler().ServeHTTP(w, r)
		if w.Code != tc.code {
			t.Fatalf("%s: %d", tc.method, w.Code)
		}
	}
	if calls != 1 {
		t.Fatal(calls)
	}
}

func TestMalformedStepNeverRuns(t *testing.T) {
	calls := 0
	h := NewHandlers(&Deps{RunStepTest: func(int, float32, float32, float32, float32) error { calls++; return nil }})
	for _, query := range []string{"joint=1&kp=1&ki=0&kd=0&step=bad", "joint=0&kp=1&ki=0&kd=0&step=8"} {
		w := httptest.NewRecorder()
		h.StepTest(w, httptest.NewRequest("POST", "/api/steptest?"+query, nil))
		if w.Code != 400 {
			t.Fatal(w.Code)
		}
	}
	if calls != 0 {
		t.Fatal(calls)
	}
}

func TestPolarityEndpointRemoved(t *testing.T) {
	s := New(7070, NewHandlers(&Deps{}), fstest.MapFS{})
	r := httptest.NewRequest("POST", "/api/direction/check", nil)
	r.Header.Set("X-SimSync", "1")
	w := httptest.NewRecorder()
	s.Handler().ServeHTTP(w, r)
	if w.Code != 404 {
		t.Fatalf("removed polarity endpoint returned %d", w.Code)
	}
}

func TestTuningRoutesAndValidation(t *testing.T) {
	calls := 0
	s := New(7070, NewHandlers(&Deps{
		TuneController: func(serial.ControllerTuning) error { calls++; return nil },
		RunSignal:      func(int, string, float32, float32) error { calls++; return nil },
	}), fstest.MapFS{})
	for _, path := range []string{"/api/controller/tuning", "/api/signal"} {
		for _, tc := range []struct {
			method, header, body string
			code                 int
		}{{"GET", "", "{}", 405}, {"POST", "", "{}", 405}, {"POST", "1", "invalid", 400}, {"POST", "1", "{}", 200}} {
			req := httptest.NewRequest(tc.method, path, strings.NewReader(tc.body))
			req.Header.Set("X-SimSync", tc.header)
			w := httptest.NewRecorder()
			s.Handler().ServeHTTP(w, req)
			if w.Code != tc.code {
				t.Fatalf("%s %s: %d", tc.method, path, w.Code)
			}
		}
	}
	if calls != 2 {
		t.Fatal(calls)
	}
}

func TestFiltersAndSoloUseValidatedProtectedCommands(t *testing.T) {
	filters, solo := 0, 0
	s := New(7070, NewHandlers(&Deps{
		SetMotorFilters: func(v serial.MotorFilters) error {
			filters++
			if v.Joint != 2 || v.LPFHz != 80 || v.HPFHz != 0 {
				t.Fatal(v)
			}
			return nil
		},
		SoloMotor: func(j int) error {
			solo++
			if j != 2 {
				t.Fatal(j)
			}
			return nil
		},
	}), fstest.MapFS{})
	for _, tc := range []struct {
		method, path, body, header string
		code                       int
	}{
		{"GET", "/api/motor/filters", "", "", 405},
		{"POST", "/api/motor/filters", `{"joint":2,"lpf_hz":80}`, "", 405},
		{"POST", "/api/motor/filters", `{"joint":2,"lpf_hz":80,"hpf_hz":0}`, "1", 200},
		{"POST", "/api/motor/filters", `{"joint":2,"lpf_hz":-1}`, "1", 400},
		{"POST", "/api/motor/solo?joint=0", "", "1", 400},
		{"POST", "/api/motor/solo?joint=2", "", "1", 200},
	} {
		req := httptest.NewRequest(tc.method, tc.path, strings.NewReader(tc.body))
		req.Header.Set("X-SimSync", tc.header)
		w := httptest.NewRecorder()
		s.Handler().ServeHTTP(w, req)
		if w.Code != tc.code {
			t.Fatalf("%s %s returned %d: %s", tc.method, tc.path, w.Code, w.Body.String())
		}
	}
	if filters != 1 || solo != 1 {
		t.Fatal(filters, solo)
	}
}
