package api

import (
	"net/http/httptest"
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
