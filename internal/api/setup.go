package api

import (
	"encoding/json"
	"net/http"
)

func (h *Handlers) Setup(w http.ResponseWriter, r *http.Request) {
	if h.d.GetSetup == nil || h.d.SetSetup == nil {
		writeErr(w, 503, "Setup unavailable")
		return
	}
	if r.Method == http.MethodGet {
		writeJSON(w, h.d.GetSetup())
		return
	}
	var v struct {
		Joint     int  `json:"joint"`
		Direction bool `json:"direction"`
		Tuned     bool `json:"tuned"`
	}
	if json.NewDecoder(http.MaxBytesReader(w, r.Body, 4096)).Decode(&v) != nil || v.Joint < 1 || v.Joint > 3 {
		writeErr(w, 400, "Choose motor 1, 2 or 3")
		return
	}
	if err := h.d.SetSetup(v.Joint, v.Direction, v.Tuned); err != nil {
		writeErr(w, 409, err.Error())
		return
	}
	writeJSON(w, map[string]bool{"ok": true})
}

func (h *Handlers) Connection(w http.ResponseWriter, r *http.Request) {
	if h.d.ConnectionInfo == nil || h.d.SetConnection == nil {
		writeErr(w, 503, "Connection unavailable")
		return
	}
	if r.Method == http.MethodGet {
		info, err := h.d.ConnectionInfo()
		if err != nil {
			writeErr(w, 500, err.Error())
			return
		}
		writeJSON(w, info)
		return
	}
	var v struct {
		Port  string `json:"port"`
		Bench bool   `json:"bench"`
	}
	if json.NewDecoder(http.MaxBytesReader(w, r.Body, 4096)).Decode(&v) != nil {
		writeErr(w, 400, "Invalid connection settings")
		return
	}
	if err := h.d.SetConnection(v.Port, v.Bench); err != nil {
		writeErr(w, 409, err.Error())
		return
	}
	writeJSON(w, map[string]bool{"ok": true})
}
