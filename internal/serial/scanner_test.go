package serial

import (
	"testing"

	"go.bug.st/serial/enumerator"
)

func TestSelectArduinoPort(t *testing.T) {
	legacy := &enumerator.PortDetails{Name: "COM1"}
	uno := &enumerator.PortDetails{Name: "COM8", IsUSB: true, VID: "2341", PID: "0043"}
	clone := &enumerator.PortDetails{Name: "COM4", IsUSB: true, VID: "1A86", PID: "7523"}
	other := &enumerator.PortDetails{Name: "COM9", IsUSB: true, VID: "303A", PID: "1001"}
	for _, tt := range []struct {
		name  string
		ports []*enumerator.PortDetails
		want  string
	}{
		{"no devices", nil, ""},
		{"legacy COM1 excluded", []*enumerator.PortDetails{legacy}, ""},
		{"Uno preferred over unrelated USB and COM1", []*enumerator.PortDetails{legacy, other, uno}, "COM8"},
		{"clone with COM1", []*enumerator.PortDetails{legacy, clone}, "COM4"},
		{"ambiguous USB adapters", []*enumerator.PortDetails{clone, other}, ""},
		{"ambiguous Unos", []*enumerator.PortDetails{uno, {Name: "COM5", IsUSB: true, VID: "2a03", PID: "0043"}}, ""},
		{"original Uno", []*enumerator.PortDetails{{Name: "COM3", IsUSB: true, VID: "2341", PID: "0001"}}, "COM3"},
		{"nil metadata ignored", []*enumerator.PortDetails{nil, uno}, "COM8"},
	} {
		t.Run(tt.name, func(t *testing.T) {
			got, err := selectArduinoPort(tt.ports)
			if got != tt.want || (err != nil) != (tt.want == "") {
				t.Fatalf("got %q, %v; want %q", got, err, tt.want)
			}
		})
	}
}
