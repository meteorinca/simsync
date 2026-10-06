package serial

import (
	"fmt"
	"strings"

	"go.bug.st/serial/enumerator"
)

// Prefer Uno USB identities, with a single USB adapter fallback for clones.
// The connection still must answer the SMC3 version probe before becoming ready.
func ScanForController() (string, error) {
	ports, err := enumerator.GetDetailedPortsList()
	if err != nil {
		return "", err
	}
	return selectArduinoPort(ports)
}

func selectArduinoPort(ports []*enumerator.PortDetails) (string, error) {
	var unos, usb []string
	for _, p := range ports {
		if p == nil || !p.IsUSB || p.Name == "" {
			continue
		}
		usb = append(usb, p.Name)
		vid, pid := strings.ToUpper(p.VID), strings.ToUpper(p.PID)
		// Arduino AVR boards.txt lists these Uno identities.
		if (vid == "2341" && (pid == "0001" || pid == "0043" || pid == "0243" || pid == "006A")) ||
			(vid == "2A03" && pid == "0043") {
			unos = append(unos, p.Name)
		}
	}
	if len(unos) == 1 {
		return unos[0], nil
	}
	if len(unos) > 1 {
		return "", fmt.Errorf("multiple Arduino Uno ports (%s); select one with --serial <port>", strings.Join(unos, ", "))
	}
	if len(usb) == 1 {
		return usb[0], nil
	}
	if len(usb) > 1 {
		return "", fmt.Errorf("multiple USB serial candidates (%s); select the Arduino with --serial <port>", strings.Join(usb, ", "))
	}
	return "", fmt.Errorf("no Arduino USB serial candidate found; connect the Uno or specify --serial <port> (Linux: /dev/ttyACM0 or /dev/ttyUSB0; Windows: COM9)")
}
