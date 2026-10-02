package serial

import (
	"fmt"
	goserial "go.bug.st/serial"
)

// Require an explicit choice when multiple serial devices are present.
func ScanForController() (string, error) {
	ports, err := goserial.GetPortsList()
	if err != nil {
		return "", err
	}
	if len(ports) != 1 {
		return "", fmt.Errorf("found %d ports; select the Uno with --serial COMx", len(ports))
	}
	return ports[0], nil
}
