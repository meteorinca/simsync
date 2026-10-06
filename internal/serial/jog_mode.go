package serial

import (
	"fmt"
	"time"
)

// Jog mode is explicit, session-local and cancelled on disconnect.
func (p *Port) SetDiagnostics(enabled bool) error {
	p.mu.Lock()
	defer p.mu.Unlock()
	if !p.jogSupported {
		return fmt.Errorf("upload SMC3 firmware with timed jog support")
	}
	p.diagnostics = false
	if err := p.writeLocked([]byte("[stp]")); err != nil {
		return err
	}
	p.jogUntil = time.Time{}
	p.diagnostics = enabled
	return nil
}

func (p *Port) Diagnostics() bool {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.diagnostics
}
