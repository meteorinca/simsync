// Package session provides a thread-safe in-memory ring buffer for storing
// live telemetry samples, joint states, and step test records.
package session

import (
	"encoding/csv"
	"fmt"
	"io"
	"sync"
	"time"
)

// Sample is a single timestamped observation of one joint.
type Sample struct {
	T      time.Time
	Joint  int
	Target float32
	Actual float32
	Error  float32
	Duty   int16
	Kp     float32
	Ki     float32
	Kd     float32

	// Telemetry fields (filled when a game bridge packet triggered this sample)
	Speed float32
	GLat  float32
	GLong float32
}

// Store is the ring buffer.
type Store struct {
	mu   sync.RWMutex
	buf  []Sample
	head int
	size int
	cap  int
}

// New creates a Store that holds up to cap samples.
func New(cap int) *Store {
	return &Store{buf: make([]Sample, cap), cap: cap}
}

// Add appends a sample, overwriting the oldest when full.
func (s *Store) Add(sample Sample) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.buf[s.head] = sample
	s.head = (s.head + 1) % s.cap
	if s.size < s.cap {
		s.size++
	}
}

// Last returns the most recent n samples across all joints,
// ordered oldest-first.
func (s *Store) Last(n int) []Sample {
	s.mu.RLock()
	defer s.mu.RUnlock()
	if n > s.size {
		n = s.size
	}
	out := make([]Sample, n)
	start := ((s.head - n) + s.cap) % s.cap
	for i := 0; i < n; i++ {
		out[i] = s.buf[(start+i)%s.cap]
	}
	return out
}

// Since returns all samples added after t, ordered oldest-first.
func (s *Store) Since(t time.Time) []Sample {
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := []Sample{}
	start := ((s.head - s.size) + s.cap) % s.cap
	for i := 0; i < s.size; i++ {
		sa := s.buf[(start+i)%s.cap]
		if sa.T.After(t) {
			out = append(out, sa)
		}
	}
	return out
}

// ForJoint returns the last n samples for a specific joint.
func (s *Store) ForJoint(joint, n int) []Sample {
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := []Sample{}
	start := ((s.head - s.size) + s.cap) % s.cap
	for i := 0; i < s.size; i++ {
		sa := s.buf[(start+i)%s.cap]
		if sa.Joint == joint {
			out = append(out, sa)
			if len(out) >= n {
				break
			}
		}
	}
	return out
}

// ExportCSV writes all buffered samples to w in CSV format.
func (s *Store) ExportCSV(w io.Writer) error {
	s.mu.RLock()
	samples := s.Last(s.size)
	s.mu.RUnlock()

	cw := csv.NewWriter(w)
	header := []string{"timestamp_ms", "joint", "target_counts", "actual_counts", "error_counts", "duty", "kp", "ki", "kd", "speed_ms", "g_lat", "g_long"}
	if err := cw.Write(header); err != nil {
		return err
	}
	for _, sa := range samples {
		row := []string{
			fmt.Sprintf("%d", sa.T.UnixMilli()),
			fmt.Sprintf("%d", sa.Joint),
			fmt.Sprintf("%.3f", sa.Target),
			fmt.Sprintf("%.3f", sa.Actual),
			fmt.Sprintf("%.3f", sa.Error),
			fmt.Sprintf("%d", sa.Duty),
			fmt.Sprintf("%.3f", sa.Kp),
			fmt.Sprintf("%.3f", sa.Ki),
			fmt.Sprintf("%.3f", sa.Kd),
			fmt.Sprintf("%.3f", sa.Speed),
			fmt.Sprintf("%.4f", sa.GLat),
			fmt.Sprintf("%.4f", sa.GLong),
		}
		if err := cw.Write(row); err != nil {
			return err
		}
	}
	cw.Flush()
	return cw.Error()
}
