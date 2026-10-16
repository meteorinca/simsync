package settings

import (
	"bytes"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"testing"
)

func TestCreateSaveAndRestart(t *testing.T) {
	path := filepath.Join(t.TempDir(), "simsync-settings.md")
	s, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	c := s.Snapshot()
	if !c.Motors[0].Active || !c.Motors[1].Active || c.Motors[2].Active {
		t.Fatal("default profile must include M1 and M2, with M3 optional")
	}
	if c.Motors[0].Center != 338 || c.Motors[0].Kp != 20 || c.Motors[0].Kd != .4 || c.Motion.PitchNeutral != 338 {
		t.Fatal(c)
	}
	if err := s.Update(func(c *Config) {
		c.Motors[1].Kp = 7.25
		c.Motors[1].Clip = 60
		c.Motors[1].Min = 70
		c.Motors[1].Ks = 8
		c.Motors[1].Active = true
		c.Motors[1].Center = 400
		c.Timing.Divider = 3
		c.Motion.RollGain = 17
	}); err != nil {
		t.Fatal(err)
	}
	reopened, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	next := reopened.Snapshot()
	if next.Motors[1].Kp != 7.25 || next.Motors[1].Clip != 60 || next.Motors[1].Ks != 8 || !next.Motors[1].Active || next.Motion.RollNeutral != 400 || next.Timing.Divider != 3 || next.Motion.RollGain != 17 {
		t.Fatal(next)
	}
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	edited := bytes.Replace(data, []byte("| P | 20 | 7.25 | 20 |"), []byte("| P | 22 | 7.25 | 20 |"), 1)
	if err := os.WriteFile(path, edited, 0644); err != nil {
		t.Fatal(err)
	}
	reopened, err = Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if reopened.Snapshot().Motors[0].Kp != 22 {
		t.Fatal("human edit not loaded")
	}
}

func TestLegacyProfilesDefaultFiltersOffAndPreservePerMotorCutoffs(t *testing.T) {
	old := string(Marshal(Defaults()))
	old = strings.ReplaceAll(old, "| Feedback LPF Hz | 0 | 0 | 0 |\n", "")
	old = strings.ReplaceAll(old, "| Derivative HPF Hz | 0 | 0 | 0 |\n", "")
	c, err := Parse([]byte(old))
	if err != nil {
		t.Fatal(err)
	}
	for _, m := range c.Motors {
		if m.LPFHz != 0 || m.HPFHz != 0 {
			t.Fatal("legacy filters must be bypassed")
		}
	}
	c.Motors[1].LPFHz, c.Motors[1].HPFHz = 80, 2
	next, err := Parse(Marshal(c))
	if err != nil {
		t.Fatal(err)
	}
	if next.Motors[1].LPFHz != 80 || next.Motors[1].HPFHz != 2 || next.Motors[0].LPFHz != 0 {
		t.Fatal(next)
	}
}
func TestInvalidFileIsNotOverwritten(t *testing.T) {
	data := string(Marshal(Defaults()))
	for _, bad := range []string{strings.Replace(data, "| Center | 338 | 338 | 338 |", "| Center | 700 | 338 | 338 |", 1), strings.Replace(data, "| PID divider | 1 |", "| PID divider | 0 |", 1), strings.Replace(data, "| P | 20 | 20 | 20 |", "| P | NaN | 20 | 20 |", 1), data + "\n| Filter Hz | 12 |\n", strings.Replace(data, "| Clip margin | 40 | 40 | 40 |\n", "", 1), "# Empty settings"} {
		path := filepath.Join(t.TempDir(), "simsync-settings.md")
		if err := os.WriteFile(path, []byte(bad), 0644); err != nil {
			t.Fatal(err)
		}
		if _, err := Open(path); err == nil {
			t.Fatal("invalid file accepted", bad)
		}
		actual, _ := os.ReadFile(path)
		if string(actual) != bad {
			t.Fatal("invalid file overwritten")
		}
	}
}
func TestUpdatesAreAtomicAndSerialized(t *testing.T) {
	directory := t.TempDir()
	path := filepath.Join(directory, "simsync-settings.md")
	s, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	original, _ := os.ReadFile(path)
	if s.Update(func(c *Config) { c.Motors[0].Center = 0 }) == nil {
		t.Fatal("invalid update accepted")
	}
	after, _ := os.ReadFile(path)
	if !bytes.Equal(original, after) {
		t.Fatal("invalid update changed file")
	}
	var group sync.WaitGroup
	for motor := 0; motor < 3; motor++ {
		group.Add(1)
		go func(i int) {
			defer group.Done()
			if err := s.Update(func(c *Config) { c.Motors[i].Kp = float32(30 + i) }); err != nil {
				t.Error(err)
			}
		}(motor)
	}
	group.Wait()
	reopened, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	for i, m := range reopened.Snapshot().Motors {
		if m.Kp != float32(30+i) {
			t.Fatal("concurrent update lost", i, m.Kp)
		}
	}
	before := s.Snapshot()
	moved := directory + "-moved"
	if err := os.Rename(directory, moved); err != nil {
		t.Fatal(err)
	}
	defer os.RemoveAll(moved)
	if s.Update(func(c *Config) { c.Motors[0].Kp = 40 }) == nil {
		t.Fatal("write failure hidden")
	}
	if s.Snapshot().Motors[0].Kp != before.Motors[0].Kp {
		t.Fatal("failed write committed in memory")
	}
}
