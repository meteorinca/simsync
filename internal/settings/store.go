// Package settings persists the restart profile as editable Markdown tables.
package settings

import (
	"bufio"
	"fmt"
	"os"
	"path/filepath"
	"reflect"
	"strconv"
	"strings"
	"sync"

	"github.com/meteorinca/simsync/internal/motion"
	"github.com/meteorinca/simsync/internal/serial"
)

type SetupProgress struct {
	Direction bool `json:"direction"`
	Tuned     bool `json:"tuned"`
}

type Config struct {
	Setup  [3]SetupProgress
	Motors [3]serial.MotorSettings
	Timing serial.ControllerTuning
	Motion motion.Config
}

func Defaults() Config {
	c := Config{Timing: serial.ControllerTuning{Joint: 1, Ks: 1, Divider: 1, PWM12: 25, PWM3: 31}, Motion: motion.DefaultConfig()}
	for i := range c.Motors {
		c.Motors[i] = serial.BenchDefaults()
		c.Motors[i].Active = i < 2
	}
	c.centers()
	return c
}
func (c *Config) centers() {
	c.Motion.PitchNeutral = c.Motors[0].Center
	c.Motion.RollNeutral = c.Motors[1].Center
	c.Motion.HeaveNeutral = c.Motors[2].Center
}
func (c Config) Validate() error {
	for i, m := range c.Motors {
		if err := m.Validate(); err != nil {
			return fmt.Errorf("M%d: %w", i+1, err)
		}
		if m.Ks < 1 || m.Ks > 100 {
			return fmt.Errorf("M%d: Ks must be 1-100", i+1)
		}
	}
	t := c.Timing
	t.Joint = 1
	t.Ks = c.Motors[0].Ks
	if err := t.Validate(); err != nil {
		return err
	}
	return c.Motion.Validate()
}

type field struct{ label, name string }

var motorFields = []field{{"P", "Kp"}, {"I", "Ki"}, {"D", "Kd"}, {"Ks", "Ks"}, {"Feedback LPF Hz", "LPFHz"}, {"Derivative HPF Hz", "HPFHz"}, {"PWM min", "PWMMin"}, {"PWM max", "PWMMax"}, {"PWM reverse", "PWMRev"}, {"Deadzone", "Deadzone"}, {"Cutoff margin", "Cutoff"}, {"Clip margin", "Clip"}, {"Host min", "Min"}, {"Host max", "Max"}, {"Center", "Center"}, {"Include in output", "Active"}}
var timingFields = []field{{"PID divider", "Divider"}, {"PWM M1/M2 kHz", "PWM12"}, {"PWM M3 kHz", "PWM3"}}
var motionFields = []field{{"Pitch gain", "PitchGain"}, {"Roll gain", "RollGain"}, {"Heave gain", "HeaveGain"}, {"Pitch limit", "PitchLimit"}, {"Roll limit", "RollLimit"}, {"Heave limit", "HeaveLimit"}, {"Filter Hz", "FilterHz"}}

func textField(object any, f field) string {
	return fmt.Sprint(reflect.ValueOf(object).FieldByName(f.name).Interface())
}
func setField(object any, f field, text string) error {
	v := reflect.ValueOf(object).Elem().FieldByName(f.name)
	switch v.Kind() {
	case reflect.Float32:
		n, err := strconv.ParseFloat(text, 32)
		if err != nil {
			return err
		}
		v.SetFloat(n)
	case reflect.Int:
		n, err := strconv.Atoi(text)
		if err != nil {
			return err
		}
		v.SetInt(int64(n))
	case reflect.Bool:
		if text != "true" && text != "false" {
			return fmt.Errorf("use true or false")
		}
		v.SetBool(text == "true")
	}
	return nil
}

var setupFields = []field{{"Direction confirmed", "Direction"}, {"Tune saved", "Tuned"}}

func Marshal(c Config) []byte {
	var b strings.Builder
	b.WriteString("# SimSync settings\n\nLoaded at startup; UI Apply changes save here automatically. Edit while SimSync is stopped.\nPositions and margins are ADC counts. Startup stays paused.\n\n## Motors\n\n| Setting | M1 | M2 | M3 |\n| --- | --- | --- | --- |\n")
	for _, f := range motorFields {
		fmt.Fprintf(&b, "| %s | %s | %s | %s |\n", f.label, textField(c.Motors[0], f), textField(c.Motors[1], f), textField(c.Motors[2], f))
	}
	for _, section := range []struct {
		title  string
		object any
		fields []field
	}{{"Controller", c.Timing, timingFields}, {"Motion", c.Motion, motionFields}} {
		fmt.Fprintf(&b, "\n## %s\n\n| Setting | Value |\n| --- | --- |\n", section.title)
		for _, f := range section.fields {
			fmt.Fprintf(&b, "| %s | %s |\n", f.label, textField(section.object, f))
		}
	}
	b.WriteString("\n## Setup\n\n| Setting | M1 | M2 | M3 |\n| --- | --- | --- | --- |\n")
	for _, f := range setupFields {
		fmt.Fprintf(&b, "| %s | %s | %s | %s |\n", f.label, textField(c.Setup[0], f), textField(c.Setup[1], f), textField(c.Setup[2], f))
	}
	return []byte(b.String())
}
func Parse(data []byte) (Config, error) {
	c := Defaults()
	section := ""
	seen := map[string]bool{}
	scanner := bufio.NewScanner(strings.NewReader(string(data)))
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		if strings.HasPrefix(line, "## ") {
			section = strings.TrimSpace(strings.TrimPrefix(line, "## "))
			continue
		}
		if !strings.HasPrefix(line, "|") {
			continue
		}
		cells := strings.Split(strings.Trim(line, "|"), "|")
		for i := range cells {
			cells[i] = strings.TrimSpace(cells[i])
		}
		if cells[0] == "Setting" || strings.HasPrefix(cells[0], "---") {
			continue
		}
		var fields []field
		var objects []any
		switch section {
		case "Setup":
			fields = setupFields
			objects = []any{&c.Setup[0], &c.Setup[1], &c.Setup[2]}
		case "Motors":
			fields = motorFields
			objects = []any{&c.Motors[0], &c.Motors[1], &c.Motors[2]}
		case "Controller":
			fields = timingFields
			objects = []any{&c.Timing}
		case "Motion":
			fields = motionFields
			objects = []any{&c.Motion}
		default:
			return c, fmt.Errorf("unknown settings section %q", section)
		}
		if len(cells) != len(objects)+1 {
			return c, fmt.Errorf("%s: wrong number of values", cells[0])
		}
		key := section + "/" + cells[0]
		if seen[key] {
			return c, fmt.Errorf("duplicate setting %s", key)
		}
		found := false
		for _, f := range fields {
			if f.label == cells[0] {
				found = true
				for i, object := range objects {
					if err := setField(object, f, cells[i+1]); err != nil {
						return c, fmt.Errorf("%s: %w", key, err)
					}
				}
				break
			}
		}
		if !found {
			return c, fmt.Errorf("unknown setting %s", key)
		}
		seen[key] = true
	}
	if err := scanner.Err(); err != nil {
		return c, err
	}
	for _, s := range []struct {
		title  string
		fields []field
	}{{"Motors", motorFields}, {"Controller", timingFields}, {"Motion", motionFields}} {
		for _, f := range s.fields {
			// Older profiles predate per-motor filters; missing filter rows mean off.
			if s.title == "Motors" && (f.name == "LPFHz" || f.name == "HPFHz") {
				continue
			}
			if !seen[s.title+"/"+f.label] {
				return c, fmt.Errorf("missing setting %s/%s", s.title, f.label)
			}
		}
	}
	c.centers()
	return c, c.Validate()
}

type Store struct {
	mu     sync.Mutex
	path   string
	config Config
}

func Open(path string) (*Store, error) {
	absolute, err := filepath.Abs(path)
	if err != nil {
		return nil, err
	}
	s := &Store{path: absolute, config: Defaults()}
	data, err := os.ReadFile(absolute)
	if os.IsNotExist(err) {
		err = writeAtomic(absolute, Marshal(s.config))
	} else if err == nil {
		s.config, err = Parse(data)
	}
	if err != nil {
		return nil, fmt.Errorf("settings %s: %w", absolute, err)
	}
	return s, nil
}
func (s *Store) Path() string     { return s.path }
func (s *Store) Snapshot() Config { s.mu.Lock(); defer s.mu.Unlock(); return s.config }
func (s *Store) Update(change func(*Config)) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	next := s.config
	change(&next)
	next.centers()
	if err := next.Validate(); err != nil {
		return err
	}
	if err := writeAtomic(s.path, Marshal(next)); err != nil {
		return fmt.Errorf("settings not saved to %s: %w", s.path, err)
	}
	s.config = next
	return nil
}
func writeAtomic(path string, data []byte) (err error) {
	f, err := os.CreateTemp(filepath.Dir(path), ".simsync-settings-*.tmp")
	if err != nil {
		return err
	}
	defer os.Remove(f.Name())
	defer f.Close()
	if err = f.Chmod(0644); err != nil {
		return err
	}
	if _, err = f.Write(data); err != nil {
		return err
	}
	if err = f.Sync(); err != nil {
		return err
	}
	if err = f.Close(); err != nil {
		return err
	}
	return os.Rename(f.Name(), path)
}
