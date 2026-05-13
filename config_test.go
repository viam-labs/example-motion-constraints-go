package motionconstraints

import (
	"strings"
	"testing"
)

// emptyConfigPasses confirms that an empty config validates and reports no
// dependencies. The module ships with a working empty-config startup so the
// registry can verify it loads before any machine-config attributes exist.
func TestValidate_EmptyConfigPasses(t *testing.T) {
	c := &Config{}
	deps, optional, err := c.Validate("svc.attributes")
	if err != nil {
		t.Fatalf("empty config should validate, got %v", err)
	}
	if len(deps) != 0 || len(optional) != 0 {
		t.Fatalf("empty config should report no deps, got deps=%v optional=%v", deps, optional)
	}
}

func TestValidate_DepsIncludeArmsAndMotionService(t *testing.T) {
	c := &Config{
		MotionService: "builtin-motion",
		Arms:          []string{"arm_nw", "arm_ne"},
	}
	deps, _, err := c.Validate("svc.attributes")
	if err != nil {
		t.Fatalf("valid config should not error: %v", err)
	}
	want := map[string]bool{"builtin-motion": true, "arm_nw": true, "arm_ne": true}
	if len(deps) != len(want) {
		t.Fatalf("expected %d deps, got %d (%v)", len(want), len(deps), deps)
	}
	for _, d := range deps {
		if !want[d] {
			t.Errorf("unexpected dep %q", d)
		}
	}
}

// ArmsWithoutMotionService is a likely misconfiguration: the user intends to
// plan motion but forgot to declare the motion service dependency. We catch
// this at validate time so the user sees the issue before the service starts.
func TestValidate_ArmsWithoutMotionServiceErrors(t *testing.T) {
	c := &Config{
		Arms: []string{"arm0"},
	}
	_, _, err := c.Validate("svc.attributes")
	if err == nil {
		t.Fatal("arms without motion_service should error")
	}
	if !strings.Contains(err.Error(), "motion_service") {
		t.Errorf("error should mention motion_service, got %q", err.Error())
	}
}

func TestValidate_NilConfigOK(t *testing.T) {
	var c *Config
	deps, optional, err := c.Validate("svc.attributes")
	if err != nil || deps != nil || optional != nil {
		t.Fatalf("nil config should pass cleanly, got deps=%v optional=%v err=%v", deps, optional, err)
	}
}

// BuiltinPresetsCatalog spot-checks the canonical preset key list. The DoCommand
// `list` verb hands this catalog to callers before the scenario implementations
// exist, so the list must stay in sync with NOTES.md / README.md.
func TestBuiltinPresetsCatalog(t *testing.T) {
	want := map[string]bool{
		"single_arm_obstacle":    true,
		"linear_constraint":      true,
		"orientation_constraint": true,
		"dynamic_obstacle":       true,
		"multi_arm_choreography": true,
	}
	if len(builtinPresets) != len(want) {
		t.Fatalf("preset catalog size drift: got %v", builtinPresets)
	}
	for _, k := range builtinPresets {
		if !want[k] {
			t.Errorf("unexpected preset key %q", k)
		}
	}
}
