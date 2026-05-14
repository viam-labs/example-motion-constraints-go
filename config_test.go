package motionconstraints

import (
	"testing"

	"go.viam.com/rdk/robot/framesystem"
)

// fsDep is the always-included framesystem dependency. Helper to keep test
// expectations readable as the dep list grows.
var fsDep = framesystem.PublicServiceName.String()

// emptyConfigPasses confirms that an empty config validates and reports
// only the auto-included framesystem dependency. The module ships with a
// working empty-config startup so the registry can verify it loads before
// any machine-config attributes exist.
func TestValidate_EmptyConfigPasses(t *testing.T) {
	c := &Config{}
	deps, optional, err := c.Validate("svc.attributes")
	if err != nil {
		t.Fatalf("empty config should validate, got %v", err)
	}
	if len(deps) != 1 || deps[0] != fsDep {
		t.Fatalf("empty config should depend only on the framesystem service, got %v", deps)
	}
	if len(optional) != 0 {
		t.Fatalf("expected no optional deps, got %v", optional)
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
	want := map[string]bool{
		"builtin-motion": true,
		"arm_nw":         true,
		"arm_ne":         true,
		fsDep:            true,
	}
	if len(deps) != len(want) {
		t.Fatalf("expected %d deps, got %d (%v)", len(want), len(deps), deps)
	}
	for _, d := range deps {
		if !want[d] {
			t.Errorf("unexpected dep %q", d)
		}
	}
}

// motion_service is optional — we use armplanning.PlanMotion directly,
// so configurations that omit it must still validate successfully.
func TestValidate_ArmsWithoutMotionServiceOK(t *testing.T) {
	c := &Config{Arms: []string{"arm0"}}
	deps, _, err := c.Validate("svc.attributes")
	if err != nil {
		t.Fatalf("arms without motion_service should validate, got %v", err)
	}
	want := map[string]bool{"arm0": true, fsDep: true}
	if len(deps) != len(want) {
		t.Fatalf("expected %d deps (arm + framesystem), got %d (%v)", len(want), len(deps), deps)
	}
	for _, d := range deps {
		if !want[d] {
			t.Errorf("unexpected dep %q", d)
		}
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
// `list` verb hands this catalog to callers, so the list must stay in sync
// with README.md and the presetByKey switch.
func TestBuiltinPresetsCatalog(t *testing.T) {
	want := map[string]bool{
		"single_arm_obstacle":    true,
		"linear_constraint":      true,
		"orientation_constraint": true,
		"dynamic_obstacle":       true,
		"multi_arm_choreography": true,
		"obstacle_progression":   true,
		"random_translation":     true,
		"random_rotation":        true,
		"arc_over_obstacle":         true,
		"duck_under_obstacle":       true,
		"gripper_with_box":          true,
		"corridor_passthrough":      true,
		"random_translation_linear": true,
		"random_rotation_linear":    true,
	}
	if len(builtinPresets) != len(want) {
		t.Fatalf("preset catalog size drift: got %v", builtinPresets)
	}
	for _, k := range builtinPresets {
		if !want[k] {
			t.Errorf("unexpected preset key %q", k)
		}
		if presetByKey(k) == nil {
			t.Errorf("preset %q listed but presetByKey returns nil", k)
		}
	}
}
