package motionconstraints

import (
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
)

// Config is the JSON config schema for the planner service. Phase 2 fleshes
// this out; the skeleton accepts an empty config so the module loads.
type Config struct {
	// MotionService is the resource name of the builtin motion service.
	MotionService string `json:"motion_service,omitempty"`

	// Arms is the list of arm component names to orchestrate.
	Arms []string `json:"arms,omitempty"`

	// Loop controls whether scenarios cycle automatically (default true).
	Loop *bool `json:"loop,omitempty"`

	// IntervalS is the pause in seconds between scenarios in loop mode.
	IntervalS float64 `json:"interval_s,omitempty"`

	// Presets selects which built-in scenarios to run in order on the
	// FIRST arm in legacy sequential mode. Ignored when ArmScenarios is
	// non-empty (parallel-per-arm mode takes precedence).
	Presets []string `json:"presets,omitempty"`

	// ArmScenarios binds each arm to a preset key. When non-empty, the
	// scenario loop runs in parallel: one goroutine per (arm, scenario)
	// pair, each on its own interval. Arms not listed in this map idle.
	// Mutually exclusive with Presets at runtime — if both are set,
	// ArmScenarios wins and Presets is ignored.
	ArmScenarios map[string]string `json:"arm_scenarios,omitempty"`

	// EEFrames maps an arm name to a non-default end-effector frame to
	// plan against (e.g. a gripper tip attached as a child frame in the
	// machine config). When unset for an arm, the planner targets the
	// arm's own kinematic-chain output frame (i.e. the arm name).
	EEFrames map[string]string `json:"ee_frames,omitempty"`

	// PresetSet selects a named bundle of (arm, scenario) mappings
	// shipped with the module. Set this instead of ArmScenarios for the
	// common case of "I want the canonical demo for these rows". Valid
	// values: "ee_variations" (rows A+AB, default), "obstacle_geometry"
	// (row B), "constraint_types" (row C), "ee_only" (row A), "all"
	// (every preset). ArmScenarios still wins if both are set.
	PresetSet string `json:"preset_set,omitempty"`

	// AbortOnCollision: if a pre-flight collision check trips, skip execution.
	AbortOnCollision *bool `json:"abort_on_collision,omitempty"`

	// TickHz is the visualization tick rate. Default 30; max 30.
	TickHz float64 `json:"tick_hz,omitempty"`

	// PreviewDensity is the number of interpolated joint samples to take
	// between consecutive planner-returned waypoints when rendering the
	// ghost trajectory. Higher = smoother trail; lower = lighter scene.
	// Default 15. Set to 1 to fall back to keyframes-only.
	PreviewDensity int `json:"preview_density,omitempty"`

	// MaxConcurrentPlans bounds how many armplanning.PlanMotion calls may
	// be in flight at once across all arms. The cbirrt planner spawns up
	// to runtime.NumCPU()/2 worker goroutines per call; running every arm
	// in parallel can saturate the Go runtime inside viam-server, which
	// starves the WebRTC goroutines feeding the 3D scene viewer. A small
	// cap leaves scheduler headroom and keeps the viz responsive. 0 means
	// "use default" (2). Set higher (e.g. 4) if you have many idle cores
	// and don't care about viz smoothness.
	MaxConcurrentPlans int `json:"max_concurrent_plans,omitempty"`

	// DisablePreviewGhosts skips the trajectory-ghost emission entirely.
	// Kill-switch for diagnostics. Axes markers + goal marker still emit.
	DisablePreviewGhosts bool `json:"disable_preview_ghosts,omitempty"`

	// MaxPreviewGhosts caps the TOTAL number of trajectory-ghost spheres
	// emitted per plan, regardless of trajectory length. cbirrt under a
	// LinearConstraint returns 100+ waypoints (it samples densely to
	// verify the constraint); emitting all of them × preview_density in
	// a tight loop produces a TransformChange burst that locks the 3D
	// viewer's JS main thread for the duration. With a cap, we
	// down-sample evenly across the densified samples to keep the burst
	// to a tractable size. Default 24 — enough to read the trail shape
	// clearly, small enough that the browser doesn't stall. 0 means
	// "use default"; -1 means "uncapped" (legacy behavior).
	MaxPreviewGhosts int `json:"max_preview_ghosts,omitempty"`
}

// Validate is called by the resource graph when the service is (re)configured.
// It returns the explicit dependency list (so the graph can sequence startup)
// plus an optional implicit dependency list (unused here).
//
// Skeleton validation: just enforce that required fields are present when
// any of them are set. Until Phase 2 wires real behavior, an empty config is
// also accepted so the module can be loaded standalone.
func (c *Config) Validate(path string) ([]string, []string, error) {
	if c == nil {
		return nil, nil, nil
	}

	deps := []string{}
	if c.MotionService != "" {
		deps = append(deps, c.MotionService)
	}
	deps = append(deps, c.Arms...)
	// The framesystem service is auto-injected for modules; declaring it
	// here ensures the resource graph waits for it before constructing us
	// (so framesystem.FromDependencies succeeds inside Reconfigure).
	deps = append(deps, framesystem.PublicServiceName.String())

	// motion_service is optional. We don't actually call it for the
	// scripted scenarios (we use armplanning.PlanMotion directly), so
	// configurations that omit the service block entirely still work.

	return deps, nil, nil
}

// ensure Config implements the resource.ConfigValidator interface at compile time.
var _ resource.ConfigValidator = (*Config)(nil)
