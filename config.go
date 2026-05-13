package motionconstraints

import (
	"errors"

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

	// Presets selects which built-in scenarios to run in order.
	Presets []string `json:"presets,omitempty"`

	// AbortOnCollision: if a pre-flight collision check trips, skip execution.
	AbortOnCollision *bool `json:"abort_on_collision,omitempty"`

	// TickHz is the visualization tick rate. Default 30; max 30.
	TickHz float64 `json:"tick_hz,omitempty"`
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

	if len(c.Arms) > 0 && c.MotionService == "" {
		return deps, nil, errors.New(
			path + ": arms configured but motion_service is required for planning")
	}

	return deps, nil, nil
}

// ensure Config implements the resource.ConfigValidator interface at compile time.
var _ resource.ConfigValidator = (*Config)(nil)
