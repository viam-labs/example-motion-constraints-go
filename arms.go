package motionconstraints

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/services/motion"
)

// resolved is the runtime view of the dependencies the planner service needs:
// the configured arms, the motion service (kept for callers who want
// service.Move semantics), the framesystem service (load-bearing —
// armplanning.PlanMotion needs a *referenceframe.FrameSystem), and the
// service's logger so plan helpers can emit diagnostics under the same
// resource name as the service itself.
type resolved struct {
	arms        map[string]arm.Arm
	armOrder    []string // preserve config order for grid scenarios
	motion      motion.Service
	frameSystem framesystem.Service
	logger      logging.Logger
}

// resolveDeps walks the dependency graph and returns the arms + motion
// service + framesystem service the configured names refer to. Returns a
// detailed error if anything is missing — these are configuration bugs
// the user wants to see at startup rather than during the first scenario.
func resolveDeps(deps resource.Dependencies, cfg *Config, logger logging.Logger) (*resolved, error) {
	out := &resolved{
		arms:   map[string]arm.Arm{},
		logger: logger,
	}
	for _, name := range cfg.Arms {
		dep, err := findDepByShortName(deps, name)
		if err != nil {
			return nil, fmt.Errorf("arm %q: %w", name, err)
		}
		a, ok := dep.(arm.Arm)
		if !ok {
			return nil, fmt.Errorf("arm %q: dependency is %T, not arm.Arm", name, dep)
		}
		out.arms[name] = a
		out.armOrder = append(out.armOrder, name)
	}

	if cfg.MotionService != "" {
		dep, err := findDepByShortName(deps, cfg.MotionService)
		if err != nil {
			return nil, fmt.Errorf("motion_service %q: %w", cfg.MotionService, err)
		}
		ms, ok := dep.(motion.Service)
		if !ok {
			return nil, fmt.Errorf("motion_service %q: dependency is %T, not motion.Service", cfg.MotionService, dep)
		}
		out.motion = ms
	}

	// The framesystem service is auto-injected for modules under the
	// public name "$framesystem". It's the entry point to obtaining a
	// *referenceframe.FrameSystem we can pass to armplanning.PlanMotion.
	fsSvc, err := framesystem.FromDependencies(deps)
	if err != nil {
		return nil, fmt.Errorf("framesystem service unavailable: %w", err)
	}
	out.frameSystem = fsSvc

	return out, nil
}

// findDepByShortName tolerates configs that list arms by short name (e.g.
// "arm_nw") rather than the fully-qualified resource name. resource.Dependencies
// is keyed by resource.Name, which has Namespace/API/Name components — direct
// short-name lookups don't work.
func findDepByShortName(deps resource.Dependencies, shortName string) (resource.Resource, error) {
	for name, dep := range deps {
		if name.ShortName() == shortName || name.Name == shortName {
			return dep, nil
		}
	}
	return nil, fmt.Errorf("dependency not found")
}

// buildFrameSystem obtains the machine's current frame system. Returns a
// fresh snapshot — the frame system service computes this on demand and
// callers should not cache the result across plan calls if any frame
// configurations may have changed.
func buildFrameSystem(ctx context.Context, r *resolved) (*referenceframe.FrameSystem, error) {
	if r == nil || r.frameSystem == nil {
		return nil, fmt.Errorf("frame system service is not resolved")
	}
	return framesystem.NewFromService(ctx, r.frameSystem, nil)
}
