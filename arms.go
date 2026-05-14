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
	"go.viam.com/rdk/spatialmath"
)

// resolved is the runtime view of the dependencies the planner service needs:
// the configured arms, the motion service (kept for callers who want
// service.Move semantics), the framesystem service (load-bearing —
// armplanning.PlanMotion needs a *referenceframe.FrameSystem), and the
// service's logger so plan helpers can emit diagnostics under the same
// resource name as the service itself.
//
// armBases caches each arm's base pose in world coordinates so per-arm
// scenarios can place obstacles relative to their own arm rather than in
// absolute world coords. Populated in resolveDeps via framesystem.GetPose.
type resolved struct {
	arms        map[string]arm.Arm
	armOrder    []string // preserve config order for grid scenarios
	armBases    map[string]spatialmath.Pose
	eeFrames    map[string]string // arm -> non-default EE frame (e.g. gripper)
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
		arms:     map[string]arm.Arm{},
		armBases: map[string]spatialmath.Pose{},
		eeFrames: map[string]string{},
		logger:   logger,
	}
	for k, v := range cfg.EEFrames {
		out.eeFrames[k] = v
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

	if logger != nil {
		logger.Infow("resolveDeps: arms resolved",
			"arms", out.armOrder,
			"motion_service_configured", cfg.MotionService != "",
		)
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

// populateArmBases queries the framesystem for each configured arm's pose
// in world coordinates and caches it on the resolved struct. Per-arm
// scenarios use these offsets to place obstacles relative to the arm's
// base rather than at absolute world coords.
//
// Best-effort: arms whose pose can't be resolved (e.g. transient framesystem
// errors) silently fall back to a zero pose, which keeps single-arm-at-
// origin configurations behaving exactly as before.
func (r *resolved) populateArmBases(ctx context.Context) {
	if r == nil || r.frameSystem == nil {
		return
	}
	for _, name := range r.armOrder {
		// IMPORTANT: framesystem.GetPose(<armName>, "world") returns the
		// arm's END-EFFECTOR pose at zero config, not the mount pose —
		// the kinematic chain's primary output frame is named after the
		// component itself. The static offset frame that captures the
		// arm's mount in world is named <armName>_origin.
		originFrame := name + "_origin"
		pif, err := r.frameSystem.GetPose(ctx, originFrame, referenceframe.World, nil, nil)
		if err != nil || pif == nil {
			r.armBases[name] = spatialmath.NewZeroPose()
			if r.logger != nil {
				r.logger.Warnw("populateArmBases: GetPose failed, using zero",
					"arm", name, "origin_frame", originFrame, "err", err)
			}
			continue
		}
		r.armBases[name] = pif.Pose()
		if r.logger != nil {
			pt := pif.Pose().Point()
			r.logger.Infow("populateArmBases: arm mount in world",
				"arm", name,
				"origin_frame", originFrame,
				"xyz", []float64{pt.X, pt.Y, pt.Z},
			)
		}
	}
}

// armBase returns the cached world-frame pose of an arm, or a zero pose if
// the arm isn't in the cache.
func (r *resolved) armBase(armName string) spatialmath.Pose {
	if r == nil || r.armBases == nil {
		return spatialmath.NewZeroPose()
	}
	if p, ok := r.armBases[armName]; ok && p != nil {
		return p
	}
	return spatialmath.NewZeroPose()
}

// homeJointPositionsCandle returns a "tucked" joint-space pose:
// j1 = j3 = -90deg folds 6/7-DOF arms upward, keeping all links well
// away from the typical (500, 0, 300) obstacle region. Used for
// scenarios that put boxes near zero-config workspace.
func homeJointPositionsCandle(numDoF int) []referenceframe.Input {
	h := make([]referenceframe.Input, numDoF)
	if numDoF >= 2 {
		h[1] = -1.5708
	}
	if numDoF >= 4 {
		h[3] = -1.5708
	}
	return h
}

// homeJointPositionsReady returns a forward-pointing "ready" pose:
// j1 = -90deg (shoulder up), j2 = +90deg (elbow flex), j3 = -90deg
// (wrist flex). Puts the EE roughly out front of the arm at a mid-
// height position — inside the typical workspace targets — so cbirrt
// has continuous IK solutions across plans to (500, 0, 400)-style
// anchors.
//
// Used for non-obstacle scenarios where the previous "all zeros"
// default made the EE land 800mm BEHIND the arm at zero-config, and
// any linear-constrained plan to a forward goal had to trace a line
// passing through joint singularities near the base.
func homeJointPositionsReady(numDoF int) []referenceframe.Input {
	h := make([]referenceframe.Input, numDoF)
	if numDoF >= 2 {
		h[1] = -1.5708
	}
	if numDoF >= 3 {
		h[2] = 1.5708
	}
	if numDoF >= 4 {
		h[3] = -1.5708
	}
	return h
}

// eeFrame returns the frame name the planner should target for an arm —
// the configured non-default frame (e.g. a gripper) if set, otherwise the
// arm's own kinematic-output frame (i.e. the arm name itself).
func (r *resolved) eeFrame(armName string) string {
	if r == nil || r.eeFrames == nil {
		return armName
	}
	if name, ok := r.eeFrames[armName]; ok && name != "" {
		return name
	}
	return armName
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
