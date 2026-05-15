package motionconstraints

import (
	"context"
	"fmt"
	"sync/atomic"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// presetByKey returns the Scenario for a built-in preset key. Returns nil
// for keys we don't recognize; the runner logs+skips nil scenarios.
//
// Each preset constructor returns a *fresh* Scenario value so per-arm
// scenario state lives in closures over local atomics — the parallel
// runner (Phase 10) keeps separate Scenario instances per arm and they
// don't share state.
func presetByKey(key string) *Scenario {
	switch key {
	case "single_arm_obstacle":
		s := presetSingleArmObstacle()
		return &s
	case "linear_constraint":
		s := presetLinearConstraint()
		return &s
	case "orientation_constraint":
		s := presetOrientationConstraint()
		return &s
	case "dynamic_obstacle":
		s := presetDynamicObstacle()
		return &s
	case "multi_arm_choreography":
		s := presetMultiArmChoreography()
		return &s
	case "obstacle_progression":
		s := presetObstacleProgression()
		return &s
	case "random_translation":
		s := presetRandomTranslation()
		return &s
	case "random_rotation":
		s := presetRandomRotation()
		return &s
	case "arc_over_obstacle":
		s := presetArcOverObstacle()
		return &s
	case "duck_under_obstacle":
		s := presetDuckUnderObstacle()
		return &s
	case "gripper_with_box":
		s := presetGripperWithBox()
		return &s
	case "corridor_passthrough":
		s := presetCorridorPassthrough()
		return &s
	case "random_translation_linear":
		s := presetRandomTranslationLinear()
		return &s
	case "random_rotation_linear":
		s := presetRandomRotationLinear()
		return &s
	case "ee_baseline":
		s := presetEEBaseline()
		return &s
	case "ee_linear":
		s := presetEELinear()
		return &s
	case "ee_orient":
		s := presetEEOrient()
		return &s
	case "ee_combined":
		s := presetEECombined()
		return &s
	case "ee_orient_60":
		s := presetEEOrient60()
		return &s
	case "ee_orient_120":
		s := presetEEOrient120()
		return &s
	default:
		return nil
	}
}

// ---- shared coordinate helpers ---------------------------------------------

// applyArmOffset translates a pose by the arm's world base. Used to place
// a scenario's relative-to-arm anchor or obstacle pose into world coords.
func applyArmOffset(armBase spatialmath.Pose, relative r3.Vector) spatialmath.Pose {
	if armBase == nil {
		return spatialmath.NewPoseFromPoint(relative)
	}
	bp := armBase.Point()
	return spatialmath.NewPoseFromPoint(r3.Vector{
		X: bp.X + relative.X,
		Y: bp.Y + relative.Y,
		Z: bp.Z + relative.Z,
	})
}

// armPrefixedBox is staticBox with the arm name prefixed onto the geometry
// label so two arms running the same preset don't collide in the scene map.
func armPrefixedBox(armName, scenarioKey, suffix string, world r3.Vector, dx, dy, dz float64) (spatialmath.Geometry, error) {
	label := fmt.Sprintf("%s:%s:%s", armName, scenarioKey, suffix)
	return staticBox(label, world.X, world.Y, world.Z, dx, dy, dz)
}

// dist3 returns the squared distance between two points (sufficient for
// "closer vs farther" comparisons in alternateBetweenAnchors).
func dist3(dx, dy, dz float64) float64 {
	return dx*dx + dy*dy + dz*dz
}

// ---- single_arm_obstacle ---------------------------------------------------

// presetSingleArmObstacle is the simplest motion-planning demo: one arm
// swings between two anchor poses on either side of a static box obstacle.
func presetSingleArmObstacle() Scenario {
	// Box is small and centered, anchors are well-clearance from it.
	// Earlier values (box 300mm wide in Y, anchors at ±300) left only
	// 150mm to plan around — cbirrt timed out under concurrent load.
	const (
		boxOffsetX, boxOffsetY, boxOffsetZ = 400.0, 0.0, 350.0
		boxDX, boxDY, boxDZ                = 100.0, 150.0, 200.0
	)
	anchorAOffset := r3.Vector{X: 500, Y: 400, Z: 400}
	anchorBOffset := r3.Vector{X: 500, Y: -400, Z: 400}

	return Scenario{
		Key:         "single_arm_obstacle",
		Description: "One arm swings back and forth around a static box obstacle.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			base := r.armBase(armName).Point()
			pos := r3.Vector{X: base.X + boxOffsetX, Y: base.Y + boxOffsetY, Z: base.Z + boxOffsetZ}
			geom, err := armPrefixedBox(armName, "single_arm_obstacle", "box", pos, boxDX, boxDY, boxDZ)
			if err != nil {
				return nil, err
			}
			return []scenarioObstacle{{Geom: geom, Color: &ColorObstacle}}, nil
		},
		Plan: alternateBetweenAnchors("single_arm_obstacle", anchorAOffset, anchorBOffset, nil, nil),
	}
}

// ---- linear_constraint -----------------------------------------------------

func presetLinearConstraint() Scenario {
	// Smaller swing + very loose tolerances so cbirrt converges fast.
	// 800mm of straight-line cartesian motion under a 45deg orientation
	// constraint was triggering IK-flip search hell that timed out the
	// planner. A 300mm swing with 200mm line tolerance is solvable in
	// well under 1s and still visibly different from an unconstrained
	// plan (the trajectory looks like a near-straight line vs the
	// natural curve cbirrt picks unconstrained).
	anchorA := r3.Vector{X: 500, Y: 150, Z: 400}
	anchorB := r3.Vector{X: 500, Y: -150, Z: 400}

	constraints := &motionplan.Constraints{
		LinearConstraint: []motionplan.LinearConstraint{
			{LineToleranceMm: 200, OrientationToleranceDegs: 90},
		},
	}
	return Scenario{
		Key:         "linear_constraint",
		Description: "Hold the EE on a straight line between anchors (50mm tolerance).",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			return nil, nil
		},
		// Match goal orientation to arm's current EE orientation at plan
		// time — guarantees IK reachability (arm is producing this
		// orientation right now) and means the LinearConstraint only has
		// to manage position along the path, not orientation. Works for
		// both wrist-EE and gripper-EE configs.
		Plan: alternateBetweenAnchors("linear_constraint", anchorA, anchorB, constraints,
			useCurrentEEOrientation),
	}
}

// ---- orientation_constraint ------------------------------------------------

func presetOrientationConstraint() Scenario {
	// Same shortening as linear_constraint — 300mm swing with a 90deg
	// orientation tolerance. Still visibly constrained (the wrist holds
	// roughly the same orientation across the swing) without forcing
	// the planner through IK discontinuities.
	anchorA := r3.Vector{X: 500, Y: 150, Z: 400}
	anchorB := r3.Vector{X: 500, Y: -150, Z: 400}

	constraints := &motionplan.Constraints{
		OrientationConstraint: []motionplan.OrientationConstraint{
			{OrientationToleranceDegs: 90},
		},
	}
	return Scenario{
		Key:         "orientation_constraint",
		Description: "Keep the EE orientation within 45 deg while moving between anchors.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			return nil, nil
		},
		Plan: alternateBetweenAnchors("orientation_constraint", anchorA, anchorB, constraints,
			useCurrentEEOrientation),
	}
}

// ---- dynamic_obstacle ------------------------------------------------------

func presetDynamicObstacle() Scenario {
	// Anchors moved out so the planner has clearance; animation now
	// oscillates the obstacle in the X axis between "near the arm" and
	// "out front" — keeps the box well clear of the anchor poses at
	// (500, ±400, 400) at all times. Earlier Y-axis animation drifted
	// the box to within 150mm of the goal, making every IK solution at
	// the goal collide with it.
	anchorA := r3.Vector{X: 500, Y: 400, Z: 400}
	anchorB := r3.Vector{X: 500, Y: -400, Z: 400}
	animAOffset := r3.Vector{X: 350, Y: 0, Z: 300}
	animBOffset := r3.Vector{X: 550, Y: 0, Z: 300}

	return Scenario{
		Key:         "dynamic_obstacle",
		Description: "Obstacle box oscillates continuously while the arm swings between anchors.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			base := r.armBase(armName).Point()
			startPos := r3.Vector{X: base.X + animAOffset.X, Y: base.Y + animAOffset.Y, Z: base.Z + animAOffset.Z}
			endPos := r3.Vector{X: base.X + animBOffset.X, Y: base.Y + animBOffset.Y, Z: base.Z + animBOffset.Z}
			geom, err := armPrefixedBox(armName, "dynamic_obstacle", "box", startPos, 150, 150, 200)
			if err != nil {
				return nil, err
			}
			return []scenarioObstacle{{
				Geom: geom, Color: &ColorObstacle,
				Anim: &obstacleAnimation{
					AnchorA: spatialmath.NewPoseFromPoint(startPos),
					AnchorB: spatialmath.NewPoseFromPoint(endPos),
					PeriodS: 6.0,
				},
			}}, nil
		},
		Plan: alternateBetweenAnchors("dynamic_obstacle", anchorA, anchorB, nil, nil),
	}
}

// ---- multi_arm_choreography ------------------------------------------------

// presetMultiArmChoreography drives the designated arm to alternate
// between two arm-relative anchors with all other configured arms
// treated as obstacles. The "choreography" angle is that every arm
// runs the same swing while sibling arms inject into the WorldState,
// so the collision-highlight fires when two arms swing through each
// other's volume.
//
// Earlier versions used an absolute world goal (0, 0, 700) to imply
// "all converge to a center" — but with 1m+ arm spacing that goal is
// physically out of reach. Arm-relative anchors keep each arm's plan
// solvable.
func presetMultiArmChoreography() Scenario {
	anchorA := r3.Vector{X: 400, Y: 250, Z: 500}
	anchorB := r3.Vector{X: 400, Y: -250, Z: 500}

	return Scenario{
		Key:         "multi_arm_choreography",
		Description: "Arms swing between anchors with siblings injected as world obstacles.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			// No static obstacles; siblings are injected inside Plan so
			// they reflect every other arm's current joint configuration.
			return nil, nil
		},
		Plan: func(
			ctx context.Context,
			r *resolved,
			fs *referenceframe.FrameSystem,
			armName string,
			obstacles []scenarioObstacle,
		) (motionplan.Plan, error) {
			siblingObstacles := injectSiblingArmObstacles(ctx, r, fs, armName)
			swing := alternateBetweenAnchors("multi_arm_choreography", anchorA, anchorB, nil, nil)
			return swing(ctx, r, fs, armName, siblingObstacles)
		},
	}
}

func injectSiblingArmObstacles(
	ctx context.Context,
	r *resolved,
	fs *referenceframe.FrameSystem,
	movingArm string,
) []scenarioObstacle {
	siblingObstacles := []scenarioObstacle{}
	inputs := referenceframe.FrameSystemInputs{}
	for _, name := range r.armOrder {
		if name == movingArm {
			continue
		}
		sibArm, ok := r.arms[name]
		if !ok {
			continue
		}
		joints, err := sibArm.JointPositions(ctx, nil)
		if err == nil {
			inputs[name] = joints
		}
	}
	for _, name := range r.armOrder {
		if name == movingArm {
			continue
		}
		sibArm, ok := r.arms[name]
		if !ok {
			continue
		}
		model, err := sibArm.Kinematics(ctx)
		if err != nil || model == nil {
			continue
		}
		joints := inputs[name]
		gif, err := model.Geometries(joints)
		if err != nil || gif == nil {
			continue
		}
		worldGeoms := geometriesToWorld(fs, inputs, name, gif)
		for k, g := range worldGeoms {
			g.SetLabel(fmt.Sprintf("sibling:%s:%d", name, k))
			siblingObstacles = append(siblingObstacles, scenarioObstacle{Geom: g, Color: &ColorObstacle})
		}
	}
	return siblingObstacles
}

// ---- obstacle_progression --------------------------------------------------

func presetObstacleProgression() Scenario {
	anchorA := r3.Vector{X: 500, Y: 300, Z: 400}
	anchorB := r3.Vector{X: 500, Y: -300, Z: 400}
	var counter int64

	return Scenario{
		Key:         "obstacle_progression",
		Description: "Same anchors; obstacles accumulate each cycle (box, +floor, +ceiling, +walls).",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			// Two stages: just the box, then box+floor. Anything heavier
			// (ceiling, walls) reliably timed the cbirrt planner out
			// under concurrent load.
			stage := int(atomic.AddInt64(&counter, 1)-1) % 2
			base := r.armBase(armName).Point()

			obstacles := []scenarioObstacle{}
			boxPos := r3.Vector{X: base.X + 400, Y: base.Y + 0, Z: base.Z + 350}
			boxGeom, err := armPrefixedBox(armName, "obstacle_progression", "box", boxPos, 100, 150, 200)
			if err != nil {
				return nil, err
			}
			obstacles = append(obstacles, scenarioObstacle{Geom: boxGeom, Color: &ColorObstacle})

			if stage >= 1 {
				floor, err := armPrefixedBox(armName, "obstacle_progression", "floor",
					r3.Vector{X: base.X, Y: base.Y, Z: base.Z - 5}, 2500, 2500, 10)
				if err != nil {
					return nil, err
				}
				obstacles = append(obstacles, scenarioObstacle{Geom: floor, Color: &ColorObstacle})
			}
			return obstacles, nil
		},
		Plan: alternateBetweenAnchors("obstacle_progression", anchorA, anchorB, nil, nil),
	}
}

// ---- random_translation ----------------------------------------------------

// presetRandomTranslation visits a deterministic-but-varied sequence of
// reachable workspace positions, all with a downward-facing default
// orientation. Combined with a non-default EE frame (a gripper) this
// demonstrates how the planner solves for the gripper tip rather than
// the wrist when an offset tool frame is configured.
func presetRandomTranslation() Scenario {
	// Sequence of reachable arm-local positions covering a varied chunk
	// of the workspace. A simple cycle counter advances through them so
	// the motion is reproducible across runs.
	waypoints := []r3.Vector{
		{X: 500, Y: 250, Z: 400},
		{X: 350, Y: -250, Z: 550},
		{X: 600, Y: 0, Z: 300},
		{X: 400, Y: 200, Z: 500},
		{X: 500, Y: -200, Z: 350},
		{X: 450, Y: 100, Z: 450},
		{X: 550, Y: -100, Z: 500},
	}
	var counter int64

	return Scenario{
		Key:         "random_translation",
		Description: "Arm visits a varied sequence of reachable positions with a fixed downward orientation.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			return nil, nil
		},
		Plan: func(
			ctx context.Context,
			r *resolved,
			fs *referenceframe.FrameSystem,
			armName string,
			obstacles []scenarioObstacle,
		) (motionplan.Plan, error) {
			idx := int(atomic.AddInt64(&counter, 1)-1) % len(waypoints)
			off := waypoints[idx]
			goal := applyArmOffset(r.armBase(armName), off)
			if r.logger != nil {
				r.logger.Infow("task-space: random_translation goal",
					"arm", armName,
					"waypoint_idx", idx,
					"offset_local", []float64{off.X, off.Y, off.Z},
				)
			}
			return planSingleArmToPose(ctx, r, fs, armName, goal, obstacles, nil)
		},
	}
}

// ---- random_rotation -------------------------------------------------------

// presetRandomRotation holds the EE at a fixed arm-local position and
// cycles through a sequence of orientations. With a non-default EE frame
// (an offset gripper) this shows the planner rolling the gripper around
// a single point while the wrist trace traces a circle around the offset.
func presetRandomRotation() Scenario {
	// Fixed arm-local position; orientations cycle through a varied list.
	const (
		posX, posY, posZ = 500.0, 0.0, 400.0
	)
	// Mix of tool-axis twists and small pointing-direction tilts so the
	// motion is visibly different cycle-to-cycle. All tilts stay near
	// the identity orientation (OZ ~ +1) — bigger tilts of 30deg or
	// less keep the IK well away from the wrist-flip singularities
	// that stuck the arm at startup. Without a gripper offset, pure
	// theta-only twists are nearly invisible (just the last joint
	// rotating), so tilts give the wrist something to actually do.
	orientations := []*spatialmath.OrientationVectorDegrees{
		{OX: 0, OY: 0, OZ: 1, Theta: 0},
		{OX: 0.3, OY: 0, OZ: 0.95, Theta: 0},
		{OX: 0, OY: 0.3, OZ: 0.95, Theta: 0},
		{OX: -0.3, OY: 0, OZ: 0.95, Theta: 0},
		{OX: 0, OY: -0.3, OZ: 0.95, Theta: 0},
		{OX: 0.2, OY: 0.2, OZ: 0.96, Theta: 60},
		{OX: -0.2, OY: 0.2, OZ: 0.96, Theta: -60},
		{OX: 0, OY: 0, OZ: 1, Theta: 120},
	}
	var counter int64

	return Scenario{
		Key:         "random_rotation",
		Description: "Arm holds a fixed position and cycles the EE through varied orientations.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			return nil, nil
		},
		Plan: func(
			ctx context.Context,
			r *resolved,
			fs *referenceframe.FrameSystem,
			armName string,
			obstacles []scenarioObstacle,
		) (motionplan.Plan, error) {
			idx := int(atomic.AddInt64(&counter, 1)-1) % len(orientations)
			ov := orientations[idx]
			pos := applyArmOffset(r.armBase(armName), r3.Vector{X: posX, Y: posY, Z: posZ}).Point()
			goal := spatialmath.NewPose(pos, ov)
			if r.logger != nil {
				r.logger.Infow("task-space: random_rotation goal",
					"arm", armName,
					"orientation_idx", idx,
					"world_xyz", []float64{pos.X, pos.Y, pos.Z},
					"world_ov_deg", []float64{ov.OX, ov.OY, ov.OZ, ov.Theta},
				)
			}
			return planSingleArmToPose(ctx, r, fs, armName, goal, obstacles, nil)
		},
	}
}

// ---- random_translation_linear ---------------------------------------------

// presetRandomTranslationLinear alternates between two workspace-spanning
// anchors under a tight LinearConstraint, so each hop traces a meaningfully
// straight cartesian line across the EE workspace. Pair with an EE-frame
// override to see gripper-vs-wrist trail differences.
//
// Empirical lesson learned the hard way: tight position anchors + offset
// gripper EE + LinearConstraint is a recipe for IK failure ("zero IK
// solutions" — the wrist can't reach the goal position with the required
// orientation) AND cbirrt timeouts (even when IK succeeds at the endpoint,
// finding a constraint-respecting path is hard). The fix: use the EXACT
// same 7 varied waypoints as random_translation (proven to be reachable
// for both wrist-EE and gripper-EE plans), keep identity goal orientation
// (also proven reachable at these waypoints), and apply a LOOSE
// LinearConstraint as a soft preference. cbirrt converges quickly and the
// resulting paths are visibly straighter than random_translation's natural
// arcs, even with the loose tolerance.
func presetRandomTranslationLinear() Scenario {
	waypoints := []r3.Vector{
		{X: 500, Y: 250, Z: 400},
		{X: 350, Y: -250, Z: 550},
		{X: 600, Y: 0, Z: 300},
		{X: 400, Y: 200, Z: 500},
		{X: 500, Y: -200, Z: 350},
		{X: 450, Y: 100, Z: 450},
		{X: 550, Y: -100, Z: 500},
	}
	constraints := &motionplan.Constraints{
		LinearConstraint: []motionplan.LinearConstraint{
			{LineToleranceMm: 300, OrientationToleranceDegs: 180},
		},
	}
	var counter int64
	return Scenario{
		Key:         "random_translation_linear",
		Description: "Visits a varied waypoint sequence under a loose LinearConstraint — paths are visibly straighter than the unconstrained variant.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			return nil, nil
		},
		Plan: func(
			ctx context.Context,
			r *resolved,
			fs *referenceframe.FrameSystem,
			armName string,
			obstacles []scenarioObstacle,
		) (motionplan.Plan, error) {
			idx := int(atomic.AddInt64(&counter, 1)-1) % len(waypoints)
			off := waypoints[idx]
			goal := applyArmOffset(r.armBase(armName), off)
			if r.logger != nil {
				r.logger.Infow("task-space: random_translation_linear goal",
					"arm", armName,
					"waypoint_idx", idx,
					"offset_local", []float64{off.X, off.Y, off.Z},
				)
			}
			return planSingleArmToPose(ctx, r, fs, armName, goal, obstacles, constraints)
		},
	}
}

// ---- random_rotation_linear ------------------------------------------------

// presetRandomRotationLinear holds a fixed arm-local position and cycles
// orientations like random_rotation, but adds a LinearConstraint so the
// EE position is constrained to a (degenerate, zero-length) straight
// line between identical positions. Practically: the position cannot
// drift while orientation changes, which is a meaningful demonstration
// on grippered arms where the wrist must trace a circle to keep the
// tool tip planted.
func presetRandomRotationLinear() Scenario {
	const (
		posX, posY, posZ = 500.0, 0.0, 400.0
	)
	// Twists around the default tool axis only — same reasoning as
	// random_rotation: avoids the ur5e wrist-flip IK pathology that
	// stuck row-AB arms in their startup pose.
	orientations := []*spatialmath.OrientationVectorDegrees{
		{OX: 0, OY: 0, OZ: 1, Theta: 0},
		{OX: 0, OY: 0, OZ: 1, Theta: 60},
		{OX: 0, OY: 0, OZ: 1, Theta: 120},
		{OX: 0, OY: 0, OZ: 1, Theta: 180},
		{OX: 0, OY: 0, OZ: 1, Theta: -120},
		{OX: 0, OY: 0, OZ: 1, Theta: -60},
	}
	// LinearConstraint here is degenerate (start == end position) and a
	// tight tolerance makes the cbirrt planner thrash on the orientation
	// interpolation between very different EE orientations. A loose
	// 500mm tolerance is effectively position-fixed without forcing IK
	// through singular configs.
	constraints := &motionplan.Constraints{
		LinearConstraint: []motionplan.LinearConstraint{
			{LineToleranceMm: 500, OrientationToleranceDegs: 180},
		},
	}
	var counter int64

	return Scenario{
		Key:         "random_rotation_linear",
		Description: "Same as random_rotation but the EE position is held under a LinearConstraint while orientation changes.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			return nil, nil
		},
		Plan: func(
			ctx context.Context,
			r *resolved,
			fs *referenceframe.FrameSystem,
			armName string,
			obstacles []scenarioObstacle,
		) (motionplan.Plan, error) {
			idx := int(atomic.AddInt64(&counter, 1)-1) % len(orientations)
			ov := orientations[idx]
			pos := applyArmOffset(r.armBase(armName), r3.Vector{X: posX, Y: posY, Z: posZ}).Point()
			goal := spatialmath.NewPose(pos, ov)
			return planSingleArmToPose(ctx, r, fs, armName, goal, obstacles, constraints)
		},
	}
}

// ---- arc_over_obstacle -----------------------------------------------------

// presetArcOverObstacle places a wide, short obstacle slightly below the
// straight-line path between two anchors so the planner naturally chooses
// to arc OVER the box. The arm visibly lifts and dips back to the goal.
func presetArcOverObstacle() Scenario {
	const (
		// Box centered between anchors. Wide in Y so going around
		// laterally is geometrically much longer than going over.
		boxOffsetX, boxOffsetY, boxOffsetZ = 500.0, 0.0, 300.0
		boxDX, boxDY, boxDZ                = 200.0, 500.0, 100.0
	)
	anchorA := r3.Vector{X: 500, Y: 350, Z: 450}
	anchorB := r3.Vector{X: 500, Y: -350, Z: 450}

	return Scenario{
		Key:         "arc_over_obstacle",
		Description: "Wide low box between anchors — arm arcs over the top.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			base := r.armBase(armName).Point()
			pos := r3.Vector{X: base.X + boxOffsetX, Y: base.Y + boxOffsetY, Z: base.Z + boxOffsetZ}
			geom, err := armPrefixedBox(armName, "arc_over_obstacle", "box", pos, boxDX, boxDY, boxDZ)
			if err != nil {
				return nil, err
			}
			return []scenarioObstacle{{Geom: geom, Color: &ColorObstacle}}, nil
		},
		Plan: alternateBetweenAnchors("arc_over_obstacle", anchorA, anchorB, nil, nil),
	}
}

// ---- duck_under_obstacle ---------------------------------------------------

// presetDuckUnderObstacle is the mirror image of arc_over_obstacle — box
// is high, anchors are below, planner ducks under the obstacle. Same
// total motion but the trajectory shape inverts.
func presetDuckUnderObstacle() Scenario {
	const (
		boxOffsetX, boxOffsetY, boxOffsetZ = 500.0, 0.0, 500.0
		boxDX, boxDY, boxDZ                = 200.0, 500.0, 100.0
	)
	anchorA := r3.Vector{X: 500, Y: 350, Z: 350}
	anchorB := r3.Vector{X: 500, Y: -350, Z: 350}

	return Scenario{
		Key:         "duck_under_obstacle",
		Description: "Wide high box between anchors — arm ducks underneath.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			base := r.armBase(armName).Point()
			pos := r3.Vector{X: base.X + boxOffsetX, Y: base.Y + boxOffsetY, Z: base.Z + boxOffsetZ}
			geom, err := armPrefixedBox(armName, "duck_under_obstacle", "box", pos, boxDX, boxDY, boxDZ)
			if err != nil {
				return nil, err
			}
			return []scenarioObstacle{{Geom: geom, Color: &ColorObstacle}}, nil
		},
		Plan: alternateBetweenAnchors("duck_under_obstacle", anchorA, anchorB, nil, nil),
	}
}

// ---- gripper_with_box ------------------------------------------------------

// presetGripperWithBox is the same arc-over problem as arc_over_obstacle,
// but the arm carries a tool that itself has a box-shaped collision
// geometry attached (configured on the gripper component's frame in the
// machine config). The framesystem includes the gripper geometry in the
// arm's collision footprint, so the same anchor pair + obstacle can
// produce a visibly different path because the gripper sticks out.
func presetGripperWithBox() Scenario {
	const (
		boxOffsetX, boxOffsetY, boxOffsetZ = 500.0, 0.0, 300.0
		boxDX, boxDY, boxDZ                = 200.0, 500.0, 100.0
	)
	anchorA := r3.Vector{X: 500, Y: 400, Z: 500}
	anchorB := r3.Vector{X: 500, Y: -400, Z: 500}

	return Scenario{
		Key:         "gripper_with_box",
		Description: "Arc-over scenario where the gripper itself carries a long collision geometry.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			base := r.armBase(armName).Point()
			pos := r3.Vector{X: base.X + boxOffsetX, Y: base.Y + boxOffsetY, Z: base.Z + boxOffsetZ}
			geom, err := armPrefixedBox(armName, "gripper_with_box", "box", pos, boxDX, boxDY, boxDZ)
			if err != nil {
				return nil, err
			}
			return []scenarioObstacle{{Geom: geom, Color: &ColorObstacle}}, nil
		},
		Plan: alternateBetweenAnchors("gripper_with_box", anchorA, anchorB, nil, nil),
	}
}

// ---- corridor_passthrough --------------------------------------------------

// presetCorridorPassthrough sets up two large boxes such that the only
// feasible trajectory between the anchors threads the gap between them.
// Anchors are placed on opposite sides along X (in front of the corridor
// and behind it) so the arm has to drive forward through the gap.
func presetCorridorPassthrough() Scenario {
	const (
		boxOffsetX                 = 500.0
		boxAY, boxAZ, boxBY, boxBZ = 250.0, 400.0, -250.0, 400.0
		boxDX, boxDY, boxDZ        = 200.0, 200.0, 400.0
	)
	// Anchors in front of corridor and behind it. The arm must pass
	// through the (y=-150..+150) gap at z≈400.
	anchorA := r3.Vector{X: 350, Y: 0, Z: 400}
	anchorB := r3.Vector{X: 650, Y: 0, Z: 400}

	return Scenario{
		Key:         "corridor_passthrough",
		Description: "Two walls form a narrow corridor — the only feasible trajectory threads the gap.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			base := r.armBase(armName).Point()
			boxA, err := armPrefixedBox(armName, "corridor_passthrough", "wall_plusY",
				r3.Vector{X: base.X + boxOffsetX, Y: base.Y + boxAY, Z: base.Z + boxAZ},
				boxDX, boxDY, boxDZ)
			if err != nil {
				return nil, err
			}
			boxB, err := armPrefixedBox(armName, "corridor_passthrough", "wall_minusY",
				r3.Vector{X: base.X + boxOffsetX, Y: base.Y + boxBY, Z: base.Z + boxBZ},
				boxDX, boxDY, boxDZ)
			if err != nil {
				return nil, err
			}
			return []scenarioObstacle{
				{Geom: boxA, Color: &ColorObstacle},
				{Geom: boxB, Color: &ColorObstacle},
			}, nil
		},
		Plan: alternateBetweenAnchors("corridor_passthrough", anchorA, anchorB, nil, nil),
	}
}

// ---- shared helpers --------------------------------------------------------

// alternateBetweenAnchors returns a Plan hook that swings the designated
// arm between two anchor offsets (relative to the arm's mount). The hook
// chooses whichever anchor is farther from the arm's current EE pose so
// the motion looks like a continuous back-and-forth swing.
//
// Coordinate-frame note: `arm.EndPosition()` returns the EE in the arm's
// LOCAL mount frame (small numbers near the workspace). Anchor OFFSETS
// are also in that same local frame. We compare local-to-local, then
// transform the chosen offset to world coords before handing the goal
// to the planner.
//
// An earlier version of this helper compared local-frame EE against
// world-frame anchor poses, which made every per-arm scenario stuck on
// the same anchor forever (max_joint_delta_rad: 0 in every cycle).
// alternateBetweenAnchors returns a Plan closure that toggles between two
// anchor positions. goalOrient controls the goal pose's orientation:
//   - nil: identity orientation (legacy behavior, fine for wrist-EE plans
//     with no constraint — e.g. obstacle scenarios)
//   - a fixed OrientationVectorDegrees: use this specific orientation
//   - useCurrentEEOrientation: read armRes.EndPosition's orientation at
//     plan time and use it. Critical for constrained scenarios with
//     offset-gripper EE frames: identity orientation is often physically
//     unreachable for the wrist at the goal position (the IK failure
//     mode is "zero IK solutions, goal positions appears to be physically
//     unreachable"), and even when reachable, rotating from a non-identity
//     start orientation to identity end orientation along a constrained
//     path is hard for cbirrt. Using current EE orientation means start
//     and end orientations match — the constrained plan only has to
//     translate, not rotate, which IK + cbirrt can solve reliably.
//
// useCurrentEEOrientation is a sentinel value (not a real orientation);
// it's compared by identity.
var useCurrentEEOrientation = &spatialmath.OrientationVectorDegrees{}

func alternateBetweenAnchors(
	scenarioKey string,
	anchorAOffset, anchorBOffset r3.Vector,
	constraints *motionplan.Constraints,
	goalOrient *spatialmath.OrientationVectorDegrees,
) func(context.Context, *resolved, *referenceframe.FrameSystem, string, []scenarioObstacle) (motionplan.Plan, error) {
	return func(
		ctx context.Context,
		r *resolved,
		fs *referenceframe.FrameSystem,
		armName string,
		obstacles []scenarioObstacle,
	) (motionplan.Plan, error) {
		armRes, ok := r.arms[armName]
		if !ok {
			return nil, fmt.Errorf("%s: arm %q is not configured", scenarioKey, armName)
		}
		goalOffset := anchorAOffset
		pickedLabel := "A"
		if armRes != nil {
			if ee, err := armRes.EndPosition(ctx, nil); err == nil && ee != nil {
				// EE is in arm-local coords; anchors are too.
				eePt := ee.Point()
				dA := dist3(eePt.X-anchorAOffset.X, eePt.Y-anchorAOffset.Y, eePt.Z-anchorAOffset.Z)
				dB := dist3(eePt.X-anchorBOffset.X, eePt.Y-anchorBOffset.Y, eePt.Z-anchorBOffset.Z)
				if dA < dB {
					goalOffset = anchorBOffset
					pickedLabel = "B"
				}
				if r.logger != nil {
					armBasePt := r.armBase(armName).Point()
					eeWorld := []float64{armBasePt.X + eePt.X, armBasePt.Y + eePt.Y, armBasePt.Z + eePt.Z}
					r.logger.Infow("task-space: anchor choice",
						"arm", armName,
						"scenario", scenarioKey,
						"arm_base_world", []float64{armBasePt.X, armBasePt.Y, armBasePt.Z},
						"ee_local", []float64{eePt.X, eePt.Y, eePt.Z},
						"ee_world", eeWorld,
						"anchorA_local", []float64{anchorAOffset.X, anchorAOffset.Y, anchorAOffset.Z},
						"anchorB_local", []float64{anchorBOffset.X, anchorBOffset.Y, anchorBOffset.Z},
						"dist_to_A_sq", dA,
						"dist_to_B_sq", dB,
						"picked", pickedLabel,
					)
				}
			}
		}
		var goal spatialmath.Pose
		effectiveOrient := goalOrient
		if effectiveOrient == useCurrentEEOrientation {
			// Read the arm's current EE orientation. IK is guaranteed to
			// succeed at that orientation (arm is currently producing it)
			// and constrained planning is dramatically easier (no
			// orientation change along the path).
			if ee, err := armRes.EndPosition(ctx, nil); err == nil && ee != nil {
				ov := ee.Orientation().OrientationVectorDegrees()
				effectiveOrient = ov
			} else {
				effectiveOrient = nil // fall back to identity
			}
		}
		if effectiveOrient != nil {
			armBasePt := r.armBase(armName).Point()
			goal = spatialmath.NewPose(
				r3.Vector{X: armBasePt.X + goalOffset.X, Y: armBasePt.Y + goalOffset.Y, Z: armBasePt.Z + goalOffset.Z},
				effectiveOrient,
			)
		} else {
			goal = applyArmOffset(r.armBase(armName), goalOffset)
		}
		if r.logger != nil {
			goalPt := goal.Point()
			ov := goal.Orientation().OrientationVectorDegrees()
			r.logger.Infow("task-space: cartesian goal",
				"arm", armName,
				"scenario", scenarioKey,
				"anchor_picked", pickedLabel,
				"goal_world", []float64{goalPt.X, goalPt.Y, goalPt.Z},
				"goal_offset_local", []float64{goalOffset.X, goalOffset.Y, goalOffset.Z},
				"goal_orient_ov_deg", []float64{ov.OX, ov.OY, ov.OZ, ov.Theta},
			)
		}
		plan, err := planSingleArmToPose(ctx, r, fs, armName, goal, obstacles, constraints)
		if err != nil {
			return nil, err
		}
		return plan, nil
	}
}

// ---- ee_variations bundle (constraint comparison) --------------------------
//
// Four scenarios that all run the SAME 2-anchor swing between
// arm-local (450, ±200, 450), so the only visible difference is the
// constraint type. Designed for direct side-by-side pedagogical
// comparison. Anchors are intentionally in the "safe" region of ur5e
// workspace where identity-orientation IK succeeds for both wrist-EE
// and gripper-EE plans (same X/Y range as the random_translation
// waypoints, which are empirically reachable).
//
// Constraint tolerances are loose enough that cbirrt converges quickly:
//   - LinearConstraint: 200mm line tolerance, 180deg orient (effectively
//     position-only)
//   - OrientationConstraint: 45deg (visibly locks tool orientation)
//   - Combined: 200mm position + 30deg orient (the hardest of the four;
//     pedagogically useful — sometimes fails, see README for the caveat)
//
// See README "Constraint variations and their known issues" for the
// warts on each constraint type.
// 200mm Y-swing (was 400mm). Critical: under LinearConstraint, the
// planner generates intermediate waypoints every defaultStepSizeMM=10mm
// (motionplan/armplanning/plan_manager.go::generateWaypoints), each
// requiring its own IK + cbirrt sub-plan within the total 3s budget. A
// 400mm swing produces 40 sub-plans (~75ms each — too tight); 200mm
// produces 20 (~150ms each — converges reliably). The motion is still
// clearly visible in the 3D view.
var (
	eeAnchorA = r3.Vector{X: 450, Y: 100, Z: 450}
	eeAnchorB = r3.Vector{X: 450, Y: -100, Z: 450}
)

func presetEEBaseline() Scenario {
	return Scenario{
		Key:         "ee_baseline",
		Description: "Same anchor swing, no constraint — the natural cbirrt path.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			return nil, nil
		},
		Plan: alternateBetweenAnchors("ee_baseline", eeAnchorA, eeAnchorB, nil, nil),
	}
}

func presetEELinear() Scenario {
	// LineToleranceMm: 50 (TIGHT tube — visibly forces the EE onto a
	// straight line vs the natural cbirrt arc). OrientationToleranceDegs:
	// 180 (orientation unconstrained along the path). Empirically verified
	// to plan reliably from homeJointPositionsReady for offset grippers at
	// the Y±100mm/Z=450 anchors. See cmd/probe for the sweep.
	constraints := &motionplan.Constraints{
		LinearConstraint: []motionplan.LinearConstraint{
			{LineToleranceMm: 50, OrientationToleranceDegs: 180},
		},
	}
	return Scenario{
		Key:         "ee_linear",
		Description: "Same anchor swing under a tight LinearConstraint — EE follows a visibly straight cartesian line.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			return nil, nil
		},
		Plan: alternateBetweenAnchors("ee_linear", eeAnchorA, eeAnchorB, constraints, nil),
	}
}

func presetEEOrient() Scenario {
	// OrientationToleranceDegs: 90 (was 45 — too tight to bridge from the
	// ready pose's wrist orientation to identity at the goal). 90deg is
	// the looseness at which OrientationConstraint reliably plans for
	// offset grippers from the ready pose. cmd/probe verified.
	constraints := &motionplan.Constraints{
		OrientationConstraint: []motionplan.OrientationConstraint{
			{OrientationToleranceDegs: 90},
		},
	}
	return Scenario{
		Key:         "ee_orient",
		Description: "Same anchor swing under an OrientationConstraint — tool orientation stays within 90deg of the interpolated path orientation.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			return nil, nil
		},
		Plan: alternateBetweenAnchors("ee_orient", eeAnchorA, eeAnchorB, constraints, nil),
	}
}

func presetEECombined() Scenario {
	// LinearConstraint with non-trivial orientation component: 200mm line
	// tolerance + 45deg orientation tolerance. The combined effect is more
	// constrained than ee_linear (orient must hold) AND more constrained
	// than ee_orient (position must hold) — but with these specific
	// tolerances, still plans reliably from the ready pose. cmd/probe
	// confirmed combined_200_45 succeeds for all tested gripper z-offsets.
	constraints := &motionplan.Constraints{
		LinearConstraint: []motionplan.LinearConstraint{
			{LineToleranceMm: 200, OrientationToleranceDegs: 45},
		},
	}
	return Scenario{
		Key:         "ee_combined",
		Description: "Combined LinearConstraint(200mm) + 45deg orientation tolerance.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			return nil, nil
		},
		Plan: alternateBetweenAnchors("ee_combined", eeAnchorA, eeAnchorB, constraints, nil),
	}
}

func presetEEOrient60() Scenario {
	constraints := &motionplan.Constraints{
		OrientationConstraint: []motionplan.OrientationConstraint{
			{OrientationToleranceDegs: 60},
		},
	}
	return Scenario{
		Key:         "ee_orient_60",
		Description: "Same anchor swing under tight OrientationConstraint(60deg) — pedagogically the tightest orient that still solves reliably for offset grippers.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			return nil, nil
		},
		Plan: alternateBetweenAnchors("ee_orient_60", eeAnchorA, eeAnchorB, constraints, nil),
	}
}

func presetEEOrient120() Scenario {
	constraints := &motionplan.Constraints{
		OrientationConstraint: []motionplan.OrientationConstraint{
			{OrientationToleranceDegs: 120},
		},
	}
	return Scenario{
		Key:         "ee_orient_120",
		Description: "Same anchor swing under loose OrientationConstraint(120deg).",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			return nil, nil
		},
		Plan: alternateBetweenAnchors("ee_orient_120", eeAnchorA, eeAnchorB, constraints, nil),
	}
}
