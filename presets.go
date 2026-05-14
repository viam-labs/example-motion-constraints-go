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
		Plan: alternateBetweenAnchors("single_arm_obstacle", anchorAOffset, anchorBOffset, nil),
	}
}

// ---- linear_constraint -----------------------------------------------------

func presetLinearConstraint() Scenario {
	// No obstacle in this scenario. The educational story is the shape
	// of the EE trajectory under a line constraint vs an unconstrained
	// plan — adding a box made the cbirrt planner consistently time out
	// at 15s+ under concurrent load with no extra pedagogical value.
	anchorA := r3.Vector{X: 500, Y: 400, Z: 400}
	anchorB := r3.Vector{X: 500, Y: -400, Z: 400}

	// Tolerances loose enough for cbirrt to solve quickly. Tighten in a
	// per-machine override (e.g. LineToleranceMm: 5) to see the planner
	// give up and trigger the timeout pedagogy.
	constraints := &motionplan.Constraints{
		LinearConstraint: []motionplan.LinearConstraint{
			{LineToleranceMm: 50, OrientationToleranceDegs: 45},
		},
	}
	return Scenario{
		Key:         "linear_constraint",
		Description: "Hold the EE on a straight line between anchors (50mm tolerance).",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			return nil, nil
		},
		Plan: alternateBetweenAnchors("linear_constraint", anchorA, anchorB, constraints),
	}
}

// ---- orientation_constraint ------------------------------------------------

func presetOrientationConstraint() Scenario {
	// No obstacle — same rationale as linear_constraint. The constraint
	// itself is the demo.
	anchorA := r3.Vector{X: 500, Y: 400, Z: 400}
	anchorB := r3.Vector{X: 500, Y: -400, Z: 400}

	constraints := &motionplan.Constraints{
		OrientationConstraint: []motionplan.OrientationConstraint{
			{OrientationToleranceDegs: 45},
		},
	}
	return Scenario{
		Key:         "orientation_constraint",
		Description: "Keep the EE orientation within 45 deg while moving between anchors.",
		Setup: func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error) {
			return nil, nil
		},
		Plan: alternateBetweenAnchors("orientation_constraint", anchorA, anchorB, constraints),
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
		Plan: alternateBetweenAnchors("dynamic_obstacle", anchorA, anchorB, nil),
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
			swing := alternateBetweenAnchors("multi_arm_choreography", anchorA, anchorB, nil)
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
		Plan: alternateBetweenAnchors("obstacle_progression", anchorA, anchorB, nil),
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
func alternateBetweenAnchors(
	scenarioKey string,
	anchorAOffset, anchorBOffset r3.Vector,
	constraints *motionplan.Constraints,
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
		goal := applyArmOffset(r.armBase(armName), goalOffset)
		if r.logger != nil {
			goalPt := goal.Point()
			r.logger.Infow("task-space: cartesian goal",
				"arm", armName,
				"scenario", scenarioKey,
				"anchor_picked", pickedLabel,
				"goal_world", []float64{goalPt.X, goalPt.Y, goalPt.Z},
				"goal_offset_local", []float64{goalOffset.X, goalOffset.Y, goalOffset.Z},
			)
		}
		plan, err := planSingleArmToPose(ctx, r, fs, armName, goal, obstacles, constraints)
		if err != nil {
			return nil, err
		}
		return plan, nil
	}
}
