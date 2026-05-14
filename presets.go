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
// Each preset constructor returns a *fresh* Scenario value, so callers
// that want per-scenario state (cycle counters, etc.) should rely on
// closures over local atomics — see presetObstacleProgression.
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

// ---- single_arm_obstacle ---------------------------------------------------

// presetSingleArmObstacle is the simplest motion-planning demo: one arm
// swings between two anchor poses on either side of a static box obstacle.
func presetSingleArmObstacle() Scenario {
	const (
		boxX, boxY, boxZ    = 400.0, 0.0, 350.0
		boxDX, boxDY, boxDZ = 150.0, 300.0, 200.0
	)
	anchorA := spatialmath.NewPoseFromPoint(r3.Vector{X: 500, Y: 300, Z: 400})
	anchorB := spatialmath.NewPoseFromPoint(r3.Vector{X: 500, Y: -300, Z: 400})

	return Scenario{
		Key:         "single_arm_obstacle",
		Description: "One arm swings back and forth around a static box obstacle.",
		Setup: func(ctx context.Context, r *resolved) ([]scenarioObstacle, error) {
			geom, err := staticBox("single_arm_obstacle:box", boxX, boxY, boxZ, boxDX, boxDY, boxDZ)
			if err != nil {
				return nil, err
			}
			return []scenarioObstacle{{Geom: geom, Color: &ColorObstacle}}, nil
		},
		Plan: alternateBetweenAnchors("single_arm_obstacle", anchorA, anchorB, nil),
	}
}

// ---- linear_constraint -----------------------------------------------------

// presetLinearConstraint asks the planner to hold the EE on a straight
// cartesian line between two anchors. With a centered box, the strict
// line tolerance often forces the planner to return either a tightly
// curved path or no plan at all — both outcomes are educational.
func presetLinearConstraint() Scenario {
	const (
		boxX, boxY, boxZ    = 400.0, 0.0, 250.0
		boxDX, boxDY, boxDZ = 100.0, 100.0, 100.0
	)
	anchorA := spatialmath.NewPoseFromPoint(r3.Vector{X: 500, Y: 250, Z: 400})
	anchorB := spatialmath.NewPoseFromPoint(r3.Vector{X: 500, Y: -250, Z: 400})

	constraints := &motionplan.Constraints{
		LinearConstraint: []motionplan.LinearConstraint{
			{LineToleranceMm: 10, OrientationToleranceDegs: 30},
		},
	}

	return Scenario{
		Key:         "linear_constraint",
		Description: "Hold the EE on a straight line between anchors with a centered box obstacle.",
		Setup: func(ctx context.Context, r *resolved) ([]scenarioObstacle, error) {
			geom, err := staticBox("linear_constraint:box", boxX, boxY, boxZ, boxDX, boxDY, boxDZ)
			if err != nil {
				return nil, err
			}
			return []scenarioObstacle{{Geom: geom, Color: &ColorObstacle}}, nil
		},
		Plan: alternateBetweenAnchors("linear_constraint", anchorA, anchorB, constraints),
	}
}

// ---- orientation_constraint ------------------------------------------------

// presetOrientationConstraint asks the planner to hold the EE orientation
// fixed (within a small tolerance) while moving between two anchors. The
// box is small and offset so the planner has to maneuver while keeping
// the tool aligned — illustrating "pouring"-style motion.
func presetOrientationConstraint() Scenario {
	const (
		boxX, boxY, boxZ    = 400.0, 0.0, 350.0
		boxDX, boxDY, boxDZ = 100.0, 200.0, 100.0
	)
	anchorA := spatialmath.NewPoseFromPoint(r3.Vector{X: 500, Y: 300, Z: 400})
	anchorB := spatialmath.NewPoseFromPoint(r3.Vector{X: 500, Y: -300, Z: 400})

	constraints := &motionplan.Constraints{
		OrientationConstraint: []motionplan.OrientationConstraint{
			{OrientationToleranceDegs: 15},
		},
	}

	return Scenario{
		Key:         "orientation_constraint",
		Description: "Keep the EE orientation within 15 deg while moving between anchors.",
		Setup: func(ctx context.Context, r *resolved) ([]scenarioObstacle, error) {
			geom, err := staticBox("orientation_constraint:box", boxX, boxY, boxZ, boxDX, boxDY, boxDZ)
			if err != nil {
				return nil, err
			}
			return []scenarioObstacle{{Geom: geom, Color: &ColorObstacle}}, nil
		},
		Plan: alternateBetweenAnchors("orientation_constraint", anchorA, anchorB, constraints),
	}
}

// ---- dynamic_obstacle ------------------------------------------------------

// presetDynamicObstacle has the obstacle continuously oscillate between
// two world-frame poses while the arm swings between its anchor pair.
// The plan is recomputed each scenario iteration using the obstacle's
// pose at the moment of planning, so the visualization shows: planned
// arc → arm executes → obstacle drifts away to a new spot → next plan
// reroutes around the new position. Phase 8 wiring; Phase 6 collision
// red-tint still fires if a snapshot is unlucky.
func presetDynamicObstacle() Scenario {
	anchorA := spatialmath.NewPoseFromPoint(r3.Vector{X: 500, Y: 300, Z: 400})
	anchorB := spatialmath.NewPoseFromPoint(r3.Vector{X: 500, Y: -300, Z: 400})

	// The obstacle oscillates side-to-side across the arm's working volume.
	obstacleAnimA := spatialmath.NewPoseFromPoint(r3.Vector{X: 400, Y: 200, Z: 350})
	obstacleAnimB := spatialmath.NewPoseFromPoint(r3.Vector{X: 400, Y: -200, Z: 350})

	return Scenario{
		Key:         "dynamic_obstacle",
		Description: "Obstacle box oscillates continuously while the arm swings between anchors.",
		Setup: func(ctx context.Context, r *resolved) ([]scenarioObstacle, error) {
			// Start the obstacle at AnchorA; the animation tick will move
			// it from there. We can't read s.advanceAnimations' current
			// pose from inside the preset (the scenario doesn't have
			// service handle), so the planner uses whatever pose was
			// emitted most recently — close enough for educational use.
			geom, err := staticBox("dynamic_obstacle:box",
				obstacleAnimA.Point().X, obstacleAnimA.Point().Y, obstacleAnimA.Point().Z,
				150, 150, 200,
			)
			if err != nil {
				return nil, err
			}
			return []scenarioObstacle{{
				Geom: geom, Color: &ColorObstacle,
				Anim: &obstacleAnimation{
					AnchorA: obstacleAnimA,
					AnchorB: obstacleAnimB,
					PeriodS: 6.0,
				},
			}}, nil
		},
		Plan: alternateBetweenAnchors("dynamic_obstacle", anchorA, anchorB, nil),
	}
}

// ---- multi_arm_choreography ------------------------------------------------

// presetMultiArmChoreography reaches each configured arm toward a shared
// center point in world coordinates. Other arms' link geometries are
// injected into the WorldState for each plan call (the Phase 3 spike
// confirmed this is the only way to make planning respect siblings).
//
// Best run with a 2x2 grid (4 arms at the corners). Falls back to the
// single configured arm if fewer are present.
func presetMultiArmChoreography() Scenario {
	const sharedX, sharedY, sharedZ = 0.0, 0.0, 700.0
	goal := spatialmath.NewPoseFromPoint(r3.Vector{X: sharedX, Y: sharedY, Z: sharedZ})

	return Scenario{
		Key:         "multi_arm_choreography",
		Description: "Every configured arm reaches toward a shared center; arms treat each other as obstacles.",
		Setup: func(ctx context.Context, r *resolved) ([]scenarioObstacle, error) {
			// No static obstacles for this scenario — the obstacles are
			// the sibling arms themselves, injected per-plan inside Plan().
			return nil, nil
		},
		Plan: func(
			ctx context.Context,
			r *resolved,
			fs *referenceframe.FrameSystem,
			obstacles []scenarioObstacle,
		) (string, motionplan.Plan, error) {
			if len(r.armOrder) == 0 {
				return "", nil, fmt.Errorf("multi_arm_choreography requires at least one arm")
			}
			// Pick the next arm in round-robin order via shared state.
			armIdx := nextMultiArmIndex(len(r.armOrder))
			armName := r.armOrder[armIdx]

			// Inject the *other* arms' link geometries as world obstacles
			// so the planner avoids them. Sibling arms stay where they
			// currently are (no recomputation of their motion in this
			// scenario; multi-arm coordination is Phase 8+ work).
			siblingObstacles := []scenarioObstacle{}
			for i, name := range r.armOrder {
				if i == armIdx {
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
				joints, err := sibArm.JointPositions(ctx, nil)
				if err != nil {
					continue
				}
				gif, err := model.Geometries(joints)
				if err != nil || gif == nil {
					continue
				}
				// Build a one-frame inputs map for fs.Transform to use the
				// sibling arm's actual current joint positions.
				inputs := referenceframe.FrameSystemInputs{name: joints}
				for j, n2 := range r.armOrder {
					if j == armIdx || j == i {
						continue
					}
					if sib2, ok := r.arms[n2]; ok {
						if jp2, err := sib2.JointPositions(ctx, nil); err == nil {
							inputs[n2] = jp2
						}
					}
				}
				worldGeoms := geometriesToWorld(fs, inputs, name, gif)
				for k, g := range worldGeoms {
					// Rename to avoid duplicate-label collisions with
					// the framesystem-owned copies.
					g.SetLabel(fmt.Sprintf("sibling:%s:%d", name, k))
					siblingObstacles = append(siblingObstacles, scenarioObstacle{Geom: g, Color: &ColorObstacle})
				}
			}
			plan, err := planSingleArmToPose(ctx, r, fs, armName, goal, siblingObstacles, nil)
			if err != nil {
				return armName, nil, err
			}
			return armName, plan, nil
		},
	}
}

var multiArmCursor int64

func nextMultiArmIndex(armCount int) int {
	if armCount <= 1 {
		return 0
	}
	i := int(atomic.AddInt64(&multiArmCursor, 1)-1) % armCount
	if i < 0 {
		i = -i
	}
	return i
}

// ---- obstacle_progression --------------------------------------------------

// presetObstacleProgression reuses the single_arm_obstacle anchor pair but
// cycles the obstacle set on each iteration, adding one more geometry at
// a time. Pedagogical: shows the planner producing visibly different
// trajectories as constraints accumulate, and once the workspace is too
// tight the trajectory should fail collision check (red-tint payoff).
func presetObstacleProgression() Scenario {
	anchorA := spatialmath.NewPoseFromPoint(r3.Vector{X: 500, Y: 300, Z: 400})
	anchorB := spatialmath.NewPoseFromPoint(r3.Vector{X: 500, Y: -300, Z: 400})

	var counter int64

	return Scenario{
		Key:         "obstacle_progression",
		Description: "Same anchors; obstacles accumulate each cycle (box, +floor, +ceiling, +walls).",
		Setup: func(ctx context.Context, r *resolved) ([]scenarioObstacle, error) {
			stage := int(atomic.AddInt64(&counter, 1)-1) % 4

			obstacles := []scenarioObstacle{}
			boxGeom, err := staticBox("op:box", 400, 0, 350, 150, 300, 200)
			if err != nil {
				return nil, err
			}
			obstacles = append(obstacles, scenarioObstacle{Geom: boxGeom, Color: &ColorObstacle})

			// Stage 0: just the box. Stages 1-3 add successively.
			if stage >= 1 {
				floor, err := staticBox("op:floor", 0, 0, -5, 2500, 2500, 10)
				if err != nil {
					return nil, err
				}
				obstacles = append(obstacles, scenarioObstacle{Geom: floor, Color: &ColorObstacle})
			}
			if stage >= 2 {
				ceiling, err := staticBox("op:ceiling", 0, 0, 750, 2500, 2500, 10)
				if err != nil {
					return nil, err
				}
				obstacles = append(obstacles, scenarioObstacle{Geom: ceiling, Color: &ColorObstacle})
			}
			if stage >= 3 {
				wallPlus, err := staticBox("op:wall_plusY", 0, 800, 350, 2500, 10, 800)
				if err != nil {
					return nil, err
				}
				wallMinus, err := staticBox("op:wall_minusY", 0, -800, 350, 2500, 10, 800)
				if err != nil {
					return nil, err
				}
				obstacles = append(obstacles,
					scenarioObstacle{Geom: wallPlus, Color: &ColorObstacle},
					scenarioObstacle{Geom: wallMinus, Color: &ColorObstacle},
				)
			}
			return obstacles, nil
		},
		Plan: alternateBetweenAnchors("obstacle_progression", anchorA, anchorB, nil),
	}
}

// ---- shared helpers --------------------------------------------------------

// alternateBetweenAnchors returns a Plan hook that drives the configured
// arm toward whichever of two anchors is farther from its current EE
// pose. Optional constraints flow through to PlanMotion.
func alternateBetweenAnchors(
	scenarioKey string,
	anchorA, anchorB spatialmath.Pose,
	constraints *motionplan.Constraints,
) func(context.Context, *resolved, *referenceframe.FrameSystem, []scenarioObstacle) (string, motionplan.Plan, error) {
	return func(
		ctx context.Context,
		r *resolved,
		fs *referenceframe.FrameSystem,
		obstacles []scenarioObstacle,
	) (string, motionplan.Plan, error) {
		if len(r.armOrder) == 0 {
			return "", nil, fmt.Errorf("%s requires at least one configured arm", scenarioKey)
		}
		armName := r.armOrder[0]
		armRes := r.arms[armName]
		goal := anchorA
		if armRes != nil {
			if ee, err := armRes.EndPosition(ctx, nil); err == nil && ee != nil {
				eePt := ee.Point()
				aPt := anchorA.Point()
				bPt := anchorB.Point()
				dA := dist3(eePt.X-aPt.X, eePt.Y-aPt.Y, eePt.Z-aPt.Z)
				dB := dist3(eePt.X-bPt.X, eePt.Y-bPt.Y, eePt.Z-bPt.Z)
				if dA < dB {
					goal = anchorB
				}
			}
		}
		plan, err := planSingleArmToPose(ctx, r, fs, armName, goal, obstacles, constraints)
		if err != nil {
			return armName, nil, err
		}
		return armName, plan, nil
	}
}

func dist3(dx, dy, dz float64) float64 {
	return dx*dx + dy*dy + dz*dz
}
