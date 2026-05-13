package motionconstraints

import (
	"context"
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// presetByKey returns the Scenario for a built-in preset key. Returns nil
// for keys whose implementations land in later phases — the runner skips
// nil scenarios with a clear log line rather than crashing on an unknown
// preset name.
func presetByKey(key string) *Scenario {
	switch key {
	case "single_arm_obstacle":
		s := presetSingleArmObstacle()
		return &s
	case "linear_constraint",
		"orientation_constraint",
		"dynamic_obstacle",
		"multi_arm_choreography":
		// Phase 7 / Phase 8 work — recognize the key but treat as not-yet-
		// implemented so the loop driver can log+skip cleanly.
		return nil
	default:
		return nil
	}
}

// presetSingleArmObstacle is the simplest motion-planning demo: one arm
// swings between two anchor poses on either side of a static box obstacle.
// Each scenario iteration alternates which anchor is the goal based on
// the arm's current EE pose, so the motion looks like a continuous back-
// and-forth swing rather than a one-shot move.
func presetSingleArmObstacle() Scenario {
	// Box centered between the two anchors at typical EE height. Wider in
	// Y than the anchor offset so a straight cartesian path is blocked.
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
		Plan: func(
			ctx context.Context,
			r *resolved,
			fs *referenceframe.FrameSystem,
			obstacles []scenarioObstacle,
		) (string, motionplan.Plan, error) {
			if len(r.armOrder) == 0 {
				return "", nil, fmt.Errorf("scenario requires at least one configured arm")
			}
			armName := r.armOrder[0]
			armRes := r.arms[armName]

			// Pick whichever anchor the arm isn't currently sitting at.
			// On the first run (zero config), distance from EE to both
			// anchors is similar; either one is a fine starting goal.
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
			plan, err := planSingleArmToPose(ctx, r, fs, armName, goal, obstacles, nil)
			if err != nil {
				return armName, nil, err
			}
			return armName, plan, nil
		},
	}
}

func dist3(dx, dy, dz float64) float64 {
	return dx*dx + dy*dy + dz*dz // squared distance is enough for ordering
}
