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
// must reach a goal pose with a static box blocking the direct path. This
// is the canonical "hello world" scenario the rest of the module builds on.
func presetSingleArmObstacle() Scenario {
	return Scenario{
		Key:         "single_arm_obstacle",
		Description: "One arm plans around a static box obstacle.",
		Setup: func(ctx context.Context, r *resolved) ([]scenarioObstacle, error) {
			// Box sized 200x200x200mm centered halfway between the arm's
			// base and the goal pose, at the typical EE working height.
			geom, err := staticBox("single_arm_obstacle:box", 300, 0, 300, 200, 200, 200)
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
			// Goal: 600mm in +X, 0 in Y, 300mm up. The box at (300,0,300)
			// blocks the straight-line cartesian path so the planner must
			// route around it.
			goal := spatialmath.NewPoseFromPoint(r3.Vector{X: 600, Y: 0, Z: 300})
			plan, err := planSingleArmToPose(ctx, r, fs, armName, goal, obstacles, nil)
			if err != nil {
				return armName, nil, err
			}
			return armName, plan, nil
		},
	}
}
