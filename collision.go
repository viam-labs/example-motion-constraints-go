package motionconstraints

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// checkTrajectoryCollisions walks the planner-returned trajectory and reports
// which obstacles, if any, intersect the arm's link geometries at any
// trajectory step. This is the "independent validation" step the Phase 3
// spike showed is load-bearing — the planner can return paths whose
// configurations still overlap world obstacles, so we never trust the
// planner alone for the educational red-tint highlight.
//
// Returns the set of obstacle labels that had at least one collision,
// plus the trajectory step index where the first collision occurred (for
// log diagnostics). Empty label list means clean trajectory.
func checkTrajectoryCollisions(
	ctx context.Context,
	armRes arm.Arm,
	armName string,
	traj motionplan.Trajectory,
	fs *referenceframe.FrameSystem,
	obstacles []scenarioObstacle,
) (collidedLabels []string, firstHitStep int, err error) {
	firstHitStep = -1
	if armRes == nil || len(traj) == 0 || len(obstacles) == 0 {
		return nil, firstHitStep, nil
	}
	model, err := armRes.Kinematics(ctx)
	if err != nil {
		return nil, firstHitStep, fmt.Errorf("kinematics: %w", err)
	}
	if model == nil {
		return nil, firstHitStep, nil
	}

	// Flatten obstacles into a label -> world geometry map. Obstacle poses
	// are already in world coordinates so no further transform is needed.
	obstaclesByLabel := map[string]spatialmath.Geometry{}
	obstacleLabels := make([]string, 0, len(obstacles))
	for _, ob := range obstacles {
		label := ob.label()
		obstaclesByLabel[label] = ob.Geom
		obstacleLabels = append(obstacleLabels, label)
	}
	collisionSet := map[string]struct{}{}

	for stepIdx, step := range traj {
		armInputs, ok := step[armName]
		if !ok {
			continue
		}
		linkGIF, err := model.Geometries(armInputs)
		if err != nil {
			continue
		}
		armLinksWorld := geometriesToWorld(fs, step, armName, linkGIF)
		for _, link := range armLinksWorld {
			for _, label := range obstacleLabels {
				if _, already := collisionSet[label]; already {
					continue
				}
				ob := obstaclesByLabel[label]
				collided, _, err := link.CollidesWith(ob, 0)
				if err == nil && collided {
					collisionSet[label] = struct{}{}
					if firstHitStep < 0 {
						firstHitStep = stepIdx
					}
				}
			}
		}
		// Early exit if all obstacles have collided already.
		if len(collisionSet) == len(obstacleLabels) {
			break
		}
	}

	collidedLabels = make([]string, 0, len(collisionSet))
	for k := range collisionSet {
		collidedLabels = append(collidedLabels, k)
	}
	return collidedLabels, firstHitStep, nil
}

// geometriesToWorld transforms a frame-local GeometriesInFrame to a flat
// slice of world-space geometries by walking the FrameSystem. Each link
// geometry is repositioned individually so collision checks work in a
// single shared coordinate frame.
func geometriesToWorld(
	fs *referenceframe.FrameSystem,
	inputs referenceframe.FrameSystemInputs,
	frameName string,
	gif *referenceframe.GeometriesInFrame,
) []spatialmath.Geometry {
	if gif == nil {
		return nil
	}
	out := make([]spatialmath.Geometry, 0, len(gif.Geometries()))
	for _, g := range gif.Geometries() {
		tf, err := fs.Transform(
			inputs.ToLinearInputs(),
			referenceframe.NewGeometriesInFrame(frameName, []spatialmath.Geometry{g}),
			referenceframe.World,
		)
		if err != nil {
			continue
		}
		if gif2, ok := tf.(*referenceframe.GeometriesInFrame); ok {
			out = append(out, gif2.Geometries()...)
		}
	}
	return out
}
