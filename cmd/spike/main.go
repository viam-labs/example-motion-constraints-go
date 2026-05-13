// Spike program: throwaway exploration of the RDK motion-planning API.
//
// Answers two questions that gate the rest of the module:
//
//	OQ1: Can we get a planned trajectory back from armplanning.PlanMotion
//	     and walk it as discrete cartesian waypoints (for ghost preview)
//	     plus discrete joint inputs (for arm execution)?
//
//	OQ2: When the FrameSystem contains two arms, does PlanMotion treat the
//	     non-moving sibling arm's link geometries as collision obstacles
//	     automatically, or must we inject them into WorldState ourselves?
//
// Run with: go run ./cmd/spike
//
// This program is NOT shipped in the module tarball. The Makefile builds
// only cmd/module/main.go.
package main

import (
	"context"
	"fmt"
	"os"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm/fake"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

func main() {
	if err := run(); err != nil {
		fmt.Fprintf(os.Stderr, "spike failed: %v\n", err)
		os.Exit(1)
	}
}

func run() error {
	ctx := context.Background()
	logger := logging.NewLogger("spike")

	armA, err := newFakeArm(ctx, "arm_a", "ur5e", logger)
	if err != nil {
		return fmt.Errorf("build arm_a: %w", err)
	}
	armB, err := newFakeArm(ctx, "arm_b", "ur5e", logger)
	if err != nil {
		return fmt.Errorf("build arm_b: %w", err)
	}

	modelA, err := armA.Kinematics(ctx)
	if err != nil {
		return fmt.Errorf("arm_a kinematics: %w", err)
	}
	modelB, err := armB.Kinematics(ctx)
	if err != nil {
		return fmt.Errorf("arm_b kinematics: %w", err)
	}

	// Build a FrameSystem with two arms whose bases are 600mm apart along X.
	// Each arm's kinematic model is attached to a static offset frame that
	// places it in the world. With ur5e reach ~850mm, the arms' workspaces
	// overlap and you can deliberately point one arm into the other's volume.
	fs := referenceframe.NewEmptyFrameSystem("spike")

	offsetA, err := referenceframe.NewStaticFrame(
		"arm_a_origin",
		spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: 0}),
	)
	if err != nil {
		return fmt.Errorf("offsetA: %w", err)
	}
	if err := fs.AddFrame(offsetA, fs.World()); err != nil {
		return fmt.Errorf("add offsetA: %w", err)
	}
	if err := fs.AddFrame(modelA, offsetA); err != nil {
		return fmt.Errorf("add modelA: %w", err)
	}

	offsetB, err := referenceframe.NewStaticFrame(
		"arm_b_origin",
		spatialmath.NewPoseFromPoint(r3.Vector{X: 600, Y: 0, Z: 0}),
	)
	if err != nil {
		return fmt.Errorf("offsetB: %w", err)
	}
	if err := fs.AddFrame(offsetB, fs.World()); err != nil {
		return fmt.Errorf("add offsetB: %w", err)
	}
	if err := fs.AddFrame(modelB, offsetB); err != nil {
		return fmt.Errorf("add modelB: %w", err)
	}

	fmt.Printf("frame system frames: %v\n", fs.FrameNames())

	// Both arms start in their zero configuration. Build the StartState
	// covering every moving frame in the FrameSystem; PlanMotion requires
	// initial inputs for every DOF.
	zero := func(m referenceframe.Model) []referenceframe.Input {
		return make([]referenceframe.Input, len(m.DoF()))
	}
	startInputs := referenceframe.FrameSystemInputs{
		modelA.Name(): zero(modelA),
		modelB.Name(): zero(modelB),
	}
	startState := armplanning.NewPlanState(nil, startInputs)

	// Goal: drive arm A's tool frame to a pose roughly above arm B's base.
	// This deliberately routes through arm B's volume if the planner is
	// ignoring it.
	goalPose := spatialmath.NewPoseFromPoint(r3.Vector{X: 600, Y: 0, Z: 400})
	goalPoses := referenceframe.FrameSystemPoses{
		modelA.Name(): referenceframe.NewPoseInFrame(referenceframe.World, goalPose),
	}
	goalState := armplanning.NewPlanState(goalPoses, nil)

	// Plan with no extra WorldState first — purely the framesystem.
	worldState, err := referenceframe.NewWorldState(nil, nil)
	if err != nil {
		return fmt.Errorf("worldstate: %w", err)
	}

	req := &armplanning.PlanRequest{
		FrameSystem: fs,
		Goals:       []*armplanning.PlanState{goalState},
		StartState:  startState,
		WorldState:  worldState,
		Constraints: &motionplan.Constraints{},
	}

	plan, meta, err := armplanning.PlanMotion(ctx, logger, req)
	if err != nil {
		fmt.Printf("PlanMotion returned error: %v\n", err)
		fmt.Printf("OQ2 finding: framesystem-aware planning REJECTED the path (likely because arm B occupies the goal volume).\n")
		return nil
	}

	path := plan.Path()
	traj := plan.Trajectory()
	fmt.Printf("PlanMotion success in %v, %d goals processed, %d trajectory steps, %d path steps\n",
		meta.Duration, meta.GoalsProcessed, len(traj), len(path))

	// OQ1 evidence: dump the first/last cartesian poses and joint inputs for
	// arm A so we can confirm the trajectory is iterable as expected.
	if len(path) > 0 {
		first := path[0][modelA.Name()]
		last := path[len(path)-1][modelA.Name()]
		fmt.Printf("OQ1: arm A first pose (%s): %v\n", first.Parent(), first.Pose().Point())
		fmt.Printf("OQ1: arm A last pose  (%s): %v\n", last.Parent(), last.Pose().Point())
	}
	armAInputs, err := traj.GetFrameInputs(modelA.Name())
	if err == nil && len(armAInputs) > 0 {
		fmt.Printf("OQ1: arm A trajectory has %d joint waypoints; first=%v last=%v\n",
			len(armAInputs), armAInputs[0], armAInputs[len(armAInputs)-1])
	}

	// OQ2 evidence: at each waypoint, check whether arm A's link geometries
	// collide with arm B's link geometries (B is stationary at zero config).
	// If we see zero collisions despite a path that ends near B's base, the
	// planner is auto-routing around B. If we see collisions, the planner
	// IS NOT including B and we must inject B's geometries via WorldState.
	bGeoms, err := modelB.Geometries(zero(modelB))
	if err != nil {
		return fmt.Errorf("arm_b geometries: %w", err)
	}
	bWorld := transformGeometriesToWorld(bGeoms, fs, modelB.Name(), startInputs)
	fmt.Printf("arm B has %d link geometries (transformed to world)\n", len(bWorld))

	collisionCount := 0
	for i, step := range traj {
		aInputs, ok := step[modelA.Name()]
		if !ok {
			continue
		}
		aGeoms, err := modelA.Geometries(aInputs)
		if err != nil {
			fmt.Printf("step %d: arm A geometries error: %v\n", i, err)
			continue
		}
		aWorld := transformGeometriesToWorld(aGeoms, fs, modelA.Name(), step)
		colls := pairwiseCollide(aWorld, bWorld)
		if len(colls) > 0 {
			collisionCount++
			if collisionCount <= 3 {
				fmt.Printf("step %d: %d arm-A/arm-B collisions: %v\n", i, len(colls), colls)
			}
		}
	}
	fmt.Printf("OQ2 finding (no explicit obstacles): %d/%d trajectory steps have arm-A/arm-B link collisions.\n",
		collisionCount, len(traj))

	// Verification pass: inject arm B's link geometries into WorldState as
	// explicit obstacles in the world frame. If the planner now refuses the
	// plan (or routes around B), OQ2 is conclusive: implicit sibling
	// inclusion is OFF but explicit obstacle inclusion works.
	fmt.Println()
	fmt.Println("=== OQ2 verification: re-plan with arm B's link geometries injected as obstacles ===")
	// Rename the world-space copies so the planner doesn't see them as
	// duplicates of arm B's framesystem-owned link geometries.
	for i, g := range bWorld {
		g.SetLabel(fmt.Sprintf("arm_b_static_%d_%s", i, g.Label()))
	}
	bObstacles := referenceframe.NewGeometriesInFrame(referenceframe.World, bWorld)
	worldStateB, err := referenceframe.NewWorldState([]*referenceframe.GeometriesInFrame{bObstacles}, nil)
	if err != nil {
		return fmt.Errorf("worldstate with B obstacles: %w", err)
	}
	reqB := &armplanning.PlanRequest{
		FrameSystem: fs,
		Goals:       []*armplanning.PlanState{goalState},
		StartState:  startState,
		WorldState:  worldStateB,
		Constraints: &motionplan.Constraints{},
	}
	planB, _, errB := armplanning.PlanMotion(ctx, logger, reqB)
	if errB != nil {
		fmt.Printf("verification: planner REJECTED the path with arm B as explicit obstacle: %v\n", errB)
		fmt.Println("  → confirms OQ2 conclusion. Inject sibling-arm geometries into WorldState for every plan call.")
	} else {
		// It returned a plan — check whether THAT plan avoids arm B.
		trajB := planB.Trajectory()
		vCount := 0
		for _, step := range trajB {
			aIn, ok := step[modelA.Name()]
			if !ok {
				continue
			}
			aG, err := modelA.Geometries(aIn)
			if err != nil {
				continue
			}
			aW := transformGeometriesToWorld(aG, fs, modelA.Name(), step)
			if len(pairwiseCollide(aW, bWorld)) > 0 {
				vCount++
			}
		}
		fmt.Printf("verification: plan accepted; %d/%d steps still collide with arm B (lower=better).\n",
			vCount, len(trajB))
		if vCount < collisionCount {
			fmt.Println("  → explicit WorldState obstacles DO reduce collisions; this is the lever to pull for multi-arm scenarios.")
		}
	}
	return nil
}

// newFakeArm builds a fake arm of the given embedded model name. This uses
// the real fake-arm component constructor so the spike runs against the
// same kinematic models as production deployments.
func newFakeArm(ctx context.Context, name, armModel string, logger logging.Logger) (*fake.Arm, error) {
	conf := resource.Config{
		Name:                name,
		API:                 resource.NewAPI("rdk", "component", "arm"),
		Model:               fake.Model,
		ConvertedAttributes: &fake.Config{ArmModel: armModel},
	}
	a, err := fake.NewArm(ctx, resource.Dependencies{}, conf, logger)
	if err != nil {
		return nil, err
	}
	return a.(*fake.Arm), nil
}

// transformGeometriesToWorld walks the FrameSystem to transform each of a
// frame's link geometries into world coordinates, using the provided joint
// inputs to position the kinematic chain. Returns a flat slice of world-
// space geometries suitable for pairwise collision checks.
func transformGeometriesToWorld(
	gif *referenceframe.GeometriesInFrame,
	fs *referenceframe.FrameSystem,
	frameName string,
	inputs referenceframe.FrameSystemInputs,
) []spatialmath.Geometry {
	if gif == nil {
		return nil
	}
	out := make([]spatialmath.Geometry, 0, len(gif.Geometries()))
	for _, g := range gif.Geometries() {
		// Each link geometry is expressed in the kinematic chain's parent
		// frame (frameName). Transform its pose into world using the frame
		// system at the given inputs.
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

// pairwiseCollide returns the list of geometry-pair labels that collide
// between sets a and b, using the geometry's own CollidesWith method
// (zero buffer). Naive O(n*m); fine at link-count scale.
func pairwiseCollide(a, b []spatialmath.Geometry) [][2]string {
	var hits [][2]string
	for _, ag := range a {
		for _, bg := range b {
			collided, _, err := ag.CollidesWith(bg, 0)
			if err == nil && collided {
				hits = append(hits, [2]string{ag.Label(), bg.Label()})
			}
		}
	}
	return hits
}
