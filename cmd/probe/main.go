// Probe: offline reachability + plannability tester.
//
// Iterates over candidate (gripper offset, anchor pair, constraint) combos
// and reports which actually solve via armplanning.PlanMotion. Used to
// pick task-space desireds for the ee_variations bundle WITHOUT shipping
// to a real machine and waiting for the user to test.
//
// Run with: go run ./cmd/probe
//
// Not part of the module tarball.
package main

import (
	"context"
	"fmt"
	"os"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm/fake"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// planBudget mirrors the runtime module's per-plan timeout.
const planBudget = 3 * time.Second

type anchorPair struct {
	name string
	a, b r3.Vector // arm-local positions
}

type constraintSpec struct {
	name string
	make func() *motionplan.Constraints
}

type result struct {
	gripperZ int
	anchors  string
	cstr     string
	stage    string // "ik_a", "ik_b", "plan", "ok"
	duration time.Duration
	err      string
}

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, "probe:", err)
		os.Exit(1)
	}
}

func run() error {
	ctx := context.Background()
	logger := logging.NewLogger("probe")
	logger.SetLevel(logging.ERROR) // quiet the planner; we'll print our own results

	// Gripper z-offset values to test. The runtime config uses uniform 80;
	// we sweep a few to see how sensitive the plan is.
	gripperOffsets := []int{0, 40, 80, 120}

	// Candidate anchor pairs (arm-local positions). All are mirror-symmetric
	// in Y so the "swing" semantics are clear.
	anchorPairs := []anchorPair{
		{"Y100_z450", r3.Vector{X: 450, Y: 100, Z: 450}, r3.Vector{X: 450, Y: -100, Z: 450}},
		{"Y50_z450", r3.Vector{X: 450, Y: 50, Z: 450}, r3.Vector{X: 450, Y: -50, Z: 450}},
		{"Y200_z450", r3.Vector{X: 450, Y: 200, Z: 450}, r3.Vector{X: 450, Y: -200, Z: 450}},
		{"Y100_z500", r3.Vector{X: 450, Y: 100, Z: 500}, r3.Vector{X: 450, Y: -100, Z: 500}},
		{"Y100_z550", r3.Vector{X: 450, Y: 100, Z: 550}, r3.Vector{X: 450, Y: -100, Z: 550}},
		{"Y100_z400", r3.Vector{X: 450, Y: 100, Z: 400}, r3.Vector{X: 450, Y: -100, Z: 400}},
		{"forward500_Y100_z450", r3.Vector{X: 500, Y: 100, Z: 450}, r3.Vector{X: 500, Y: -100, Z: 450}},
		{"forward550_Y100_z450", r3.Vector{X: 550, Y: 100, Z: 450}, r3.Vector{X: 550, Y: -100, Z: 450}},
		{"forward400_Y100_z450", r3.Vector{X: 400, Y: 100, Z: 450}, r3.Vector{X: 400, Y: -100, Z: 450}},
		{"Z150_x500", r3.Vector{X: 500, Y: 0, Z: 400}, r3.Vector{X: 500, Y: 0, Z: 550}}, // vertical swing
		{"x150_y0_z450", r3.Vector{X: 400, Y: 0, Z: 450}, r3.Vector{X: 550, Y: 0, Z: 450}}, // forward swing
	}

	// Candidate constraint configurations.
	constraints := []constraintSpec{
		{"none", func() *motionplan.Constraints { return &motionplan.Constraints{} }},
		{"linear_200_180", func() *motionplan.Constraints {
			return &motionplan.Constraints{LinearConstraint: []motionplan.LinearConstraint{
				{LineToleranceMm: 200, OrientationToleranceDegs: 180},
			}}
		}},
		{"linear_50_180", func() *motionplan.Constraints {
			return &motionplan.Constraints{LinearConstraint: []motionplan.LinearConstraint{
				{LineToleranceMm: 50, OrientationToleranceDegs: 180},
			}}
		}},
		{"orient_45", func() *motionplan.Constraints {
			return &motionplan.Constraints{OrientationConstraint: []motionplan.OrientationConstraint{
				{OrientationToleranceDegs: 45},
			}}
		}},
		{"orient_90", func() *motionplan.Constraints {
			return &motionplan.Constraints{OrientationConstraint: []motionplan.OrientationConstraint{
				{OrientationToleranceDegs: 90},
			}}
		}},
		{"combined_200_90", func() *motionplan.Constraints {
			return &motionplan.Constraints{
				LinearConstraint: []motionplan.LinearConstraint{
					{LineToleranceMm: 200, OrientationToleranceDegs: 90},
				},
			}
		}},
		{"combined_200_45", func() *motionplan.Constraints {
			return &motionplan.Constraints{
				LinearConstraint: []motionplan.LinearConstraint{
					{LineToleranceMm: 200, OrientationToleranceDegs: 45},
				},
			}
		}},
		// Looser orient tolerances to find the boundary
		{"orient_60", func() *motionplan.Constraints {
			return &motionplan.Constraints{OrientationConstraint: []motionplan.OrientationConstraint{
				{OrientationToleranceDegs: 60},
			}}
		}},
		{"orient_120", func() *motionplan.Constraints {
			return &motionplan.Constraints{OrientationConstraint: []motionplan.OrientationConstraint{
				{OrientationToleranceDegs: 120},
			}}
		}},
		// PseudolinearConstraint (proportional, not absolute)
		{"plin_05_05", func() *motionplan.Constraints {
			return &motionplan.Constraints{PseudolinearConstraint: []motionplan.PseudolinearConstraint{
				{LineToleranceFactor: 0.5, OrientationToleranceFactor: 0.5},
			}}
		}},
		{"plin_10_10", func() *motionplan.Constraints {
			return &motionplan.Constraints{PseudolinearConstraint: []motionplan.PseudolinearConstraint{
				{LineToleranceFactor: 1.0, OrientationToleranceFactor: 1.0},
			}}
		}},
		{"plin_02_10", func() *motionplan.Constraints {
			return &motionplan.Constraints{PseudolinearConstraint: []motionplan.PseudolinearConstraint{
				{LineToleranceFactor: 0.2, OrientationToleranceFactor: 1.0},
			}}
		}},
		// Orient + position-loose linear
		{"linear_500_180", func() *motionplan.Constraints {
			return &motionplan.Constraints{LinearConstraint: []motionplan.LinearConstraint{
				{LineToleranceMm: 500, OrientationToleranceDegs: 180},
			}}
		}},
	}

	results := []result{}
	for _, gz := range gripperOffsets {
		for _, ap := range anchorPairs {
			for _, cs := range constraints {
				r := tryCombo(ctx, logger, gz, ap, cs)
				results = append(results, r)
			}
		}
	}

	// Print summary table.
	fmt.Printf("%-8s | %-22s | %-18s | %-6s | %-7s | %s\n",
		"grip_z", "anchors", "constraint", "stage", "ms", "err")
	fmt.Println(repeat('-', 110))
	for _, r := range results {
		errShort := r.err
		if len(errShort) > 50 {
			errShort = errShort[:50] + "..."
		}
		fmt.Printf("%-8d | %-22s | %-18s | %-6s | %-7d | %s\n",
			r.gripperZ, r.anchors, r.cstr, r.stage, r.duration.Milliseconds(), errShort)
	}

	// Find the constraint+anchor combos that work for ALL gripper offsets.
	fmt.Println()
	fmt.Println("=== Combos that succeed across all tested gripper offsets ===")
	type key struct{ anchors, cstr string }
	successesByKey := map[key]int{}
	totalsByKey := map[key]int{}
	for _, r := range results {
		k := key{r.anchors, r.cstr}
		totalsByKey[k]++
		if r.stage == "ok" {
			successesByKey[k]++
		}
	}
	for k, total := range totalsByKey {
		if successesByKey[k] == total {
			fmt.Printf("  %s + %s (n=%d)\n", k.anchors, k.cstr, total)
		}
	}
	return nil
}

func tryCombo(ctx context.Context, logger logging.Logger, gripperZ int, ap anchorPair, cs constraintSpec) result {
	r := result{gripperZ: gripperZ, anchors: ap.name, cstr: cs.name}

	// Build a fresh frame system per attempt so state doesn't leak.
	fs, model, gripperFrame, err := buildFrameSystem(ctx, logger, gripperZ)
	if err != nil {
		r.stage = "setup"
		r.err = err.Error()
		return r
	}

	// Start at the runtime's "ready" home pose so probe results match
	// what the runtime sees on the first plan of each cycle.
	// homeJointPositionsReady: j1=-π/2, j2=π/2, j3=-π/2, others=0.
	zeroInputs := make([]referenceframe.Input, len(model.DoF()))
	if len(zeroInputs) >= 6 {
		zeroInputs[0] = -1.5707963267948966
		zeroInputs[1] = 1.5707963267948966
		zeroInputs[2] = -1.5707963267948966
	}

	// Identity orientation goals at both anchors.
	goalA := spatialmath.NewPoseFromPoint(ap.a)
	goalB := spatialmath.NewPoseFromPoint(ap.b)

	// Try planning A → B with the configured EE frame and constraint.
	start := time.Now()
	planCtx, cancel := context.WithTimeout(ctx, planBudget)
	defer cancel()

	startInputs := referenceframe.FrameSystemInputs{
		model.Name(): zeroInputs,
	}
	startState := armplanning.NewPlanState(nil, startInputs)

	// Move from current pose to A, then A to B. We just test the A→B leg
	// since that's what the runtime scenarios do (alternateBetweenAnchors).
	// But the arm starts at zero config, so the first plan is actually
	// "zero config → A". Test that as a proxy.
	_ = goalA
	_ = goalB

	// Test goal A specifically.
	goalPoses := referenceframe.FrameSystemPoses{
		gripperFrame: referenceframe.NewPoseInFrame(referenceframe.World, goalA),
	}
	goalState := armplanning.NewPlanState(goalPoses, nil)

	worldState, _ := referenceframe.NewWorldState(nil, nil)
	req := &armplanning.PlanRequest{
		FrameSystem: fs,
		Goals:       []*armplanning.PlanState{goalState},
		StartState:  startState,
		WorldState:  worldState,
		Constraints: cs.make(),
	}
	_, _, err = armplanning.PlanMotion(planCtx, logger, req)
	r.duration = time.Since(start)
	if err != nil {
		errStr := err.Error()
		if containsAny(errStr, "zero IK solutions", "IK solutions") {
			r.stage = "ik_a"
		} else {
			r.stage = "plan"
		}
		r.err = errStr
		return r
	}
	r.stage = "ok"
	return r
}

// buildFrameSystem creates a single ur5e at the world origin with a
// gripper-tip static frame offset by (0, 0, gripperZ).
func buildFrameSystem(ctx context.Context, logger logging.Logger, gripperZ int) (
	*referenceframe.FrameSystem, referenceframe.Model, string, error,
) {
	armRes, err := newFakeArm(ctx, "ur5e_probe", "ur5e", logger)
	if err != nil {
		return nil, nil, "", fmt.Errorf("build arm: %w", err)
	}
	model, err := armRes.Kinematics(ctx)
	if err != nil {
		return nil, nil, "", fmt.Errorf("kinematics: %w", err)
	}

	fs := referenceframe.NewEmptyFrameSystem("probe")
	origin, err := referenceframe.NewStaticFrame("arm_origin",
		spatialmath.NewPoseFromPoint(r3.Vector{}))
	if err != nil {
		return nil, nil, "", fmt.Errorf("origin: %w", err)
	}
	if err := fs.AddFrame(origin, fs.World()); err != nil {
		return nil, nil, "", fmt.Errorf("add origin: %w", err)
	}
	if err := fs.AddFrame(model, origin); err != nil {
		return nil, nil, "", fmt.Errorf("add model: %w", err)
	}

	gripperName := "gripper_tip"
	gripperFrame, err := referenceframe.NewStaticFrame(gripperName,
		spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: float64(gripperZ)}))
	if err != nil {
		return nil, nil, "", fmt.Errorf("gripper frame: %w", err)
	}
	if err := fs.AddFrame(gripperFrame, model); err != nil {
		return nil, nil, "", fmt.Errorf("add gripper: %w", err)
	}

	return fs, model, gripperName, nil
}

func newFakeArm(ctx context.Context, name, armModel string, logger logging.Logger) (*fake.Arm, error) {
	conf := resource.Config{
		Name:                name,
		API:                 resource.NewAPI("rdk", "component", "arm"),
		Model:               fake.Model,
		ConvertedAttributes: &fake.Config{ArmModel: armModel},
	}
	armRes, err := fake.NewArm(ctx, resource.Dependencies{}, conf, logger)
	if err != nil {
		return nil, err
	}
	return armRes.(*fake.Arm), nil
}

func containsAny(s string, subs ...string) bool {
	for _, sub := range subs {
		if len(sub) > 0 && len(s) >= len(sub) {
			for i := 0; i+len(sub) <= len(s); i++ {
				if s[i:i+len(sub)] == sub {
					return true
				}
			}
		}
	}
	return false
}

func repeat(c byte, n int) string {
	out := make([]byte, n)
	for i := range out {
		out[i] = c
	}
	return string(out)
}
