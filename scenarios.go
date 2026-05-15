package motionconstraints

import (
	"context"
	"fmt"
	"time"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// Scenario is one self-contained motion-planning demo. The hooks are
// intentionally low-level: setup returns the world-frame geometries to
// publish before planning, plan returns the trajectory to execute, and
// execute commands the arm. The service orchestrates lifecycle, viz, and
// timing around these hooks so individual presets stay readable.
//
// armName is the per-arm binding established by the parallel scenario
// runner. Setup and Plan use it to place obstacles relative to the arm's
// world base (via r.armBase(armName)) so multiple arms can run the same
// preset in parallel without their scenes overlapping.
type Scenario struct {
	Key         string
	Description string

	// Setup returns the obstacles to publish before planning runs. Pose
	// embedded in each geometry is already in world coordinates (Setup is
	// responsible for adding the arm's world offset).
	Setup func(ctx context.Context, r *resolved, armName string) ([]scenarioObstacle, error)

	// Plan returns the motion plan for the given arm. The obstacles list
	// mirrors what Setup returned; the planner sees them as world-frame
	// collision geometries.
	Plan func(
		ctx context.Context,
		r *resolved,
		fs *referenceframe.FrameSystem,
		armName string,
		obstacles []scenarioObstacle,
	) (motionplan.Plan, error)
}

// scenarioObstacle is a single world-frame collision geometry plus its
// visualization color. Geom contains its own pose (see spatialmath.Geometry).
//
// If Anim is non-nil, the service's animation tick goroutine continuously
// updates the obstacle's pose via field-mask UPDATEs. The planner still
// uses Geom.Pose() snapshot at the moment of planning — the motion of the
// obstacle during arm execution is not currently fed back into planning.
type scenarioObstacle struct {
	Geom  spatialmath.Geometry
	Color *Color
	Anim  *obstacleAnimation
}

// label returns the geometry's label, falling back to a generic name if
// the geometry didn't carry one.
func (o *scenarioObstacle) label() string {
	if l := o.Geom.Label(); l != "" {
		return l
	}
	return "obstacle"
}

// runScenario executes a single scenario end-to-end on the named arm: emit
// obstacles, plan, emit ghost trajectory, drive the arm, then tear down
// the ghost trajectory. Obstacles persist after return — the caller is
// responsible for clearing them between scenarios.
//
// armName binds the scenario to a specific arm in the resolved deps. In
// parallel mode (Phase 10) the runner picks armName from the
// arm_scenarios config map; in legacy sequential mode it falls back to
// r.armOrder[0].
//
// returns the list of UUIDs added to the scene during this scenario so the
// caller can remove them later.
func (s *service) runScenario(ctx context.Context, scn Scenario, armName string) (addedUUIDs [][]byte, runErr error) {
	if scn.Setup == nil || scn.Plan == nil {
		return nil, fmt.Errorf("scenario %q is not fully implemented yet", scn.Key)
	}

	s.mu.Lock()
	r := s.deps
	previewSec := s.previewS
	density := s.previewDensity
	abortOnCollision := s.abortOnCollision
	s.mu.Unlock()
	if r == nil {
		return nil, fmt.Errorf("dependencies not yet resolved")
	}
	if armName == "" {
		if len(r.armOrder) == 0 {
			return nil, fmt.Errorf("no arms configured")
		}
		armName = r.armOrder[0]
	}

	log := s.logger
	scenarioStart := time.Now()
	s.recordStage(armName, "setup")
	log.Infow("scenario: setup begin", "key", scn.Key, "arm", armName, "preview_s", previewSec)

	// Setup: emit obstacles.
	setupStart := time.Now()
	obstacles, err := scn.Setup(ctx, r, armName)
	if err != nil {
		s.recordError(armName, "setup: "+err.Error())
		log.Errorw("scenario: setup failed", "key", scn.Key, "arm", armName, "err", err)
		return nil, fmt.Errorf("setup: %w", err)
	}
	log.Infow("scenario: setup produced obstacles",
		"key", scn.Key, "arm", armName,
		"count", len(obstacles),
		"setup_ms", time.Since(setupStart).Milliseconds(),
	)
	for _, ob := range obstacles {
		// Obstacle UUIDs are arm-scoped so two arms running the same
		// preset don't fight over a single scene-map entry.
		uuid := []byte("obstacle:" + armName + ":" + ob.label())
		obPose := ob.Geom.Pose()
		obPoint := obPose.Point()
		log.Infow("scenario: emit obstacle",
			"uuid", string(uuid),
			"label", ob.label(),
			"xyz", []float64{obPoint.X, obPoint.Y, obPoint.Z},
		)
		if err := s.emitADDED(uuid, obPose, geomToVizProto(ob.Geom), ob.Color, opacityPtr(0.85)); err != nil {
			log.Errorw("scenario: emit obstacle failed", "label", ob.label(), "err", err)
			return nil, fmt.Errorf("emit obstacle %s: %w", ob.label(), err)
		}
		addedUUIDs = append(addedUUIDs, uuid)
		if ob.Anim != nil {
			s.registerAnimation(uuid, *ob.Anim)
			log.Infow("scenario: animation registered", "uuid", string(uuid), "period_s", ob.Anim.PeriodS)
		}
	}

	// Build the FrameSystem and plan.
	s.recordStage(armName, "build_fs")
	fsStart := time.Now()
	fs, err := buildFrameSystem(ctx, r)
	if err != nil {
		s.recordError(armName, "build_fs: "+err.Error())
		log.Errorw("scenario: build frame system failed", "arm", armName, "err", err)
		return addedUUIDs, fmt.Errorf("build frame system: %w", err)
	}
	log.Infow("scenario: frame system built",
		"arm", armName,
		"fs_ms", time.Since(fsStart).Milliseconds(),
		"frame_count", len(fs.FrameNames()),
	)
	// Cap concurrent PlanMotion calls across all arms. cbirrt spawns
	// MP_NUM_THREADS worker goroutines per call (we re-exec with =2 in
	// cmd/module/main.go); this semaphore caps how many such calls happen
	// at once. CRITICAL: the slot is released IMMEDIATELY after Plan
	// returns — not via defer at the end of runScenario — because the
	// post-plan phases (preview emit, collision check, execute) are not
	// CPU-bound on cbirrt and shouldn't block other arms from planning.
	// Earlier versions held the slot through execute, which artificially
	// serialized scenarios and made `planning_in_flight` actually mean
	// "scenario_in_flight". Now those metrics actually measure planning.
	s.recordStage(armName, "queued")
	release, acqErr := s.acquirePlanSlot(ctx)
	if acqErr != nil {
		s.recordError(armName, "plan-queue: "+acqErr.Error())
		return addedUUIDs, fmt.Errorf("acquire plan slot: %w", acqErr)
	}
	s.recordStage(armName, "planning")
	planStart := time.Now()
	plan, err := scn.Plan(ctx, r, fs, armName, obstacles)
	planMS := time.Since(planStart).Milliseconds()
	release()
	if err != nil {
		s.recordError(armName, "plan: "+err.Error())
		log.Errorw("scenario: plan failed", "arm", armName, "plan_ms", planMS, "err", err)
		return addedUUIDs, fmt.Errorf("plan: %w", err)
	}
	log.Infow("scenario: plan ok", "arm", armName, "plan_ms", planMS)
	armRes, ok := r.arms[armName]
	if !ok {
		log.Errorw("scenario: unknown arm", "arm", armName, "configured_arms", r.armOrder)
		return addedUUIDs, fmt.Errorf("arm %q not configured", armName)
	}

	// Preview: emit one small ghost sphere per cartesian waypoint of the
	// moving frame. The waypoints come back sparse from the default
	// planner (often just start+goal); Phase 5 will densify.
	path := plan.Path()
	traj := plan.Trajectory()
	log.Infow("scenario: plan succeeded",
		"arm", armName,
		"path_points", len(path),
		"trajectory_steps", len(traj),
	)
	if len(path) > 0 {
		first := path[0][armName]
		last := path[len(path)-1][armName]
		if first != nil {
			p := first.Pose().Point()
			log.Infow("scenario: path first", "frame", first.Parent(), "xyz", []float64{p.X, p.Y, p.Z})
		}
		if last != nil {
			p := last.Pose().Point()
			log.Infow("scenario: path last", "frame", last.Parent(), "xyz", []float64{p.X, p.Y, p.Z})
		}
	}
	// Preview at the configured EE frame so gripper-equipped arms show
	// the trail at the tool tip instead of the wrist.
	previewFrame := r.eeFrame(armName)
	previewUUIDs := s.emitDenseTrajectoryGhosts(armName, previewFrame, traj, fs, density)
	log.Infow("scenario: ghost trail emitted", "count", len(previewUUIDs), "density", density)
	// Defer ghost-trail cleanup so EVERY exit path (success, collision-
	// abort, execute error) tears down its trail. Without this, an
	// abort_on_collision=true scenario would leave ~30 sphere+axis
	// entities in the scene every iteration — observed as a runaway
	// scene_count (948 entities after ~33 cycles per arm).
	defer func() {
		for _, uuid := range previewUUIDs {
			_ = s.emitREMOVED(uuid)
		}
	}()
	_ = path // path is still useful for diagnostics; ghosts come from traj now

	// Pre-flight collision check: walk the trajectory's arm link geometries
	// against the world obstacles. Any hit recolors the offending obstacle
	// red. See NOTES.md OQ5 — the planner doesn't always honor obstacles, so
	// this independent check is the educational moment.
	collidedLabels, firstHit, ccErr := checkTrajectoryCollisions(ctx, armRes, armName, traj, fs, obstacles)
	if ccErr != nil {
		log.Warnw("scenario: collision check failed", "err", ccErr)
	}
	collisionDetected := len(collidedLabels) > 0
	if collisionDetected {
		log.Warnw("scenario: trajectory has collisions",
			"obstacles", collidedLabels,
			"first_hit_step", firstHit,
			"abort_on_collision", abortOnCollision,
		)
		for _, label := range collidedLabels {
			uuid := []byte("obstacle:" + armName + ":" + label)
			_ = s.emitColorUpdate(uuid, ColorCollision, opacityPtr(0.9))
		}
	} else {
		log.Infow("scenario: trajectory is collision-free")
	}

	// Pause briefly so a human eye sees the ghost trail before motion.
	if previewSec > 0 {
		select {
		case <-ctx.Done():
			return addedUUIDs, ctx.Err()
		case <-time.After(time.Duration(previewSec * float64(time.Second))):
		}
	}

	// Execute the joint waypoints. MoveThroughJointPositions is the
	// minimal-overhead way to drive a simulated arm through the plan;
	// real hardware would benefit from arm.MoveOptions for blending.
	armInputs, err := traj.GetFrameInputs(armName)
	if err != nil {
		log.Errorw("scenario: extract inputs failed", "arm", armName, "err", err)
		return addedUUIDs, fmt.Errorf("extract %q inputs: %w", armName, err)
	}
	if collisionDetected && abortOnCollision {
		log.Warnw("scenario: skipping execute due to collision",
			"arm", armName,
			"obstacles", collidedLabels,
		)
		s.recordStage(armName, "idle_after_collision")
		s.recordCycle(armName)
		return addedUUIDs, nil
	}
	s.recordStage(armName, "executing")
	// Joint-delta diagnostic: dump the targets we're about to send and
	// (after the move) read joints back to confirm the arm actually
	// changed configuration. If beforeJoints == afterJoints after a
	// supposedly-animated move, we know the simulated arm isn't ticking.
	beforeJoints, _ := armRes.JointPositions(ctx, nil)
	targetJoints := armInputs[len(armInputs)-1]
	maxDelta := 0.0
	for i := range targetJoints {
		if i >= len(beforeJoints) {
			break
		}
		d := float64(targetJoints[i] - beforeJoints[i])
		if d < 0 {
			d = -d
		}
		if d > maxDelta {
			maxDelta = d
		}
	}
	log.Infow("scenario: executing",
		"arm", armName,
		"waypoints", len(armInputs),
		"before_joints", inputsToFloats(beforeJoints),
		"target_joints", inputsToFloats(targetJoints),
		"max_joint_delta_rad", maxDelta,
	)
	if len(armInputs) > 1 {
		execStart := time.Now()
		if err := armRes.MoveThroughJointPositions(ctx, armInputs[1:], nil, nil); err != nil {
			s.recordError(armName, "execute: "+err.Error())
			log.Errorw("scenario: execute failed",
				"arm", armName,
				"exec_ms", time.Since(execStart).Milliseconds(),
				"err", err,
			)
			return addedUUIDs, fmt.Errorf("execute on %q: %w", armName, err)
		}
		afterJoints, _ := armRes.JointPositions(ctx, nil)
		actualMaxDelta := 0.0
		for i := range afterJoints {
			if i >= len(beforeJoints) {
				break
			}
			d := float64(afterJoints[i] - beforeJoints[i])
			if d < 0 {
				d = -d
			}
			if d > actualMaxDelta {
				actualMaxDelta = d
			}
		}
		log.Infow("scenario: executed",
			"arm", armName,
			"exec_ms", time.Since(execStart).Milliseconds(),
			"total_ms", time.Since(scenarioStart).Milliseconds(),
			"after_joints", inputsToFloats(afterJoints),
			"actual_max_delta_rad", actualMaxDelta,
			"expected_max_delta_rad", maxDelta,
		)
	} else {
		log.Warnw("scenario: trajectory too short to execute", "arm", armName, "waypoints", len(armInputs))
	}
	s.recordStage(armName, "idle")
	s.recordCycle(armName)

	// Ghost trail tear-down is handled by the defer set up at emit time
	// (covers every exit path: success, collision-abort, execute error).
	return addedUUIDs, nil
}

// emitDenseTrajectoryGhosts publishes the cartesian EE path the arm will
// actually follow, by interpolating joint inputs between consecutive
// planner-returned trajectory steps and transforming each substep through
// the FrameSystem. This is far more useful than emitting one sphere per
// planner keyframe — the default planner often returns 2-step trajectories
// (start, goal) which would otherwise show only the endpoints.
//
// `density` is the number of interpolated samples per segment, including
// the segment endpoints (so density=15 produces 14 evenly-spaced inner
// samples plus the endpoint between any two waypoints).
//
// Returns the UUIDs of every emitted ghost so the caller can clear them.
func (s *service) emitDenseTrajectoryGhosts(
	armName string,
	previewFrame string,
	traj motionplan.Trajectory,
	fs *referenceframe.FrameSystem,
	density int,
) [][]byte {
	if previewFrame == "" {
		previewFrame = armName
	}
	if density < 1 {
		density = 1
	}
	if len(traj) < 1 {
		return nil
	}

	out := make([][]byte, 0, len(traj)*density)
	ts := time.Now().UnixMilli()
	radius := 8.0
	color := ColorTrajectory
	opacity := opacityPtr(0.4)

	// Collect every sampled pose first so we can mark a few as "reference
	// frames" (axes triads) — start, end, and a small number of evenly-
	// spaced intermediates if the trail is long enough to warrant them.
	type sample struct {
		inputs referenceframe.FrameSystemInputs
		pose   spatialmath.Pose
	}
	samples := make([]sample, 0, len(traj)*density+1)
	addSample := func(inputs referenceframe.FrameSystemInputs) {
		pose, err := eePoseFromInputs(fs, inputs, previewFrame)
		if err != nil || pose == nil {
			return
		}
		samples = append(samples, sample{inputs: inputs, pose: pose})
	}
	addSample(traj[0])
	for i := 1; i < len(traj); i++ {
		prev := traj[i-1]
		curr := traj[i]
		for k := 1; k <= density; k++ {
			t := float64(k) / float64(density)
			addSample(lerpInputs(prev, curr, t))
		}
	}
	if len(samples) == 0 {
		return nil
	}

	// Reference-frame markers (axes triads): only start + end. Each axes
	// marker is its own scene entity (sphere + 3 axis cylinders the
	// renderer draws from show_axes_helper) and contributes
	// disproportionately to per-cycle message volume — intermediates were
	// nice but cost more than they were worth in browser render load.
	axesIdx := map[int]struct{}{0: {}, len(samples) - 1: {}}

	for i, sm := range samples {
		uuid := []byte(fmt.Sprintf("traj:%s:%d:%d", armName, ts, i))
		label := fmt.Sprintf("traj_%s_%d", armName, i)
		if err := s.emitADDED(uuid, sm.pose, sphereGeometry(radius, label), &color, opacity); err == nil {
			out = append(out, uuid)
		}
		if _, ok := axesIdx[i]; ok {
			axesUUID := []byte(fmt.Sprintf("traj_axes:%s:%d:%d", armName, ts, i))
			if err := s.emitAxesMarker(axesUUID, sm.pose); err == nil {
				out = append(out, axesUUID)
			}
		}
	}

	// Goal marker: a larger, gold-tinted sphere at the trajectory's
	// final pose so the user can clearly see what pose the planner was
	// aiming at (vs. the start/intermediate trajectory samples, which
	// are all the same green tint). Lives alongside the ghost trail and
	// gets cleaned up with it.
	goalSample := samples[len(samples)-1]
	goalUUID := []byte(fmt.Sprintf("traj_goal:%s:%d", armName, ts))
	goalLabel := fmt.Sprintf("goal_%s", armName)
	goalColor := ColorGoal
	if err := s.emitADDED(goalUUID, goalSample.pose, sphereGeometry(18, goalLabel), &goalColor, opacityPtr(0.7)); err == nil {
		out = append(out, goalUUID)
	}
	return out
}

// eePoseFromInputs returns the cartesian world-frame pose of the named
// frame given a full FrameSystemInputs map. Used to derive the EE position
// from interpolated joint values for ghost-trajectory rendering.
func eePoseFromInputs(
	fs *referenceframe.FrameSystem,
	inputs referenceframe.FrameSystemInputs,
	frameName string,
) (spatialmath.Pose, error) {
	tf, err := fs.Transform(
		inputs.ToLinearInputs(),
		referenceframe.NewPoseInFrame(frameName, spatialmath.NewZeroPose()),
		referenceframe.World,
	)
	if err != nil {
		return nil, err
	}
	pif, ok := tf.(*referenceframe.PoseInFrame)
	if !ok {
		return nil, fmt.Errorf("expected PoseInFrame from fs.Transform, got %T", tf)
	}
	return pif.Pose(), nil
}

// lerpInputs linearly interpolates two FrameSystemInputs maps at parameter
// t in [0,1]. Any frame missing from either input is skipped (no entry in
// the output) — the caller must supply matched maps from a single Plan.
func lerpInputs(a, b referenceframe.FrameSystemInputs, t float64) referenceframe.FrameSystemInputs {
	out := referenceframe.FrameSystemInputs{}
	for name, av := range a {
		bv, ok := b[name]
		if !ok || len(bv) != len(av) {
			out[name] = av
			continue
		}
		mixed := make([]referenceframe.Input, len(av))
		for i := range av {
			mixed[i] = av[i] + referenceframe.Input(t*(float64(bv[i])-float64(av[i])))
		}
		out[name] = mixed
	}
	return out
}

// removeUUIDs emits REMOVED for each UUID. Caller is responsible for
// remembering the UUIDs (this is a list, not a set).
func (s *service) removeUUIDs(uuids [][]byte) {
	for _, u := range uuids {
		_ = s.emitREMOVED(u)
	}
}

// poseToPB converts a spatialmath.Pose to a commonpb.Pose suitable for
// the PoseInObserverFrame field of a Transform.
func poseToPB(p spatialmath.Pose) *commonpb.Pose {
	pt := p.Point()
	ov := p.Orientation().OrientationVectorRadians()
	return &commonpb.Pose{
		X:     pt.X,
		Y:     pt.Y,
		Z:     pt.Z,
		OX:    ov.OX,
		OY:    ov.OY,
		OZ:    ov.OZ,
		Theta: ov.Theta * 180 / 3.141592653589793, // commonpb.Pose.Theta is degrees
	}
}

// geomToVizProto builds the dims-only geometry proto the 3D viewer expects.
// The viewer reads pose from the Transform's PoseInObserverFrame, NOT from
// the geometry's Center field — including Center is harmless but redundant.
func geomToVizProto(g spatialmath.Geometry) *commonpb.Geometry {
	proto := g.ToProtobuf()
	// Strip Center; the renderer doesn't read it.
	proto.Center = nil
	return proto
}

// staticBox is a small helper that creates a labeled spatialmath.Box at a
// fixed world pose. Used by presets to keep their setup hooks readable.
func staticBox(label string, x, y, z, dx, dy, dz float64) (spatialmath.Geometry, error) {
	pose := spatialmath.NewPoseFromPoint(r3.Vector{X: x, Y: y, Z: z})
	return spatialmath.NewBox(pose, r3.Vector{X: dx, Y: dy, Z: dz}, label)
}

// planSingleArmToPose builds the standard PlanRequest for a one-arm move and
// invokes armplanning.PlanMotion. Returns the plan plus the frame name being
// moved (so the caller can later iterate Path()/Trajectory() with that key).
func planSingleArmToPose(
	ctx context.Context,
	r *resolved,
	fs *referenceframe.FrameSystem,
	armName string,
	goal spatialmath.Pose,
	obstacles []scenarioObstacle,
	constraints *motionplan.Constraints,
) (motionplan.Plan, error) {
	armRes, ok := r.arms[armName]
	if !ok {
		return nil, fmt.Errorf("arm %q not in dependencies", armName)
	}
	log := r.logger
	// The arm's frame name in the framesystem matches the configured
	// component name. The kinematic model's primary output frame is the
	// arm's name in our case (matches viam-server's convention).
	current, err := armRes.JointPositions(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("current joints on %q: %w", armName, err)
	}
	// Build start inputs for EVERY moving frame in the system. If we omit
	// any moving frame's seed, validatePlanRequest rejects the request.
	startInputs := referenceframe.FrameSystemInputs{armName: current}
	for _, name := range fs.FrameNames() {
		if name == armName {
			continue
		}
		if _, has := startInputs[name]; has {
			continue
		}
		f := fs.Frame(name)
		if f == nil || len(f.DoF()) == 0 {
			continue
		}
		startInputs[name] = make([]referenceframe.Input, len(f.DoF()))
	}
	// Plan to the configured EE frame for this arm (a gripper, etc.) so
	// IK solves for the tool-tip rather than the wrist when an offset
	// frame is in play. Falls back to armName when no EE frame is
	// configured.
	goalFrame := r.eeFrame(armName)

	if log != nil {
		gp := goal.Point()
		gov := goal.Orientation().OrientationVectorDegrees()
		// Compute current EE in world for a same-units comparison with
		// the goal — when the planner says "you're already there", this
		// pair will be near-equal.
		var currentEEWorld []float64
		if ee, err := armRes.EndPosition(ctx, nil); err == nil && ee != nil {
			eePt := ee.Point()
			basePt := r.armBase(armName).Point()
			currentEEWorld = []float64{basePt.X + eePt.X, basePt.Y + eePt.Y, basePt.Z + eePt.Z}
		}
		log.Infow("plan: built request",
			"arm", armName,
			"goal_frame", goalFrame,
			"goal_world_xyz", []float64{gp.X, gp.Y, gp.Z},
			"goal_world_ov_deg", []float64{gov.OX, gov.OY, gov.OZ, gov.Theta},
			"current_ee_world_xyz", currentEEWorld,
			"current_joint_count", len(current),
			"obstacles_in_world_state", len(obstacles),
			"frames_in_start_state", len(startInputs),
		)
	}

	// Wrap obstacles into world-frame GeometriesInFrame for collision.
	geomList := make([]spatialmath.Geometry, 0, len(obstacles))
	for _, ob := range obstacles {
		geomList = append(geomList, ob.Geom)
	}
	var worldState *referenceframe.WorldState
	if len(geomList) > 0 {
		gif := referenceframe.NewGeometriesInFrame(referenceframe.World, geomList)
		worldState, err = referenceframe.NewWorldState([]*referenceframe.GeometriesInFrame{gif}, nil)
		if err != nil {
			return nil, fmt.Errorf("worldstate: %w", err)
		}
	} else {
		worldState, _ = referenceframe.NewWorldState(nil, nil)
	}

	goalPoses := referenceframe.FrameSystemPoses{
		goalFrame: referenceframe.NewPoseInFrame(referenceframe.World, goal),
	}
	req := &armplanning.PlanRequest{
		FrameSystem: fs,
		Goals:       []*armplanning.PlanState{armplanning.NewPlanState(goalPoses, nil)},
		StartState:  armplanning.NewPlanState(nil, startInputs),
		WorldState:  worldState,
		Constraints: constraints,
	}
	logger := r.logger
	if logger == nil {
		logger = logging.NewLogger("motionconstraints.plan")
	}
	// IMPORTANT: armplanning.PlanMotion does not enforce a timeout of its
	// own. When constraints + obstacles + goal produce an infeasible
	// problem the planner can hang indefinitely. A hard ctx deadline
	// kicks it out with DeadlineExceeded so the loop can move on.
	//
	// 3s budget. Most plans complete in <1s; the budget only matters for
	// genuinely hard cases. A STRUGGLING scenario costs ~budget worth of
	// CPU per cycle. Combined with the per-service plan-concurrency cap
	// (planSem), this puts an upper bound on how long the viam-server
	// Go runtime can stay saturated by cbirrt workers. Tighten tolerances
	// or shorten swings on the scenario itself if a hard problem is
	// genuinely needed.
	const planBudget = 3 * time.Second
	planCtx, cancel := context.WithTimeout(ctx, planBudget)
	defer cancel()
	plan, meta, err := armplanning.PlanMotion(planCtx, logger, req)
	if err != nil {
		logger.Errorw("plan: PlanMotion error", "err", err, "budget", planBudget)
		return nil, err
	}
	logger.Infow("plan: PlanMotion ok",
		"duration", meta.Duration,
		"path_points", len(plan.Path()),
		"traj_steps", len(plan.Trajectory()),
	)
	return plan, nil
}

func keysOfFrameSystemInputs(m referenceframe.FrameSystemInputs) []string {
	out := make([]string, 0, len(m))
	for k := range m {
		out = append(out, k)
	}
	return out
}

// inputsToFloats converts a []referenceframe.Input (which is a float64 alias)
// to a []float64 so zap.Infow can stringify it without panicking.
func inputsToFloats(inputs []referenceframe.Input) []float64 {
	out := make([]float64, len(inputs))
	for i, v := range inputs {
		out[i] = float64(v)
	}
	return out
}
