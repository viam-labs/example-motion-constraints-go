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
type Scenario struct {
	Key         string
	Description string

	// Setup returns the obstacles to publish before planning runs. The
	// pose embedded in each geometry is in world coordinates.
	Setup func(ctx context.Context, r *resolved) ([]scenarioObstacle, error)

	// Plan returns the arm being moved and the resulting motion plan.
	// The obstacles list mirrors what Setup returned; the planner sees
	// them as world-frame collision geometries.
	Plan func(
		ctx context.Context,
		r *resolved,
		fs *referenceframe.FrameSystem,
		obstacles []scenarioObstacle,
	) (armName string, plan motionplan.Plan, err error)
}

// scenarioObstacle is a single world-frame collision geometry plus its
// visualization color. Geom contains its own pose (see spatialmath.Geometry).
type scenarioObstacle struct {
	Geom  spatialmath.Geometry
	Color *Color
}

// label returns the geometry's label, falling back to a generic name if
// the geometry didn't carry one.
func (o *scenarioObstacle) label() string {
	if l := o.Geom.Label(); l != "" {
		return l
	}
	return "obstacle"
}

// runScenario executes a single scenario end-to-end: emit obstacles, plan,
// emit ghost trajectory, drive the arm, then tear down the ghost trajectory.
// Obstacles persist after return — the caller is responsible for clearing
// them between scenarios.
//
// returns the list of UUIDs added to the scene during this scenario so the
// caller can remove them later.
func (s *service) runScenario(ctx context.Context, scn Scenario) (addedUUIDs [][]byte, runErr error) {
	if scn.Setup == nil || scn.Plan == nil {
		return nil, fmt.Errorf("scenario %q is not fully implemented yet", scn.Key)
	}

	s.mu.Lock()
	r := s.deps
	previewSec := s.previewS
	density := s.previewDensity
	s.mu.Unlock()
	if r == nil {
		return nil, fmt.Errorf("dependencies not yet resolved")
	}

	log := s.logger
	log.Infow("scenario: setup begin", "key", scn.Key, "preview_s", previewSec)

	// Setup: emit obstacles.
	obstacles, err := scn.Setup(ctx, r)
	if err != nil {
		log.Errorw("scenario: setup failed", "key", scn.Key, "err", err)
		return nil, fmt.Errorf("setup: %w", err)
	}
	log.Infow("scenario: setup produced obstacles", "key", scn.Key, "count", len(obstacles))
	for _, ob := range obstacles {
		uuid := []byte("obstacle:" + ob.label())
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
	}

	// Build the FrameSystem and plan.
	fs, err := buildFrameSystem(ctx, r)
	if err != nil {
		log.Errorw("scenario: build frame system failed", "err", err)
		return addedUUIDs, fmt.Errorf("build frame system: %w", err)
	}
	log.Infow("scenario: frame system built", "frames", fs.FrameNames())
	armName, plan, err := scn.Plan(ctx, r, fs, obstacles)
	if err != nil {
		log.Errorw("scenario: plan failed", "err", err)
		return addedUUIDs, fmt.Errorf("plan: %w", err)
	}
	armRes, ok := r.arms[armName]
	if !ok {
		log.Errorw("scenario: unknown arm returned by Plan", "arm", armName, "configured_arms", r.armOrder)
		return addedUUIDs, fmt.Errorf("plan returned unknown arm %q", armName)
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
	previewUUIDs := s.emitDenseTrajectoryGhosts(armName, traj, fs, density)
	addedUUIDs = append(addedUUIDs, previewUUIDs...)
	log.Infow("scenario: ghost trail emitted", "count", len(previewUUIDs), "density", density)
	_ = path // path is still useful for diagnostics; ghosts come from traj now

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
	log.Infow("scenario: executing", "arm", armName, "waypoints", len(armInputs))
	if len(armInputs) > 1 {
		// Skip the seed configuration; pass only forward-going waypoints.
		if err := armRes.MoveThroughJointPositions(ctx, armInputs[1:], nil, nil); err != nil {
			log.Errorw("scenario: execute failed", "arm", armName, "err", err)
			return addedUUIDs, fmt.Errorf("execute on %q: %w", armName, err)
		}
		log.Infow("scenario: executed", "arm", armName)
	} else {
		log.Warnw("scenario: trajectory too short to execute", "arm", armName, "waypoints", len(armInputs))
	}

	// Tear down the ghost trail. Obstacles remain on screen so the user
	// can see the result of the move.
	for _, uuid := range previewUUIDs {
		_ = s.emitREMOVED(uuid)
	}
	// Remove ghost UUIDs from the returned list (the caller doesn't need
	// to remove them again).
	addedUUIDs = addedUUIDs[:len(addedUUIDs)-len(previewUUIDs)]
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
	traj motionplan.Trajectory,
	fs *referenceframe.FrameSystem,
	density int,
) [][]byte {
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
	counter := 0
	emit := func(inputs referenceframe.FrameSystemInputs) {
		pose, err := eePoseFromInputs(fs, inputs, armName)
		if err != nil || pose == nil {
			return
		}
		uuid := []byte(fmt.Sprintf("traj:%s:%d:%d", armName, ts, counter))
		label := fmt.Sprintf("traj_%s_%d", armName, counter)
		if err := s.emitADDED(uuid, pose, sphereGeometry(radius, label), &color, opacity); err != nil {
			return
		}
		out = append(out, uuid)
		counter++
	}

	// First waypoint (start configuration).
	emit(traj[0])
	for i := 1; i < len(traj); i++ {
		prev := traj[i-1]
		curr := traj[i]
		for k := 1; k <= density; k++ {
			t := float64(k) / float64(density)
			emit(lerpInputs(prev, curr, t))
		}
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
	if log != nil {
		gp := goal.Point()
		log.Infow("plan: built request",
			"arm", armName,
			"goal_xyz", []float64{gp.X, gp.Y, gp.Z},
			"start_frames", keysOfFrameSystemInputs(startInputs),
			"obstacles", len(obstacles),
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
		armName: referenceframe.NewPoseInFrame(referenceframe.World, goal),
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
	plan, meta, err := armplanning.PlanMotion(ctx, logger, req)
	if err != nil {
		logger.Errorw("plan: PlanMotion error", "err", err)
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
