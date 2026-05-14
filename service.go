// Package motionconstraints implements a Viam world_state_store service that
// orchestrates scripted motion-planning scenarios on a configurable grid of
// simulated arms. It publishes planned trajectories, obstacles, and collision
// state to the Viam 3D scene viewer.
//
// Status: Phase 4 — first end-to-end scenario. The service resolves arms +
// the framesystem service, runs the `single_arm_obstacle` preset on a loop
// (emitting obstacle + ghost-trajectory geometries + driving the arm), and
// responds to DoCommand verbs to override the loop. See CLAUDE.md and
// NOTES.md for the longer-term plan.
package motionconstraints

import (
	"context"
	"fmt"
	"runtime"
	"sort"
	"sync"
	"time"

	commonpb "go.viam.com/api/common/v1"
	pb "go.viam.com/api/service/worldstatestore/v1"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/worldstatestore"
	"go.viam.com/rdk/spatialmath"
)

// Model is the resource model registered by this module.
var Model = resource.NewModel("viam", "example-motion-constraints-go", "planner")

const (
	DefaultIntervalS       = 3.0
	DefaultPreviewS        = 1.0
	DefaultTickHz          = 30.0
	DefaultPreviewDensity  = 15
	maxTickHz              = 30.0
	subscriberBufSize      = 256
)

var builtinPresets = []string{
	"single_arm_obstacle",
	"linear_constraint",
	"orientation_constraint",
	"dynamic_obstacle",
	"multi_arm_choreography",
	"obstacle_progression",
}

func init() {
	resource.RegisterService(worldstatestore.API, Model,
		resource.Registration[worldstatestore.Service, *Config]{
			Constructor: newService,
		},
	)
}

// service is the world_state_store implementation. State, subscribers, and
// the scenario loop live here. Field access is protected by `mu` except
// where noted (the subscriber channels themselves are safe for concurrent
// send via the non-blocking broadcast).
type service struct {
	resource.Named

	logger logging.Logger

	mu  sync.Mutex
	cfg *Config

	// Active scene entities, keyed by UUID-as-string. Each entry carries
	// the live transform so initial-burst can replay it to new subscribers
	// and removeUUIDs can emit a REMOVED that round-trips correctly.
	scene map[string]*commonpb.Transform

	// Active subscribers. broadcastLocked walks this slice.
	subscribers []chan worldstatestore.TransformChange

	// Animated obstacles. The animation tick goroutine reads this list
	// and re-emits UPDATED transforms each tick.
	animations []animState

	// Scenario loop + animation tick control. Reconfigure cancels both
	// and starts fresh ones.
	tickCancel context.CancelFunc
	tickDone   chan struct{}
	animCancel context.CancelFunc
	animDone   chan struct{}
	advanceSig chan struct{} // buffered cap-1; `next`/`run` poke this

	// Resolved dependencies.
	deps *resolved

	// Cached config values.
	armNames      []string
	motionService string
	tickHz             float64
	intervalS          float64
	previewS           float64
	previewDensity     int
	abortOnCollision   bool
	loop          bool
	paused        bool
	presets       []string
	// armScenarios is the parallel-mode binding (arm name -> preset key).
	// Empty in legacy sequential mode.
	armScenarios map[string]string

	// Diagnostic state. Updated under s.mu by runScenario; surfaced by
	// the "stats" DoCommand verb.
	cycleCount     map[string]int64
	lastStageByArm map[string]string
	lastStageAtByArm map[string]time.Time
	lastErrorByArm map[string]string
	// pinnedScenario, if non-empty, is run exactly once before the loop
	// resumes — set by DoCommand `run`. Legacy mode only.
	pinnedScenario string
}

func newService(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (worldstatestore.Service, error) {
	s := &service{
		Named:            conf.ResourceName().AsNamed(),
		logger:           logger,
		scene:            map[string]*commonpb.Transform{},
		tickHz:           DefaultTickHz,
		intervalS:        DefaultIntervalS,
		previewS:         DefaultPreviewS,
		loop:             true,
		advanceSig:       make(chan struct{}, 1),
		cycleCount:       map[string]int64{},
		lastStageByArm:   map[string]string{},
		lastStageAtByArm: map[string]time.Time{},
		lastErrorByArm:   map[string]string{},
	}
	if err := s.Reconfigure(ctx, deps, conf); err != nil {
		return nil, err
	}
	return s, nil
}

// Reconfigure (re)parses the config, refreshes the resolved dependency set,
// restarts the scenario loop, and notifies existing subscribers of the new
// world. Subscribers see REMOVED for prior scene entities + ADDED for fresh
// ones (the latter happens as scenarios run).
func (s *service) Reconfigure(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
) error {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return err
	}
	r, err := resolveDeps(deps, cfg, s.logger)
	if err != nil {
		return err
	}
	// Cache each arm's world-frame base pose for relative-to-arm scenario
	// coordinates. Best-effort — failures fall back to zero pose.
	r.populateArmBases(ctx)

	s.mu.Lock()
	prevCancel := s.tickCancel
	prevDone := s.tickDone

	// Snapshot the prior scene so we can emit REMOVED on reconfigure.
	priorScene := s.scene
	s.scene = map[string]*commonpb.Transform{}
	s.cfg = cfg
	s.deps = r
	s.armNames = append(s.armNames[:0], cfg.Arms...)
	s.motionService = cfg.MotionService

	if cfg.TickHz > 0 {
		s.tickHz = cfg.TickHz
		if s.tickHz > maxTickHz {
			s.tickHz = maxTickHz
		}
	} else {
		s.tickHz = DefaultTickHz
	}
	if cfg.IntervalS > 0 {
		s.intervalS = cfg.IntervalS
	} else {
		s.intervalS = DefaultIntervalS
	}
	s.previewS = DefaultPreviewS
	if cfg.PreviewDensity > 0 {
		s.previewDensity = cfg.PreviewDensity
	} else {
		s.previewDensity = DefaultPreviewDensity
	}
	if cfg.AbortOnCollision != nil {
		s.abortOnCollision = *cfg.AbortOnCollision
	} else {
		s.abortOnCollision = true
	}
	if cfg.Loop != nil {
		s.loop = *cfg.Loop
	} else {
		s.loop = true
	}
	s.paused = false
	if len(cfg.Presets) > 0 {
		s.presets = append(s.presets[:0], cfg.Presets...)
	} else {
		s.presets = []string{"single_arm_obstacle"}
	}
	s.armScenarios = nil
	if len(cfg.ArmScenarios) > 0 {
		s.armScenarios = make(map[string]string, len(cfg.ArmScenarios))
		for k, v := range cfg.ArmScenarios {
			s.armScenarios[k] = v
		}
	}
	s.pinnedScenario = ""

	// Emit REMOVED for prior scene entities so existing subscribers don't
	// see ghosts from a previous configuration.
	for _, t := range priorScene {
		s.broadcastLocked(worldstatestore.TransformChange{
			ChangeType: pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_REMOVED,
			Transform:  t,
		})
	}
	s.mu.Unlock()

	if prevCancel != nil {
		prevCancel()
		<-prevDone
	}
	// Stop and clear any prior animations.
	s.mu.Lock()
	prevAnimCancel := s.animCancel
	prevAnimDone := s.animDone
	s.animations = nil
	s.mu.Unlock()
	if prevAnimCancel != nil {
		prevAnimCancel()
		<-prevAnimDone
	}

	tickCtx, cancel := context.WithCancel(context.Background())
	done := make(chan struct{})
	animCtx, animCancel := context.WithCancel(context.Background())
	animDone := make(chan struct{})
	s.mu.Lock()
	s.tickCancel = cancel
	s.tickDone = done
	s.animCancel = animCancel
	s.animDone = animDone
	s.mu.Unlock()
	go s.runLoop(tickCtx, done)
	go s.animationLoop(animCtx, animDone)

	s.logger.Infow("example-motion-constraints-go (re)configured",
		"name", conf.ResourceName().Name,
		"arms", s.armNames,
		"motion_service", s.motionService,
		"tick_hz", s.tickHz,
		"interval_s", s.intervalS,
		"loop", s.loop,
		"presets", s.presets,
	)
	return nil
}

// Close stops the scenario loop, emits REMOVED for any remaining scene
// entities, and tears down all subscribers.
func (s *service) Close(ctx context.Context) error {
	s.mu.Lock()
	cancel := s.tickCancel
	done := s.tickDone
	animCancel := s.animCancel
	animDone := s.animDone
	subs := s.subscribers
	s.subscribers = nil
	scene := s.scene
	s.scene = map[string]*commonpb.Transform{}
	s.animations = nil
	s.mu.Unlock()

	if cancel != nil {
		cancel()
		<-done
	}
	if animCancel != nil {
		animCancel()
		<-animDone
	}
	for _, t := range scene {
		change := worldstatestore.TransformChange{
			ChangeType: pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_REMOVED,
			Transform:  t,
		}
		for _, ch := range subs {
			select {
			case ch <- change:
			default:
			}
		}
	}
	for _, ch := range subs {
		close(ch)
	}
	return nil
}

// runLoop is the scenario driver. In parallel mode (cfg.ArmScenarios non-
// empty) it spawns one goroutine per (arm, scenario) pair and waits for
// ctx to cancel. In legacy sequential mode it cycles through cfg.Presets
// on r.armOrder[0].
func (s *service) runLoop(ctx context.Context, done chan struct{}) {
	defer close(done)

	s.mu.Lock()
	parallel := s.armScenarios
	s.mu.Unlock()
	if len(parallel) > 0 {
		s.runParallelLoops(ctx, parallel)
		return
	}

	cursor := 0
	for {
		if ctx.Err() != nil {
			return
		}

		s.mu.Lock()
		paused := s.paused
		loop := s.loop
		pinned := s.pinnedScenario
		s.pinnedScenario = ""
		presets := append([]string{}, s.presets...)
		interval := s.intervalS
		s.mu.Unlock()

		var key string
		switch {
		case pinned != "":
			key = pinned
		case paused:
			if !s.sleepCancelable(ctx, time.Second) {
				return
			}
			continue
		case len(presets) == 0:
			if !s.sleepCancelable(ctx, time.Second) {
				return
			}
			continue
		default:
			if cursor >= len(presets) {
				if !loop {
					if !s.sleepCancelable(ctx, time.Second) {
						return
					}
					continue
				}
				cursor = 0
			}
			key = presets[cursor]
			cursor++
		}

		scn := presetByKey(key)
		if scn == nil {
			s.logger.Infow("scenario not implemented; skipping", "key", key)
			continue
		}

		s.logger.Infow("running scenario", "key", key)
		uuids, err := s.runScenario(ctx, *scn, "")
		if err != nil {
			s.logger.Warnw("scenario failed", "key", key, "err", err)
		}
		// Hold the obstacles on screen during the inter-scenario interval.
		// They're idempotent (same UUID) on re-emit, so this just leaves
		// the scene populated until the user actively clears it.
		_ = uuids
		select {
		case <-ctx.Done():
			return
		case <-time.After(time.Duration(interval * float64(time.Second))):
		case <-s.advanceSig:
		}
	}
}

// runParallelLoops fan-outs one goroutine per (arm, scenarioKey) entry in
// armScenarios. Each goroutine independently runs its scenario on the
// configured interval. Returns when ctx is canceled and every per-arm
// goroutine has drained.
func (s *service) runParallelLoops(ctx context.Context, bindings map[string]string) {
	var wg sync.WaitGroup
	for armName, key := range bindings {
		armName, key := armName, key // capture
		wg.Add(1)
		go func() {
			defer wg.Done()
			s.runArmLoop(ctx, armName, key)
		}()
	}
	s.logger.Infow("parallel-mode scenario loops started", "count", len(bindings))
	wg.Wait()
}

// runArmLoop is one per-arm scenario goroutine. It re-fetches a fresh
// Scenario value each iteration so per-scenario counters (e.g.
// obstacle_progression's cycle counter) reset per goroutine — each arm
// gets independent counter state, which is the natural behavior in
// parallel mode.
func (s *service) runArmLoop(ctx context.Context, armName, scenarioKey string) {
	scn := presetByKey(scenarioKey)
	if scn == nil {
		s.logger.Warnw("runArmLoop: unknown scenario; arm idle", "arm", armName, "key", scenarioKey)
		return
	}
	for {
		if ctx.Err() != nil {
			return
		}
		s.mu.Lock()
		interval := s.intervalS
		paused := s.paused
		s.mu.Unlock()

		if !paused {
			s.logger.Infow("parallel: running scenario", "arm", armName, "key", scenarioKey)
			if _, err := s.runScenario(ctx, *scn, armName); err != nil {
				s.logger.Warnw("parallel: scenario failed", "arm", armName, "key", scenarioKey, "err", err)
			}
		}
		select {
		case <-ctx.Done():
			return
		case <-time.After(time.Duration(interval * float64(time.Second))):
		case <-s.advanceSig:
		}
	}
}

func (s *service) sleepCancelable(ctx context.Context, d time.Duration) bool {
	select {
	case <-ctx.Done():
		return false
	case <-time.After(d):
		return true
	case <-s.advanceSig:
		return true
	}
}

// emitADDED constructs and broadcasts an ADDED transform. The geometry is
// stored in the scene map so it can be re-emitted to late subscribers and
// removed on demand. pose is in world coordinates; refFrame is the parent
// frame the renderer should attach the entity to (almost always "world").
//
// If the UUID is already present in the scene map, emitADDED is a no-op —
// callers can safely re-invoke each scenario iteration without flicker.
func (s *service) emitADDED(
	uuid []byte,
	pose spatialmath.Pose,
	geom *commonpb.Geometry,
	color *Color,
	opacity *float64,
) error {
	if pose == nil {
		pose = spatialmath.NewZeroPose()
	}
	tf := &commonpb.Transform{
		Uuid:           uuid,
		ReferenceFrame: stringFromBytes(uuid),
		PoseInObserverFrame: &commonpb.PoseInFrame{
			ReferenceFrame: "world",
			Pose:           poseToPB(pose),
		},
		PhysicalObject: geom,
		Metadata: buildMetadata(metadataOpts{
			Color:   color,
			Opacity: opacity,
		}),
	}
	s.mu.Lock()
	_, exists := s.scene[string(uuid)]
	s.mu.Unlock()
	if exists {
		// Refresh color/opacity in place instead of re-emitting ADDED.
		// Keeps the cycle from re-broadcasting redundant ADDEDs and lets a
		// scenario reset a previously-collided (red) obstacle back to its
		// default tint at the start of the next iteration.
		if color != nil {
			return s.emitColorUpdate(uuid, *color, opacity)
		}
		s.logger.Debugw("emitADDED skipped (UUID already in scene, no recolor)", "uuid", string(uuid))
		return nil
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	s.scene[string(uuid)] = tf
	geomKind := "unknown"
	if geom != nil {
		switch geom.GeometryType.(type) {
		case *commonpb.Geometry_Box:
			geomKind = "box"
		case *commonpb.Geometry_Sphere:
			geomKind = "sphere"
		case *commonpb.Geometry_Capsule:
			geomKind = "capsule"
		case *commonpb.Geometry_Mesh:
			geomKind = "mesh"
		}
	}
	pb_pose := tf.PoseInObserverFrame.GetPose()
	s.logger.Infow("emitADDED",
		"uuid", string(uuid),
		"geom_kind", geomKind,
		"geom_label", geom.GetLabel(),
		"parent_frame", tf.PoseInObserverFrame.GetReferenceFrame(),
		"pose_xyz", []float64{pb_pose.GetX(), pb_pose.GetY(), pb_pose.GetZ()},
		"subscribers", len(s.subscribers),
		"scene_count", len(s.scene),
	)
	s.broadcastLocked(worldstatestore.TransformChange{
		ChangeType: pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_ADDED,
		Transform:  tf,
	})
	return nil
}

// emitAxesMarker publishes a "reference frame" entity at the given pose —
// the renderer draws a 3-axis triad (red X, green Y, blue Z) using the
// metadata.show_axes_helper flag. The accompanying geometry is invisible
// since the triad is the interesting part. UUIDs should be unique per
// emission (the caller's ts:i scheme handles that).
func (s *service) emitAxesMarker(uuid []byte, pose spatialmath.Pose) error {
	if pose == nil {
		pose = spatialmath.NewZeroPose()
	}
	// A tiny invisible sphere keeps the geometry slot populated; the
	// renderer reads pose from the Transform and the axes triad from
	// metadata.show_axes_helper.
	geom := sphereGeometry(1.0, stringFromBytes(uuid))
	tf := &commonpb.Transform{
		Uuid:           uuid,
		ReferenceFrame: stringFromBytes(uuid),
		PoseInObserverFrame: &commonpb.PoseInFrame{
			ReferenceFrame: "world",
			Pose:           poseToPB(pose),
		},
		PhysicalObject: geom,
		Metadata: buildMetadata(metadataOpts{
			Invisible:      true,
			ShowAxesHelper: true,
		}),
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if _, exists := s.scene[string(uuid)]; exists {
		return nil
	}
	s.scene[string(uuid)] = tf
	s.broadcastLocked(worldstatestore.TransformChange{
		ChangeType: pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_ADDED,
		Transform:  tf,
	})
	return nil
}

// emitColorUpdate replaces an existing entity's color (and opacity) without
// rotating the UUID. Emits an UPDATED change with a field-mask covering the
// affected metadata keys so the renderer should pick up the recolor in
// place. If the viewer turns out NOT to repaint on field-mask UPDATEs
// (NOTES.md OQ3), we fall back to versioned-UUID re-add — see emitRecolorVia
// ReAdd below for that path.
func (s *service) emitColorUpdate(uuid []byte, newColor Color, opacity *float64) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	tf, ok := s.scene[string(uuid)]
	if !ok {
		return nil
	}
	tf.Metadata = buildMetadata(metadataOpts{Color: &newColor, Opacity: opacity})
	s.logger.Infow("emitColorUpdate",
		"uuid", string(uuid),
		"new_color", []int{newColor.R, newColor.G, newColor.B},
	)
	s.broadcastLocked(worldstatestore.TransformChange{
		ChangeType:    pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_UPDATED,
		Transform:     tf,
		UpdatedFields: []string{"metadata.colors", "metadata.opacities"},
	})
	return nil
}

// emitREMOVED retires an entity by UUID. Returns nil even if the entity
// was already removed; this verb is meant to be safely retryable.
func (s *service) emitREMOVED(uuid []byte) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	tf, ok := s.scene[string(uuid)]
	if !ok {
		return nil
	}
	delete(s.scene, string(uuid))
	s.logger.Infow("emitREMOVED", "uuid", string(uuid), "scene_count", len(s.scene))
	s.broadcastLocked(worldstatestore.TransformChange{
		ChangeType: pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_REMOVED,
		Transform:  tf,
	})
	return nil
}

// stringFromBytes returns a plain string copy of a UUID byte slice. We
// stamp it onto Transform.ReferenceFrame so the viewer has a readable
// identifier even when the UUID has non-printable bytes.
func stringFromBytes(b []byte) string {
	return string(b)
}

// broadcastLocked sends a change to every subscriber, non-blocking. Caller
// must hold s.mu.
func (s *service) broadcastLocked(change worldstatestore.TransformChange) {
	for _, ch := range s.subscribers {
		select {
		case ch <- change:
		default:
			s.logger.Warnw("subscriber queue full; dropping change",
				"change_type", change.ChangeType.String())
		}
	}
}

// ListUUIDs returns the UUIDs of every entity currently in the scene.
func (s *service) ListUUIDs(ctx context.Context, extra map[string]any) ([][]byte, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	out := make([][]byte, 0, len(s.scene))
	for _, tf := range s.scene {
		out = append(out, tf.Uuid)
	}
	return out, nil
}

// GetTransform fetches a single entity by UUID. Returns nil if absent.
func (s *service) GetTransform(
	ctx context.Context,
	uuid []byte,
	extra map[string]any,
) (*commonpb.Transform, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	tf, ok := s.scene[string(uuid)]
	if !ok {
		return nil, nil
	}
	return tf, nil
}

// StreamTransformChanges registers a subscriber. The subscriber receives an
// initial burst of ADDED events for the current scene followed by live
// changes as scenarios run.
func (s *service) StreamTransformChanges(
	ctx context.Context,
	extra map[string]any,
) (*worldstatestore.TransformChangeStream, error) {
	ch := make(chan worldstatestore.TransformChange, subscriberBufSize)

	s.mu.Lock()
	// Initial burst.
	burst := 0
	for _, tf := range s.scene {
		select {
		case ch <- worldstatestore.TransformChange{
			ChangeType: pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_ADDED,
			Transform:  tf,
		}:
			burst++
		default:
			s.logger.Warnw("subscriber join: initial burst dropped event")
		}
	}
	s.subscribers = append(s.subscribers, ch)
	sCount := len(s.subscribers)
	s.mu.Unlock()
	s.logger.Infow("subscriber joined", "subscribers", sCount, "initial_burst", burst)

	go func() {
		<-ctx.Done()
		s.removeSubscriber(ch)
		s.logger.Infow("subscriber left")
	}()
	return worldstatestore.NewTransformChangeStreamFromChannel(ctx, ch), nil
}

func (s *service) removeSubscriber(target chan worldstatestore.TransformChange) {
	s.mu.Lock()
	defer s.mu.Unlock()
	for i, ch := range s.subscribers {
		if ch == target {
			s.subscribers = append(s.subscribers[:i], s.subscribers[i+1:]...)
			close(ch)
			return
		}
	}
}

// DoCommand is the manual-control surface for scenarios.
//
// Verbs:
//   - {"command":"list"}                    → returns the catalog of built-in preset keys
//   - {"command":"status"}                  → returns current loop state
//   - {"command":"pause"} / "resume"        → toggles scenario advancement
//   - {"command":"clear"}                   → emits REMOVED for every scene entity
//   - {"command":"run","scenario":"<key>"}  → runs a specific preset on the next loop iteration
//   - {"command":"next"}                    → skips the inter-scenario sleep and advances now
func (s *service) DoCommand(
	ctx context.Context,
	cmd map[string]any,
) (map[string]any, error) {
	verb, _ := cmd["command"].(string)
	switch verb {
	case "list":
		presets := append([]string{}, builtinPresets...)
		sort.Strings(presets)
		return map[string]any{
			"presets":     presets,
			"implemented": presets, // all built-in presets are implemented
		}, nil

	case "status":
		s.mu.Lock()
		defer s.mu.Unlock()
		return map[string]any{
			"phase":           "4-single-arm",
			"paused":          s.paused,
			"loop":            s.loop,
			"tick_hz":         s.tickHz,
			"interval_s":      s.intervalS,
			"arms":            append([]string{}, s.armNames...),
			"motion_service":  s.motionService,
			"presets":         append([]string{}, s.presets...),
			"pinned_scenario": s.pinnedScenario,
			"scene_count":     len(s.scene),
		}, nil

	case "pause":
		s.mu.Lock()
		s.paused = true
		s.mu.Unlock()
		return map[string]any{"paused": true}, nil

	case "resume":
		s.mu.Lock()
		s.paused = false
		s.mu.Unlock()
		s.poke()
		return map[string]any{"paused": false}, nil

	case "clear":
		s.mu.Lock()
		count := len(s.scene)
		uuids := make([][]byte, 0, count)
		for k := range s.scene {
			uuids = append(uuids, []byte(k))
		}
		s.mu.Unlock()
		for _, u := range uuids {
			_ = s.emitREMOVED(u)
		}
		return map[string]any{"cleared": count}, nil

	case "run":
		key, _ := cmd["scenario"].(string)
		if key == "" {
			return nil, fmt.Errorf("missing 'scenario' field on run command")
		}
		if presetByKey(key) == nil {
			return nil, fmt.Errorf("scenario %q is not implemented yet", key)
		}
		s.mu.Lock()
		s.pinnedScenario = key
		s.mu.Unlock()
		s.poke()
		return map[string]any{"queued": key}, nil

	case "next":
		s.poke()
		return map[string]any{"advanced": true}, nil

	case "stats":
		s.mu.Lock()
		stages := map[string]any{}
		stageAges := map[string]any{}
		now := time.Now()
		for k, v := range s.lastStageByArm {
			stages[k] = v
			if t, ok := s.lastStageAtByArm[k]; ok {
				stageAges[k] = now.Sub(t).Seconds()
			}
		}
		cycles := map[string]any{}
		for k, v := range s.cycleCount {
			cycles[k] = v
		}
		errs := map[string]any{}
		for k, v := range s.lastErrorByArm {
			errs[k] = v
		}
		sceneCount := len(s.scene)
		subCount := len(s.subscribers)
		animCount := len(s.animations)
		s.mu.Unlock()
		return map[string]any{
			"phase":                 "10-parallel",
			"goroutines":            runtime.NumGoroutine(),
			"scene_count":           sceneCount,
			"subscribers":           subCount,
			"animations":            animCount,
			"cycles":                cycles,
			"current_stage":         stages,
			"current_stage_age_sec": stageAges,
			"last_error":            errs,
		}, nil

	default:
		return nil, fmt.Errorf("unrecognized command %q; try one of: list, status, pause, resume, clear, run, next", verb)
	}
}

// poke wakes runLoop without filling the buffer.
func (s *service) poke() {
	select {
	case s.advanceSig <- struct{}{}:
	default:
	}
}

// recordStage tracks which scenario stage each arm is currently in so the
// "stats" DoCommand verb can identify stuck goroutines. Stages: "idle",
// "setup", "build_fs", "planning", "preview_wait", "executing",
// "interval_sleep", "errored".
func (s *service) recordStage(armName, stage string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.lastStageByArm[armName] = stage
	s.lastStageAtByArm[armName] = time.Now()
}

// recordCycle increments the per-arm completed-cycle counter and clears
// any prior error for that arm.
func (s *service) recordCycle(armName string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.cycleCount[armName]++
	delete(s.lastErrorByArm, armName)
}

// recordError stamps the most recent failure on an arm so callers of
// "stats" can see what each stuck arm last tripped on.
func (s *service) recordError(armName, errMsg string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.lastErrorByArm[armName] = errMsg
	s.lastStageByArm[armName] = "errored"
	s.lastStageAtByArm[armName] = time.Now()
}
