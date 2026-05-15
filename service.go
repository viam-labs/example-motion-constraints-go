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
	"os"
	"runtime"
	"sort"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	pb "go.viam.com/api/service/worldstatestore/v1"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/worldstatestore"
	"go.viam.com/rdk/spatialmath"
)

// Model is the resource model registered by this module.
var Model = resource.NewModel("viam", "example-motion-constraints-go", "motion-playground")

const (
	DefaultIntervalS       = 5.0
	DefaultPreviewS        = 1.0
	DefaultTickHz          = 30.0
	DefaultPreviewDensity  = 2
	maxTickHz              = 30.0
	// DefaultMaxConcurrentPlans is the default ceiling on simultaneous
	// armplanning.PlanMotion calls. Picked to leave roughly half of a
	// typical dev box's cores free for viam-server's gRPC handlers when
	// cbirrt is using NumCPU/2 worker goroutines per plan.
	DefaultMaxConcurrentPlans = 2
	// DefaultMaxPreviewGhosts caps trajectory-ghost emissions per plan to
	// keep the browser-side burst small. Picked empirically: 24 is dense
	// enough to read the path's shape (especially under linear constraint
	// where every ghost is on a straight line), small enough that the
	// emit burst doesn't stall the viewer's JS main thread.
	DefaultMaxPreviewGhosts = 24
	subscriberBufSize      = 256
)

// PresetBundles maps a friendly bundle name to a canonical
// (arm-name -> preset-key) mapping. Every bundle is exactly four arms
// — always `arm_a1..a4` — so switching bundles reuses the same machine
// config and the CPU/render cost stays predictable. Earlier wider
// bundles (rows AB/B/C, "all") proved heavier than the renderer could
// keep up with; the 4-arm constraint keeps every preset_set responsive.
//
// In each bundle: arm_a1 + arm_a2 are gripperless; arm_a3 + arm_a4
// expect offset grippers configured via the `ee_frames` attribute,
// matching the examples/grid-of-arms.json layout.
var PresetBundles = map[string]map[string]string{
	// EE control-frame variations: position-varying vs orientation-
	// varying, with vs without an offset gripper.
	"ee_only": {
		"arm_a1": "random_translation",
		"arm_a2": "random_rotation",
		"arm_a3": "random_translation",
		"arm_a4": "random_rotation",
	},
	// Same task-space variations as ee_only with a LinearConstraint
	// layered on where it actually applies. Rotation slots stay
	// unconstrained because a "stay on a zero-length cartesian line"
	// constraint is degenerate — cbirrt either fails IK fast or grinds
	// against the orientation interpolation until the plan budget
	// fires, both of which starve the viz.
	"ee_variations": {
		// Constraint-variation comparison: all 4 arms run the SAME
		// 2-anchor swing (Y±100/Z=450 arm-local) under DIFFERENT
		// constraint types. The differences are visible because Linear
		// and Combined force a notably different path than cbirrt's
		// natural arc; OrientationConstraint with tight tolerance
		// constrains the wrist's roll.
		//
		// Each ee_* scenario triggers a startup warmup (see
		// scenarioNeedsWarmup) — one unconstrained plan to the anchor
		// before the constrained loop — so arms that can't break through
		// tight constraints from the ready pose get unstuck first.
		"arm_a1": "ee_baseline", // no constraint — reference natural path
		"arm_a2": "ee_linear",   // LinearConstraint — forces straight cartesian line
		"arm_a3": "ee_orient",   // OrientationConstraint 90° — wrist orient locked
		"arm_a4": "ee_combined", // LinearConstraint + 45° orient — both at once
	},
	// Obstacle-geometry pedagogy: arc-over, duck-under, gripper-with-
	// box, corridor pass-through. gripper_with_box assumes arm_a3 has
	// a gripper configured (preferably one whose frame.geometry adds
	// the long tool collision body).
	"obstacle_geometry": {
		"arm_a1": "arc_over_obstacle",
		"arm_a2": "duck_under_obstacle",
		"arm_a3": "gripper_with_box",
		"arm_a4": "corridor_passthrough",
	},
	// Constraint and dynamic-obstacle variations.
	"constraint_types": {
		"arm_a1": "linear_constraint",
		"arm_a2": "orientation_constraint",
		"arm_a3": "dynamic_obstacle",
		"arm_a4": "single_arm_obstacle",
	},
}

// DefaultPresetSet is used when neither arm_scenarios nor preset_set is
// supplied. Picked to keep the demo lightweight (4 arms) and visually
// readable for first-time users; switch to "ee_variations" or "all"
// for richer demos at higher CPU cost.
const DefaultPresetSet = "ee_only"

// RowDescriptions are human-readable labels for the conceptual rows in
// the grid demo. Surfaced via DoCommand `list` so users can identify
// which row a given preset belongs to.
var RowDescriptions = map[string]string{
	"ee_only":           "End-Effector Control Frame Variations",
	"ee_variations":     "EE Variations Under a Linear Constraint",
	"obstacle_geometry": "Obstacle Geometry Variations",
	"constraint_types":  "Constraint and Dynamic-Obstacle Variations",
}

var builtinPresets = []string{
	"single_arm_obstacle",
	"linear_constraint",
	"orientation_constraint",
	"dynamic_obstacle",
	"multi_arm_choreography",
	"obstacle_progression",
	"random_translation",
	"random_rotation",
	"arc_over_obstacle",
	"duck_under_obstacle",
	"gripper_with_box",
	"corridor_passthrough",
	"random_translation_linear",
	"random_rotation_linear",
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
	disablePreviewGhosts bool
	maxPreviewGhosts   int
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

	// planSem caps the number of concurrent armplanning.PlanMotion calls
	// across all arms. cbirrt spawns runtime.NumCPU()/2 CPU-bound worker
	// goroutines per PlanMotion call; with N arms planning in parallel,
	// the Go scheduler inside viam-server gets saturated, starving the
	// gRPC stream goroutines that feed the 3D scene viewer (other Viam
	// app tabs aren't affected — they're request/response and tolerate
	// scheduler latency). Bounding concurrent plans is the cheapest
	// in-our-control mitigation (the per-plan thread count itself is set
	// from MP_NUM_THREADS at armplanning's init time — out of reach from
	// our main()). Buffered channel of cap MaxConcurrentPlans; acquire by
	// sending a sentinel, release by receiving. (chan-as-semaphore is the
	// idiomatic Go pattern for this.)
	planSem chan struct{}
	// planInFlight is the count of plans currently holding planSem slots.
	// planQueued is the count waiting on the semaphore. Both surfaced via
	// the "stats" verb so the user can directly observe whether the cap
	// is biting.
	planInFlight int
	planQueued   int
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
	s.disablePreviewGhosts = cfg.DisablePreviewGhosts
	switch {
	case cfg.MaxPreviewGhosts == 0:
		s.maxPreviewGhosts = DefaultMaxPreviewGhosts
	case cfg.MaxPreviewGhosts < 0:
		s.maxPreviewGhosts = 0 // uncapped
	default:
		s.maxPreviewGhosts = cfg.MaxPreviewGhosts
	}
	// (Re)build the plan-concurrency semaphore. We rebuild on every
	// Reconfigure so a config change in MaxConcurrentPlans takes effect
	// without a module restart. Any in-flight plans hold slots from the
	// PRIOR semaphore — that's fine; they release into a channel nobody
	// reads from any more (it just gets GC'd once the senders return).
	maxPlans := cfg.MaxConcurrentPlans
	if maxPlans <= 0 {
		maxPlans = DefaultMaxConcurrentPlans
	}
	s.planSem = make(chan struct{}, maxPlans)
	s.planInFlight = 0
	s.planQueued = 0
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
	// Clear per-arm diagnostics so stale entries from a prior config
	// don't leak into the "stats" verb. Without this, switching from a
	// 12-arm bundle to a 4-arm bundle leaves the 8 dropped arms looking
	// "errored" or "planning" in stats forever.
	s.cycleCount = map[string]int64{}
	s.lastStageByArm = map[string]string{}
	s.lastStageAtByArm = map[string]time.Time{}
	s.lastErrorByArm = map[string]string{}

	s.armScenarios = nil
	// Resolution order: explicit ArmScenarios > named PresetSet >
	// DefaultPresetSet. Bundles are filtered against the configured
	// arms — arms in the bundle that aren't declared in the machine
	// config silently drop out.
	switch {
	case len(cfg.ArmScenarios) > 0:
		s.armScenarios = make(map[string]string, len(cfg.ArmScenarios))
		for k, v := range cfg.ArmScenarios {
			s.armScenarios[k] = v
		}
	case cfg.PresetSet != "":
		bundle, ok := PresetBundles[cfg.PresetSet]
		if !ok {
			s.logger.Warnw("unknown preset_set; falling back to default",
				"requested", cfg.PresetSet, "default", DefaultPresetSet)
			bundle = PresetBundles[DefaultPresetSet]
		}
		s.armScenarios = filterBundleToConfiguredArms(bundle, cfg.Arms)
	default:
		s.armScenarios = filterBundleToConfiguredArms(
			PresetBundles[DefaultPresetSet], cfg.Arms)
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

	// Emit per-arm text labels (small PLY meshes generated offline by
	// scripts/generate_text_assets.py). Placed in front of each arm at
	// floor height so the viewer's default camera reads them. Failures
	// are non-fatal — the demo runs without labels if assets are
	// missing.
	s.emitArmLabelMeshes()

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
	// One-time startup home so the arm has a known starting pose
	// regardless of what state it carried over from a previous module
	// session (simulated-arm joint state persists across module
	// reloads in viam-server). The pose chosen depends on the scenario:
	//   - obstacle scenarios: candle pose (j1, j3 = -90deg) folds the
	//     arm up out of the typical obstacle region.
	//   - non-obstacle scenarios: all-zeros, the fresh-load default,
	//     so plans start from the arm's natural zero-config orientation
	//     that IK has continuous solutions around.
	s.mu.Lock()
	deps := s.deps
	s.mu.Unlock()
	if deps != nil {
		if armRes, ok := deps.arms[armName]; ok {
			if model, err := armRes.Kinematics(ctx); err == nil && model != nil {
				var home []referenceframe.Input
				var poseKind string
				if scenarioNeedsHome(scenarioKey) {
					home = homeJointPositionsCandle(len(model.DoF()))
					poseKind = "candle"
				} else {
					home = homeJointPositionsReady(len(model.DoF()))
					poseKind = "ready"
				}
				s.logger.Infow("home: moving arm to startup pose",
					"arm", armName, "scenario", scenarioKey,
					"pose_kind", poseKind, "home", inputsToFloats(home))
				if err := armRes.MoveToJointPositions(ctx, home, nil); err != nil {
					s.logger.Warnw("home: move failed (will run anyway)", "arm", armName, "err", err)
				}
			}
		}
	}
	// Warmup for constraint-based ee_ scenarios: an unconstrained plan
	// to the bundle's anchor A, executed in full, so the arm settles
	// into a goal-friendly joint state before the constrained scenario
	// begins. Empirically (see probe_constraints), arms stuck at the
	// ready pose can't break through tight constraints on the first
	// plan, but a single unconstrained plan first unlocks them.
	if scenarioNeedsWarmup(scenarioKey) {
		s.warmupArm(ctx, armName)
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

// acquirePlanSlot blocks until a slot is available in the plan-concurrency
// semaphore (or the context is cancelled). Returns a release func that the
// caller MUST invoke when the plan completes — typically via `defer release()`
// immediately after the call. The semaphore reference is snapshotted at
// acquire time so a Reconfigure between acquire and release doesn't break
// pairing (the old buffered channel still accepts the release send).
//
// While waiting, increments planQueued; while holding, increments planInFlight.
// Both counts are exposed by the "stats" verb.
func (s *service) acquirePlanSlot(ctx context.Context) (release func(), err error) {
	s.mu.Lock()
	sem := s.planSem
	s.planQueued++
	s.mu.Unlock()
	defer func() {
		s.mu.Lock()
		s.planQueued--
		s.mu.Unlock()
	}()
	select {
	case sem <- struct{}{}:
		s.mu.Lock()
		s.planInFlight++
		s.mu.Unlock()
		return func() {
			<-sem
			s.mu.Lock()
			s.planInFlight--
			s.mu.Unlock()
		}, nil
	case <-ctx.Done():
		return nil, ctx.Err()
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

// emitLabelMesh publishes an extruded-text PLY mesh at the given pose
// with the given label string. The PLY asset must already exist on
// disk (generated offline by scripts/generate_text_assets.py).
//
// Items are emitted at world-frame pose; the PLY itself is oriented
// "upright with front face at +Y" by the generator so the default
// Viam viewer camera reads them left-to-right.
func (s *service) emitLabelMesh(uuid []byte, pose spatialmath.Pose, label string, heightMM int) error {
	plyBytes, err := loadTextPLY(label, heightMM)
	if err != nil {
		return err
	}
	if pose == nil {
		pose = spatialmath.NewZeroPose()
	}
	geom := &commonpb.Geometry{
		Label: label,
		GeometryType: &commonpb.Geometry_Mesh{
			Mesh: &commonpb.Mesh{
				ContentType: "ply",
				Mesh:        plyBytes,
			},
		},
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
			Color:   &Color{R: 25, G: 25, B: 25},
			Opacity: opacityPtr(1.0),
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

// emitAxesMarker publishes a "reference frame" entity at the given pose —
// the renderer draws a 3-axis triad (red X, green Y, blue Z) using the
// metadata.show_axes_helper flag. UUIDs should be unique per emission
// (the caller's ts:i scheme handles that).
//
// Note: the renderer hides the entire entity when invisible=true,
// including the axes helper. So we leave invisible=false and ship a
// very-small sphere as the placeholder geometry; the sphere is too
// small to be distracting but keeps the entity "visible" so the axes
// helper renders.
func (s *service) emitAxesMarker(uuid []byte, pose spatialmath.Pose) error {
	if pose == nil {
		pose = spatialmath.NewZeroPose()
	}
	geom := sphereGeometry(3.0, stringFromBytes(uuid))
	tf := &commonpb.Transform{
		Uuid:           uuid,
		ReferenceFrame: stringFromBytes(uuid),
		PoseInObserverFrame: &commonpb.PoseInFrame{
			ReferenceFrame: "world",
			Pose:           poseToPB(pose),
		},
		PhysicalObject: geom,
		Metadata: buildMetadata(metadataOpts{
			Invisible:      false,
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
		// Also include the named bundles + their human-readable row
		// descriptions so callers can pick a preset_set without
		// reading the source.
		bundles := map[string]any{}
		for k, v := range PresetBundles {
			arms := make([]string, 0, len(v))
			for arm := range v {
				arms = append(arms, arm)
			}
			sort.Strings(arms)
			bundles[k] = map[string]any{
				"description": RowDescriptions[k],
				"arms":        arms,
			}
		}
		return map[string]any{
			"presets":     presets,
			"implemented": presets,
			"bundles":     bundles,
			"default":     DefaultPresetSet,
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
		// Break down scene entities by UUID prefix so we can see WHICH kind
		// is accumulating. UUIDs are namespaced — e.g. "traj:armname:ts:i",
		// "traj_axes:...", "traj_goal:...", "obstacle:armname:label",
		// "label:armname". If `traj:*` keeps growing across cycles, the
		// trajectory-ghost cleanup defer isn't firing for some path.
		sceneByPrefix := map[string]int{}
		for k := range s.scene {
			prefix := "other"
			for _, p := range []string{"traj_axes", "traj_goal", "traj", "obstacle", "label"} {
				if len(k) >= len(p) && k[:len(p)] == p {
					prefix = p
					break
				}
			}
			sceneByPrefix[prefix]++
		}
		subCount := len(s.subscribers)
		animCount := len(s.animations)
		planInFlight := s.planInFlight
		planQueued := s.planQueued
		planCap := cap(s.planSem)
		s.mu.Unlock()
		return map[string]any{
			"phase":                 "10-parallel",
			"goroutines":            runtime.NumGoroutine(),
			"scene_count":           sceneCount,
			"scene_by_prefix":       sceneByPrefix,
			"subscribers":           subCount,
			"animations":            animCount,
			"cycles":                cycles,
			"current_stage":         stages,
			"current_stage_age_sec": stageAges,
			"last_error":            errs,
			// Plan concurrency observability. If planning_queued stays > 0
			// and planning_in_flight is at the cap, the semaphore is biting
			// — arms are waiting for a planning slot. Increase
			// max_concurrent_plans to trade viz responsiveness for arm
			// parallelism.
			"planning_in_flight": planInFlight,
			"planning_queued":    planQueued,
			"planning_cap":       planCap,
		}, nil

	case "stack_dump":
		// Full goroutine stack dump. Diagnostic-only — use to compare a
		// healthy state (e.g. ee_only running smoothly) vs. a frozen state
		// (e.g. ee_variations with the 3D viewer unresponsive). Goroutines
		// blocked on a mutex or channel in the frozen dump but not in the
		// healthy dump point at the actual contention site.
		//
		// 4MB buffer is enough for ~10k goroutines worth of stack at typical
		// depths — way more than this module ever runs.
		buf := make([]byte, 4*1024*1024)
		n := runtime.Stack(buf, true)
		return map[string]any{
			"goroutines":   runtime.NumGoroutine(),
			"stack_bytes":  n,
			"stack":        string(buf[:n]),
			"mp_threads":   os.Getenv("MP_NUM_THREADS"),
			"num_cpu":      runtime.NumCPU(),
			"gomaxprocs":   runtime.GOMAXPROCS(0),
		}, nil

	case "probe_constraints":
		// Run a small sweep of (arm, constraint) plans inside the live
		// runtime — same code path as scenarios use, but with a controlled
		// set of constraint configs. Returns per-combo success/failure +
		// timing. Diagnostic-only.
		return s.probeConstraints(ctx, cmd)

	default:
		return nil, fmt.Errorf("unrecognized command %q; try one of: list, status, pause, resume, clear, run, next, stats, stack_dump, probe_constraints", verb)
	}
}

// scenarioNeedsWarmup reports whether the scenario benefits from an
// initial unconstrained plan to the ee anchor before the constrained
// scenario loop starts. See warmupArm for the why.
func scenarioNeedsWarmup(scenarioKey string) bool {
	switch scenarioKey {
	case "ee_linear", "ee_orient", "ee_orient_60", "ee_orient_120", "ee_combined":
		return true
	}
	return false
}

// warmupArm runs a single UNCONSTRAINED plan from the arm's current
// (ready-pose) state to ee_variations' anchor A and executes it. This
// puts the arm in a goal-friendly joint state so the subsequent
// constrained scenario can break through — empirically, arms stuck at
// ready pose can't satisfy LinearConstraint/Combined on the first
// plan, but the state after an unconstrained plan to the same anchor
// unlocks them.
//
// Best-effort: failures are logged and the loop proceeds anyway.
func (s *service) warmupArm(ctx context.Context, armName string) {
	s.mu.Lock()
	r := s.deps
	s.mu.Unlock()
	if r == nil {
		return
	}
	armRes, ok := r.arms[armName]
	if !ok {
		return
	}
	fs, err := buildFrameSystem(ctx, r)
	if err != nil {
		s.logger.Warnw("warmup: build frame system failed", "arm", armName, "err", err)
		return
	}
	goal := applyArmOffset(r.armBase(armName), eeAnchorA)
	warmupCtx, cancel := context.WithTimeout(ctx, 5*time.Second)
	defer cancel()
	plan, err := planSingleArmToPose(warmupCtx, r, fs, armName, goal, nil, nil)
	if err != nil {
		s.logger.Warnw("warmup: plan failed (will continue)", "arm", armName, "err", err)
		return
	}
	armInputs, err := plan.Trajectory().GetFrameInputs(armName)
	if err != nil {
		s.logger.Warnw("warmup: extract inputs failed", "arm", armName, "err", err)
		return
	}
	if len(armInputs) <= 1 {
		s.logger.Infow("warmup: plan trivial (already at anchor)", "arm", armName)
		return
	}
	if err := armRes.MoveThroughJointPositions(ctx, armInputs[1:], nil, nil); err != nil {
		s.logger.Warnw("warmup: execute failed", "arm", armName, "err", err)
		return
	}
	s.logger.Infow("warmup: complete", "arm", armName, "steps", len(armInputs))
}

// scenarioNeedsHome reports whether a preset's typical scene includes
// obstacles near the arm's zero/home config. Used to gate the one-time
// startup home move so unobstructed presets (row A, row AB) don't get
// tucked into a "candle" pose from which their linear-constrained first
// plan can't reach the first goal.
func scenarioNeedsHome(scenarioKey string) bool {
	switch scenarioKey {
	case "single_arm_obstacle",
		"arc_over_obstacle",
		"duck_under_obstacle",
		"gripper_with_box",
		"corridor_passthrough",
		"obstacle_progression",
		"multi_arm_choreography",
		"dynamic_obstacle":
		return true
	}
	return false
}

// emitArmLabelMeshes places a pre-generated text PLY mesh in front of
// each arm in the active bundle, with a brief description of its
// scenario. UUIDs are stable per arm so the labels survive subsequent
// reconfigures unless the bundle changes the arm's scenario or gripper.
//
// Layout: 600mm forward of the arm in world -Y (toward the default
// camera viewer) at floor level. The PLYs are oriented "upright, front
// face at +Y" by the generator, so they read left-to-right when viewed
// from the default camera angle.
func (s *service) emitArmLabelMeshes() {
	s.mu.Lock()
	deps := s.deps
	scenarios := s.armScenarios
	s.mu.Unlock()
	if deps == nil || len(scenarios) == 0 {
		return
	}
	const (
		labelHeightMM = 35 // matches scripts/generate_text_assets.py
		// Place the plaque well below the arm's mount so it sits clear
		// of the arm body even when joints fold the arm into low
		// configurations. The plaque is four lines tall (~200mm), so
		// the BOTTOM of the plaque sits ~500mm below the arm base.
		labelZ = -400.0
	)
	for armName, scenarioKey := range scenarios {
		base := deps.armBase(armName)
		if base == nil {
			continue
		}
		hasGripper := false
		if eeFrame, ok := deps.eeFrames[armName]; ok && eeFrame != "" && eeFrame != armName {
			hasGripper = true
		}
		label := labelTextForArm(scenarioKey, hasGripper)
		bp := base.Point()
		pose := spatialmath.NewPoseFromPoint(r3.Vector{
			X: bp.X,
			Y: bp.Y,
			Z: bp.Z + labelZ,
		})
		uuid := []byte("label:" + armName)
		if err := s.emitLabelMesh(uuid, pose, label, labelHeightMM); err != nil {
			s.logger.Warnw("emitArmLabelMeshes: skipping arm",
				"arm", armName, "label", label, "err", err)
		}
	}
}

// filterBundleToConfiguredArms returns a copy of the bundle containing
// only arms that appear in the configured arms list. Lets a customer
// pick a heavy bundle (e.g. "all") without first declaring every arm.
func filterBundleToConfiguredArms(bundle map[string]string, configuredArms []string) map[string]string {
	if len(configuredArms) == 0 {
		out := make(map[string]string, len(bundle))
		for k, v := range bundle {
			out[k] = v
		}
		return out
	}
	configured := make(map[string]struct{}, len(configuredArms))
	for _, name := range configuredArms {
		configured[name] = struct{}{}
	}
	out := make(map[string]string)
	for k, v := range bundle {
		if _, ok := configured[k]; ok {
			out[k] = v
		}
	}
	return out
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

// probeConstraints runs a small constraint-sweep inside the LIVE runtime
// so we get the actual runtime behavior — bypasses any divergence
// between cmd/probe and the real framesystem service.
//
// For each (arm, constraint) combo, calls planSingleArmToPose with a
// goal at the ee_variations anchor A pose and reports success/failure
// + timing. Single shot — call it again to re-run.
//
// Caller can optionally pass {"arm": "<name>"} to test a single arm,
// otherwise sweeps all configured arms.
func (s *service) probeConstraints(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	s.mu.Lock()
	r := s.deps
	armNames := append([]string{}, s.armNames...)
	s.mu.Unlock()
	if r == nil {
		return nil, fmt.Errorf("dependencies not yet resolved")
	}
	// Filter to a single arm if requested.
	if armOnly, ok := cmd["arm"].(string); ok && armOnly != "" {
		filtered := armNames[:0]
		for _, n := range armNames {
			if n == armOnly {
				filtered = append(filtered, n)
			}
		}
		armNames = filtered
	}

	// Same anchor as ee_variations bundle (eeAnchorA from presets.go).
	anchor := r3.Vector{X: 450, Y: 100, Z: 450}

	type cs struct {
		name string
		make func() *motionplan.Constraints
	}
	specs := []cs{
		{"none", func() *motionplan.Constraints { return &motionplan.Constraints{} }},
		{"linear_50_180", func() *motionplan.Constraints {
			return &motionplan.Constraints{LinearConstraint: []motionplan.LinearConstraint{
				{LineToleranceMm: 50, OrientationToleranceDegs: 180},
			}}
		}},
		{"linear_200_180", func() *motionplan.Constraints {
			return &motionplan.Constraints{LinearConstraint: []motionplan.LinearConstraint{
				{LineToleranceMm: 200, OrientationToleranceDegs: 180},
			}}
		}},
		{"linear_500_180", func() *motionplan.Constraints {
			return &motionplan.Constraints{LinearConstraint: []motionplan.LinearConstraint{
				{LineToleranceMm: 500, OrientationToleranceDegs: 180},
			}}
		}},
		{"orient_45", func() *motionplan.Constraints {
			return &motionplan.Constraints{OrientationConstraint: []motionplan.OrientationConstraint{
				{OrientationToleranceDegs: 45},
			}}
		}},
		{"orient_60", func() *motionplan.Constraints {
			return &motionplan.Constraints{OrientationConstraint: []motionplan.OrientationConstraint{
				{OrientationToleranceDegs: 60},
			}}
		}},
		{"orient_90", func() *motionplan.Constraints {
			return &motionplan.Constraints{OrientationConstraint: []motionplan.OrientationConstraint{
				{OrientationToleranceDegs: 90},
			}}
		}},
		{"orient_120", func() *motionplan.Constraints {
			return &motionplan.Constraints{OrientationConstraint: []motionplan.OrientationConstraint{
				{OrientationToleranceDegs: 120},
			}}
		}},
		{"combined_200_45", func() *motionplan.Constraints {
			return &motionplan.Constraints{LinearConstraint: []motionplan.LinearConstraint{
				{LineToleranceMm: 200, OrientationToleranceDegs: 45},
			}}
		}},
		{"combined_200_90", func() *motionplan.Constraints {
			return &motionplan.Constraints{LinearConstraint: []motionplan.LinearConstraint{
				{LineToleranceMm: 200, OrientationToleranceDegs: 90},
			}}
		}},
	}

	// Build the FrameSystem ONCE — same as runScenario does.
	fs, err := buildFrameSystem(ctx, r)
	if err != nil {
		return nil, fmt.Errorf("build frame system: %w", err)
	}

	results := []map[string]any{}
	for _, armName := range armNames {
		// Identity-orientation goal at the anchor, in world coords.
		goal := applyArmOffset(r.armBase(armName), anchor)
		for _, spec := range specs {
			start := time.Now()
			_, err := planSingleArmToPose(ctx, r, fs, armName, goal, nil, spec.make())
			dur := time.Since(start)
			result := map[string]any{
				"arm":        armName,
				"constraint": spec.name,
				"ms":         dur.Milliseconds(),
			}
			if err != nil {
				result["ok"] = false
				errStr := err.Error()
				if len(errStr) > 120 {
					errStr = errStr[:120] + "..."
				}
				result["err"] = errStr
			} else {
				result["ok"] = true
			}
			results = append(results, result)
		}
	}

	return map[string]any{
		"results":     results,
		"anchor_used": []float64{anchor.X, anchor.Y, anchor.Z},
	}, nil
}
