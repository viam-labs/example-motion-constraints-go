// Package motionconstraints implements a Viam world_state_store service that
// orchestrates scripted motion-planning scenarios on a configurable grid of
// simulated arms. It publishes planned trajectories, obstacles, and collision
// state to the Viam 3D scene viewer.
//
// Status: Phase 2 — runtime skeleton. The service constructs, reconfigures,
// fans out an empty stream to subscribers, runs a no-op tick goroutine, and
// responds to a small set of DoCommand verbs. No scenarios are implemented
// yet; the preset keys are exposed via `list` so callers can see the planned
// catalog. See NOTES.md OQ1/OQ2 for the questions Phase 3 will answer.
package motionconstraints

import (
	"context"
	"sort"
	"sync"
	"time"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/worldstatestore"
)

// Model is the resource model registered by this module.
var Model = resource.NewModel("viam", "example-motion-constraints-go", "planner")

// Defaults applied when the corresponding config field is absent.
const (
	DefaultIntervalS  = 3.0
	DefaultTickHz     = 30.0
	maxTickHz         = 30.0
	subscriberBufSize = 256
)

// Built-in preset keys. The implementations land in Phase 4 (single arm) and
// Phase 7 (the rest). They're enumerated here so DoCommand `list` can report
// the planned catalog and validate user input before scenarios exist.
var builtinPresets = []string{
	"single_arm_obstacle",
	"linear_constraint",
	"orientation_constraint",
	"dynamic_obstacle",
	"multi_arm_choreography",
}

func init() {
	resource.RegisterService(worldstatestore.API, Model,
		resource.Registration[worldstatestore.Service, *Config]{
			Constructor: newService,
		},
	)
}

// service is the world_state_store implementation. State, subscribers, and
// the tick goroutine live here; the motion-planning helpers will hang off
// this struct as later phases land.
type service struct {
	resource.Named

	logger logging.Logger

	mu  sync.Mutex
	cfg *Config

	subscribers []chan worldstatestore.TransformChange

	// Tick goroutine control. Each Reconfigure cancels the prior tick and
	// (if loop mode is enabled) starts a fresh one.
	tickCancel context.CancelFunc
	tickDone   chan struct{}

	// Resolved dependencies. Empty until Phase 4 wires actual usage.
	armNames      []string
	motionService string

	// Resolved tick rate. Capped at maxTickHz.
	tickHz float64

	// Loop control. When paused, the tick still runs but does not advance
	// the scenario cursor. Phase 4+ behavior; in Phase 2 the tick is a no-op.
	loop   bool
	paused bool
}

func newService(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (worldstatestore.Service, error) {
	s := &service{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
		tickHz: DefaultTickHz,
		loop:   true,
	}
	// Go's module.ModularMain does not auto-call Reconfigure for services,
	// so we trigger it explicitly. This matches the sibling
	// example-visualizations-go module's quirk.
	if err := s.Reconfigure(ctx, deps, conf); err != nil {
		return nil, err
	}
	return s, nil
}

// Reconfigure (re)parses the config, restarts the tick goroutine, and notifies
// existing subscribers of the new world. In Phase 2 the "new world" is empty
// so no transform changes are emitted; in Phase 4+ this is where REMOVED for
// the prior scenario's entities + ADDED for the new world will fire.
func (s *service) Reconfigure(
	ctx context.Context,
	_ resource.Dependencies,
	conf resource.Config,
) error {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return err
	}

	s.mu.Lock()
	prevCancel := s.tickCancel
	prevDone := s.tickDone
	s.cfg = cfg
	s.armNames = append(s.armNames[:0], cfg.Arms...)
	s.motionService = cfg.MotionService

	if cfg.TickHz > 0 {
		s.tickHz = cfg.TickHz
	} else {
		s.tickHz = DefaultTickHz
	}
	if s.tickHz > maxTickHz {
		s.tickHz = maxTickHz
	}

	if cfg.Loop != nil {
		s.loop = *cfg.Loop
	} else {
		s.loop = true
	}
	s.paused = false
	s.mu.Unlock()

	// Stop the prior tick outside the lock so a tick goroutine that is mid-
	// broadcast can drain without deadlocking.
	if prevCancel != nil {
		prevCancel()
		<-prevDone
	}

	tickCtx, cancel := context.WithCancel(context.Background())
	done := make(chan struct{})

	s.mu.Lock()
	s.tickCancel = cancel
	s.tickDone = done
	s.mu.Unlock()

	go s.runTick(tickCtx, done)

	s.logger.Infow("example-motion-constraints-go (re)configured",
		"name", conf.ResourceName().Name,
		"arms", s.armNames,
		"motion_service", s.motionService,
		"tick_hz", s.tickHz,
		"loop", s.loop,
		"presets", cfg.Presets,
	)
	return nil
}

// Close stops the tick goroutine and tears down all active subscribers.
func (s *service) Close(ctx context.Context) error {
	s.mu.Lock()
	cancel := s.tickCancel
	done := s.tickDone
	subs := s.subscribers
	s.subscribers = nil
	s.mu.Unlock()

	if cancel != nil {
		cancel()
		<-done
	}
	for _, ch := range subs {
		close(ch)
	}
	return nil
}

// runTick is the scenario-driver loop. Phase 2 is a no-op heartbeat that
// logs once per minute so we can confirm the goroutine survives reconfigures.
// Phase 4+ replaces the body with the scenario state machine.
func (s *service) runTick(ctx context.Context, done chan struct{}) {
	defer close(done)
	heartbeat := time.NewTicker(60 * time.Second)
	defer heartbeat.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-heartbeat.C:
			s.logger.Debugw("tick heartbeat", "phase", "2-skeleton")
		}
	}
}

// ListUUIDs returns the UUIDs of all transforms currently being published.
// Phase 2 publishes nothing.
func (s *service) ListUUIDs(ctx context.Context, extra map[string]any) ([][]byte, error) {
	return nil, nil
}

// GetTransform returns the transform for a given UUID. Phase 2 has none.
func (s *service) GetTransform(
	ctx context.Context,
	uuid []byte,
	extra map[string]any,
) (*commonpb.Transform, error) {
	return nil, nil
}

// StreamTransformChanges registers a subscriber and streams add/update/remove
// events. The Phase 2 implementation establishes the fanout machinery but
// emits nothing until scenarios are wired up. Subscribers automatically
// receive an initial burst of ADDED events for every currently-visible entity
// in Phase 4+.
func (s *service) StreamTransformChanges(
	ctx context.Context,
	extra map[string]any,
) (*worldstatestore.TransformChangeStream, error) {
	ch := make(chan worldstatestore.TransformChange, subscriberBufSize)

	s.mu.Lock()
	s.subscribers = append(s.subscribers, ch)
	s.mu.Unlock()

	// Tear-down goroutine: when the caller's context cancels, remove this
	// subscriber from the fanout list and close its channel.
	go func() {
		<-ctx.Done()
		s.removeSubscriber(ch)
	}()

	return worldstatestore.NewTransformChangeStreamFromChannel(ctx, ch), nil
}

// removeSubscriber removes ch from the subscriber list and closes it.
// Safe to call from any goroutine.
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

// broadcastLocked sends a transform change to every active subscriber with a
// non-blocking select; a full channel logs a warning and drops the event.
// Caller must hold s.mu.
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

// DoCommand exposes runtime control of the scenario loop.
//
// Verbs:
//   - {"command":"list"}                                  → returns the catalog of built-in preset keys
//   - {"command":"status"}                                → returns current loop state (paused, tick_hz, arms, motion_service)
//   - {"command":"pause"}                                 → stops scenario advancement (no-op until scenarios exist)
//   - {"command":"resume"}                                → resumes scenario advancement
//   - {"command":"clear"}                                 → removes every entity currently in the scene
//   - {"command":"run","scenario":"<key>"}                → runs a specific preset once (Phase 4+ implements)
//   - {"command":"next"}                                  → advances to the next scenario in the loop (Phase 4+ implements)
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
			"presets": presets,
			"note":    "scenario implementations land in phase 4+; listing reports the planned catalog",
		}, nil

	case "status":
		s.mu.Lock()
		defer s.mu.Unlock()
		configured := []string{}
		if s.cfg != nil {
			configured = append(configured, s.cfg.Presets...)
		}
		return map[string]any{
			"phase":          "2-skeleton",
			"paused":         s.paused,
			"loop":           s.loop,
			"tick_hz":        s.tickHz,
			"arms":           append([]string{}, s.armNames...),
			"motion_service": s.motionService,
			"presets":        configured,
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
		return map[string]any{"paused": false}, nil

	case "clear":
		// Phase 2 has no scene entities; this returns success so callers can
		// safely script `clear` between runs. Phase 4+ will emit REMOVED
		// changes for every active entity.
		return map[string]any{"cleared": 0}, nil

	case "run", "next":
		return map[string]any{
			"error": "scenarios not implemented yet; see NOTES.md OQ1/OQ2 (Phase 3 spike)",
			"verb":  verb,
		}, nil

	default:
		return map[string]any{
			"error": "unrecognized command; try one of: list, status, pause, resume, clear, run, next",
		}, nil
	}
}
