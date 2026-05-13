// Package motionconstraints implements a Viam world_state_store service that
// orchestrates scripted motion-planning scenarios on a configurable grid of
// simulated arms. It publishes planned trajectories, obstacles, and collision
// state to the Viam 3D scene viewer.
//
// Status: Phase 1 skeleton. The service registers and constructs cleanly but
// emits no scene primitives and runs no scenarios yet. See CLAUDE.md and
// NOTES.md for the implementation plan.
package motionconstraints

import (
	"context"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/worldstatestore"
)

// Model is the resource model registered by this module.
var Model = resource.NewModel("viam", "example-motion-constraints-go", "planner")

func init() {
	resource.RegisterService(worldstatestore.API, Model,
		resource.Registration[worldstatestore.Service, *Config]{
			Constructor: newService,
		},
	)
}

// service is the world_state_store implementation. State, subscribers, the
// scenario loop, and motion-planning helpers will hang off this struct as
// the later phases land.
type service struct {
	resource.Named
	resource.TriviallyCloseable
	resource.TriviallyReconfigurable

	logger logging.Logger
	cfg    *Config
}

func newService(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (worldstatestore.Service, error) {
	parsed, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}
	logger.Infow("example-motion-constraints-go skeleton constructed",
		"name", conf.ResourceName().Name,
		"arms", parsed.Arms,
		"motion_service", parsed.MotionService,
	)
	return &service{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
		cfg:    parsed,
	}, nil
}

// ListUUIDs returns the UUIDs of all transforms currently being published.
// Phase 1 skeleton publishes nothing.
func (s *service) ListUUIDs(ctx context.Context, extra map[string]any) ([][]byte, error) {
	return nil, nil
}

// GetTransform returns the transform for a given UUID. Phase 1 skeleton has no
// transforms to return.
func (s *service) GetTransform(
	ctx context.Context,
	uuid []byte,
	extra map[string]any,
) (*commonpb.Transform, error) {
	return nil, nil
}

// StreamTransformChanges streams add/update/remove events to a subscriber.
// Phase 1 skeleton returns a stream that emits nothing and closes when the
// caller's context is canceled.
func (s *service) StreamTransformChanges(
	ctx context.Context,
	extra map[string]any,
) (*worldstatestore.TransformChangeStream, error) {
	ch := make(chan worldstatestore.TransformChange)
	go func() {
		<-ctx.Done()
		close(ch)
	}()
	return worldstatestore.NewTransformChangeStreamFromChannel(ctx, ch), nil
}

// DoCommand is the manual-control surface for scenarios. Verbs are wired up
// in Phase 2 — list, run, pause, next, clear.
func (s *service) DoCommand(
	ctx context.Context,
	cmd map[string]any,
) (map[string]any, error) {
	return map[string]any{
		"status":  "skeleton",
		"phase":   "1",
		"message": "module loaded; no scenarios implemented yet",
	}, nil
}
