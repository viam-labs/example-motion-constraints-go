package motionconstraints

import (
	"context"
	"math"
	"time"

	"github.com/golang/geo/r3"
	pb "go.viam.com/api/service/worldstatestore/v1"
	"go.viam.com/rdk/services/worldstatestore"
	"go.viam.com/rdk/spatialmath"
)

// Path field-mask constants. CamelCase because the renderer ignores the
// snake_case form silently (per the sibling example-visualizations-go's
// renderer-side gotchas).
const (
	pathPoseX = "poseInObserverFrame.pose.x"
	pathPoseY = "poseInObserverFrame.pose.y"
	pathPoseZ = "poseInObserverFrame.pose.z"
)

// obstacleAnimation describes how a scenario obstacle moves over time.
// Only oscillate-along-a-line is supported in Phase 8 — that's enough to
// turn the dynamic_obstacle preset's box into a continuously moving target.
type obstacleAnimation struct {
	// AnchorA / AnchorB are the two world-frame endpoints to oscillate
	// between. The geometry's stored pose is reset each tick from these.
	AnchorA spatialmath.Pose
	AnchorB spatialmath.Pose

	// PeriodS is the time in seconds for one full A → B → A oscillation.
	PeriodS float64

	// PhaseOffsetS shifts the start of the cycle so multiple animated
	// obstacles can move out of phase.
	PhaseOffsetS float64
}

// animState is the runtime view of one animated obstacle. The service
// holds these in a slice and re-emits an UPDATED transform with new
// pose-axis field-mask paths each tick.
type animState struct {
	uuid []byte
	anim obstacleAnimation
}

// animationLoop runs at animTickHz and re-emits UPDATED poses for every
// currently-animated obstacle. Stopped via context cancel.
func (s *service) animationLoop(ctx context.Context, done chan struct{}) {
	defer close(done)
	const animTickHz = 20.0
	ticker := time.NewTicker(time.Duration(float64(time.Second) / animTickHz))
	defer ticker.Stop()
	start := time.Now()
	for {
		select {
		case <-ctx.Done():
			return
		case now := <-ticker.C:
			s.advanceAnimations(now.Sub(start).Seconds())
		}
	}
}

// advanceAnimations recomputes each animated obstacle's pose and emits an
// UPDATED transform with the changed axes in the field-mask.
func (s *service) advanceAnimations(elapsedS float64) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if len(s.animations) == 0 {
		return
	}
	for _, a := range s.animations {
		tf, ok := s.scene[string(a.uuid)]
		if !ok {
			continue
		}
		newPose := lerpAnchors(a.anim, elapsedS)
		pt := newPose.Point()
		tf.PoseInObserverFrame.Pose.X = pt.X
		tf.PoseInObserverFrame.Pose.Y = pt.Y
		tf.PoseInObserverFrame.Pose.Z = pt.Z
		s.broadcastLocked(worldstatestore.TransformChange{
			ChangeType:    pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_UPDATED,
			Transform:     tf,
			UpdatedFields: []string{pathPoseX, pathPoseY, pathPoseZ},
		})
	}
}

// lerpAnchors returns a pose along the line between AnchorA and AnchorB,
// oscillating sinusoidally at the configured period.
func lerpAnchors(a obstacleAnimation, elapsedS float64) spatialmath.Pose {
	period := a.PeriodS
	if period <= 0 {
		period = 4.0
	}
	t := elapsedS + a.PhaseOffsetS
	w := 0.5 * (1 + math.Sin(2*math.Pi*t/period))
	pa := a.AnchorA.Point()
	pb := a.AnchorB.Point()
	mixed := r3.Vector{
		X: pa.X + w*(pb.X-pa.X),
		Y: pa.Y + w*(pb.Y-pa.Y),
		Z: pa.Z + w*(pb.Z-pa.Z),
	}
	return spatialmath.NewPoseFromPoint(mixed)
}

// registerAnimation declares an obstacle as animated. Called by a
// scenario's Plan/Setup hook (typically Setup) for any obstacle that
// should move continuously between scenario iterations.
func (s *service) registerAnimation(uuid []byte, anim obstacleAnimation) {
	s.mu.Lock()
	defer s.mu.Unlock()
	for i := range s.animations {
		if string(s.animations[i].uuid) == string(uuid) {
			s.animations[i] = animState{uuid: uuid, anim: anim}
			return
		}
	}
	s.animations = append(s.animations, animState{uuid: uuid, anim: anim})
}

// clearAnimations removes every animation state. Called on Reconfigure so
// stale animations from a prior config don't keep ticking.
func (s *service) clearAnimations() {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.animations = nil
}
