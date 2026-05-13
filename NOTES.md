# NOTES.md — issues, open questions, RDK bugs to file

This is a living document. Append to it as we hit things; promote to RDK issues or README items once resolved.

## Resolved questions (Phase 3 spike — 2026-05-13)

### OQ1 — How do we get the planned trajectory for preview? **RESOLVED**

Use `go.viam.com/rdk/motionplan/armplanning.PlanMotion(ctx, logger, *PlanRequest) (motionplan.Plan, *PlanMeta, error)` directly. The returned `Plan` exposes:

- `.Path() Path` — `[]referenceframe.FrameSystemPoses`, the cartesian poses per frame at each waypoint. This is what feeds the ghost-trajectory rendering.
- `.Trajectory() Trajectory` — `[]referenceframe.FrameSystemInputs`, joint positions per frame at each waypoint. Pass `.Trajectory().GetFrameInputs(armName)` straight into `arm.MoveThroughJointPositions` to execute.

The motion service's `Move` is not the right API for preview — `armplanning.PlanMotion` is. Build the FrameSystem from the configured arms' `Kinematics()` outputs plus static offset frames for their world positions.

**Caveat:** the default `PlannerOptions` returns a sparse trajectory — in the spike, a "go to (600,0,400)" plan came back with **only 2 waypoints** (start + goal). For meaningful preview rendering we need either dense intermediate sampling from the planner (investigate `PlannerOptions.SmoothIter` and related knobs) or post-hoc cartesian interpolation between the sparse cartesian waypoints. **Tracked as OQ4 below.**

### OQ2 — Does motion planning auto-include sibling arms as collision obstacles? **RESOLVED — NO**

Even when both arms' kinematic models are attached to the same `FrameSystem` passed into `PlanMotion`, the planner does **not** treat the non-moving sibling arm's link geometries as obstacles when planning for the moving arm. Empirical evidence from the spike:

- Set up arm A at world origin, arm B at (600, 0, 0), both ur5e.
- Asked PlanMotion to drive arm A's tool to (600, 0, 0, height=400) — directly above arm B's base, in clear collision with arm B's volume.
- Plan succeeded immediately (~40ms). All 2 trajectory steps had `arm_a:upper_arm_link` colliding with `arm_b:forearm_link` etc.

Even more interesting: when we ALSO inject arm B's link geometries into `WorldState.GeometriesInFrame` (with renamed labels to avoid a "multiple geometries with same name" duplicate-detection error), the plan **still completes through B's volume.** Either the default planner is doing surface-level checking, or `Plan.Trajectory()` returns post-smoothing keyframes that don't represent the actual searched configurations.

**Implication for scenarios:** we cannot rely on motion planning alone to avoid inter-arm collision. The module MUST run an independent `motionplan.CheckCollisions` (or per-step pairwise check using `Geometry.CollidesWith`) on the trajectory and visualize the result. This makes the red-tint highlight a load-bearing educational feature, not just a polish item.

### OQ3 — Does `metadata.colors` field-mask UPDATE re-paint in the 3D viewer?

Not yet resolved. Will be answered empirically during Phase 6 collision-detection work — stable UUID UPDATE first, fall back to versioned-UUID re-add if the viewer doesn't repaint.

## Open questions

### OQ4 — How to get dense trajectory samples for preview rendering?

The default `armplanning.PlannerOptions` returns sparse trajectories (2 waypoints for a straight-line plan). For a ghost-trajectory preview to look continuous in the 3D viewer, we need either:

- **(a)** Configure `PlannerOptions` to produce denser sampling (investigate `SmoothIter`, `MaxIterations`, or any waypoint-density knob).
- **(b)** Post-hoc interpolate between the sparse cartesian poses in `Plan.Path()` using `spatialmath.Interpolate`. Cheap; loses fidelity if the planner inserted curves the cartesian interpolation can't see.
- **(c)** Replay the trajectory through `FrameSystem.Transform` step-by-step with manually-densified joint inputs.

**Decision target:** Phase 5 (visualization layer). (a) is the cleanest if a knob exists; (b) is the fallback.

### OQ5 — Why does `WorldState.GeometriesInFrame` not prevent the goal from being inside an obstacle?

Surprising result from the OQ2 verification: even injecting arm B's link geometries into `WorldState.GeometriesInFrame` (with renamed labels) did not cause the planner to reject the path. Two hypotheses worth investigating:

- The default planner only checks collision against the *moving frame's* chain, not against the goal-pose configuration's link placement.
- `Plan.Trajectory()` returns only smoothed keyframes; the planner's internal search saw collisions but the final returned plan landed on the smoothed approximation.

**Decision target:** Phase 6, before red-tint logic is finalized. Independent collision validation is required regardless of the answer.

## Customer-facing edge cases

_Reserved — populate as we discover them while building scenarios._

## Phase 4 (single_arm_obstacle) — assumptions to verify on first real-machine run

- `framesystem.NewFromService(ctx, fsSvc, nil)` returns a `*FrameSystem` whose frame name for each arm component matches the component's name in machine config (we plan via `armName := r.armOrder[0]` and use the same string in `FrameSystemPoses`). Confirmed by inspection of motion service builtin code; not yet exercised end-to-end.
- `framesystem.FromDependencies(deps)` resolves the auto-injected `$framesystem` service for modules. The fact that `Validate` returns `framesystem.PublicServiceName.String()` as a required dep should sequence the framework correctly.
- `arm.MoveThroughJointPositions` on a fake arm teleports through configurations one by one — fine for a demo. Real arms will want `arm.MoveOptions` blending settings.

## RDK bugs to file

_Reserved — populate as we identify issues that should go upstream._

## Improvements deferred

_Reserved — features we noticed but didn't ship yet._
