# example-motion-constraints-go

An educational Viam module that demonstrates motion-planning concepts — task-space goals, end-effector control frames, linear/orientation constraints, collision avoidance, and multi-arm coordination — on a configurable grid of simulated arms. Each arm runs a scripted scenario in its own goroutine, the planner's trajectory is previewed as a green ghost trail in the 3D scene viewer, and any geometry the planned trajectory intersects is tinted **red**.

Each arm is labeled by a small 3D plaque under its base listing what it's demonstrating (the EE control mode, whether a constraint is active, what the planner considers collidable).

## Example service `attributes`

```jsonc
{
  "attributes": {
    "arms": ["arm_a1", "arm_a2", "arm_a3", "arm_a4"],
    "preset_set": "ee_only",
    "ee_frames": {
      "arm_a3": "gripper_a3",
      "arm_a4": "gripper_a4"
    },
    "loop": true,
    "interval_s": 4,
    "preview_density": 4
  }
}
```

Drop into a `rdk:service:world_state_store` block whose `model` is `viam:example-motion-constraints-go:motion-playground`. Each entry in `arms` must reference a real `rdk:component:arm` declared in the same machine config; entries in `ee_frames` point at child gripper frames attached to those arms (used as the planner's tool tip).

## Model

| API | Model |
| --- | --- |
| `rdk:service:world_state_store` | `viam:example-motion-constraints-go:motion-playground` |

A `world_state_store` service that also orchestrates the configured arms behind one service block. Streams scene primitives to the 3D viewer (its native purpose) and runs scenarios on a background per-arm goroutine.

## Quickstart

1. Add the module via the Viam registry: `viam:example-motion-constraints-go`.
2. Use the [`examples/grid-of-arms.json`](examples/grid-of-arms.json) config as a starting point — 4 ur5e arms at the corners of a 2×2 square running the default `ee_only` preset bundle.
3. Open the 3D scene viewer in the Viam app to watch scenarios execute.

## Attributes

The service config lives under `attributes`:

| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `arms` | `[]string` | **required** | Names of `rdk:component:arm` resources to orchestrate. |
| `preset_set` | string | `"ee_only"` | Named bundle of (arm → preset) mappings to activate. See **Preset bundles** below. Mutually exclusive with `arm_scenarios`. |
| `arm_scenarios` | `{arm: preset}` | unset | Explicit per-arm scenario binding. Overrides `preset_set` when set. |
| `ee_frames` | `{arm: frame}` | unset | Per-arm override of the planner's tool frame. Points at a gripper child frame attached to the arm in the machine config; planning solves for that frame's pose rather than the arm wrist. |
| `loop` | bool | `true` | If true, scenarios cycle indefinitely; if false, the module idles until a `run` DoCommand. |
| `interval_s` | float | `5.0` | Pause in seconds between scenario iterations on a given arm. |
| `preview_density` | int | `2` | Interpolated joint samples per planner waypoint pair when rendering the ghost trajectory. Higher = smoother trail at higher render cost. Set to 1 for keyframes-only. |
| `max_preview_ghosts` | int | `24` | Hard cap on the number of trajectory-ghost spheres emitted per plan, regardless of trajectory length. Down-samples evenly to this count. Prevents linear-constrained plans (cbirrt returns 100+ waypoints to verify the constraint) from emitting a TransformChange burst that locks the 3D viewer's JS main thread. Set to `-1` for uncapped (legacy behavior). |
| `disable_preview_ghosts` | bool | `false` | Diagnostic kill-switch: skip ghost emission entirely. Axes markers + goal marker still emit. |
| `abort_on_collision` | bool | `true` | If the trajectory's pre-flight collision check finds a hit, skip the execute step (leave the trajectory + red-tinted obstacle on screen). |
| `tick_hz` | float | `30` | Visualization tick rate (capped at 30). |
| `max_concurrent_plans` | int | `2` | Ceiling on simultaneous `PlanMotion` calls across all arms. The cbirrt planner spawns ~`NumCPU/2` worker goroutines per call; without a cap, N arms planning in parallel saturate viam-server's Go runtime and starve the WebRTC stream that feeds the 3D scene viewer. Lower = smoother viz; higher = more arm parallelism. |
| `motion_service` | string | _optional_ | Name of a motion service. Currently unused — planning uses `motionplan/armplanning.PlanMotion` directly. |

### Arm-component requirement

Use `rdk:builtin:simulated` (not `:fake`) with `"simulate-time": true`. The fake arm teleports instantly; the simulated arm animates over wall clock — but only when the time-simulation goroutine is enabled.

```jsonc
{
  "name": "arm_a1",
  "api": "rdk:component:arm",
  "model": "rdk:builtin:simulated",
  "attributes": { "arm-model": "ur5e", "simulate-time": true },
  "frame": { "parent": "world", "translation": { "x": 1000, "y": 1000, "z": 0 } }
}
```

## Preset bundles

Set `preset_set` to one of these to swap the scenario assignment on the same four `arm_a1..a4` slots. Every bundle is exactly four arms so the CPU + browser cost stays predictable across swaps.

| Bundle | Description | Scenarios assigned to (a1, a2, a3, a4) |
| --- | --- | --- |
| `ee_only` (default) | End-Effector Control Frame Variations | `random_translation`, `random_rotation`, `random_translation` (gripper), `random_rotation` (gripper) |
| `ee_variations` | Constraint Type Comparison | `ee_baseline` (a1, no constraint), `ee_linear` (a2, LinearConstraint), `ee_orient` (a3, OrientationConstraint), `ee_combined` (a4, both). All four run the SAME 2-anchor swing so the only visible difference is the constraint. See **Constraint variations and their known issues** below for the warts on each. |
| `obstacle_geometry` | Obstacle Geometry Variations | `arc_over_obstacle`, `duck_under_obstacle`, `gripper_with_box`, `corridor_passthrough` |
| `constraint_types` | Constraint and Dynamic-Obstacle Variations | `linear_constraint`, `orientation_constraint`, `dynamic_obstacle`, `single_arm_obstacle` |

Bundles assume the canonical layout: `arm_a1` and `arm_a2` are gripperless; `arm_a3` and `arm_a4` have offset grippers declared in `ee_frames`. Scenarios in `a3`/`a4` slots that target a gripper tip work because of that. Arms in the bundle that aren't declared in the machine config are silently skipped.

## Scenarios

Each scenario is a per-arm cycle: setup obstacles → plan a motion → preview the trajectory → check collisions → execute → wait `interval_s`.

### Task-space pedagogy (row A / AB)

These scenarios isolate the EE-control-frame lever — same arm + same workspace, different combinations of varying-position vs varying-orientation and arm-wrist vs gripper-tip targets.

| Preset | What it demonstrates |
| --- | --- |
| `random_translation` | EE position visits a 7-waypoint sequence. Orientation held at identity. |
| `random_rotation` | EE position held at `(500, 0, 400)` arm-local; orientation tilts/twists through 8 variations. |
| `random_translation_linear` | Same as `random_translation` plus a `LinearConstraint` — each hop traces a straight cartesian line. |
| `random_rotation_linear` | Same as `random_rotation` plus a (loose) `LinearConstraint`. |

When paired via `ee_frames` with a child gripper frame, the planner solves for the *gripper tip*'s pose rather than the wrist's — the trajectory preview moves with the offset, and the wrist traces a different path (visible side-by-side with the no-gripper variant).

### Obstacle geometry (row B)

| Preset | What it demonstrates |
| --- | --- |
| `arc_over_obstacle` | Wide low box between anchors — arm arcs over the top. |
| `duck_under_obstacle` | Mirror image: high box, low anchors — arm ducks underneath. |
| `gripper_with_box` | Arc-over geometry, but the gripper itself carries a long collision geometry. The tool's footprint forces a different trajectory than the wrist alone would. |
| `corridor_passthrough` | Two walls form a narrow corridor — the only feasible trajectory threads the gap. |

### Constraint and dynamic-obstacle (row C)

| Preset | What it demonstrates |
| --- | --- |
| `linear_constraint` | Hold the EE on a straight line between anchors with a loose tolerance. |
| `orientation_constraint` | Keep the EE orientation within 45° while swinging between anchors. |
| `dynamic_obstacle` | A box oscillates continuously across the workspace; the arm's planned trajectory is computed against the obstacle's pose at the moment of planning. |

### Constraint variations and their known issues

The `ee_variations` bundle puts each arm under a different constraint over the same 2-anchor swing, so you can compare them side-by-side. Each arm has a different gripper offset, declared via `ee_frames`. Known warts:

| Arm | Constraint | What it does | Known issues |
| --- | --- | --- | --- |
| `arm_a1` | None (baseline) | cbirrt picks a natural path between anchors — generally a curve in cartesian space because joint-space shortest path doesn't map to cartesian-space shortest. | None — this is the reference. |
| `arm_a2` | `LinearConstraint{LineToleranceMm: 200, OrientationToleranceDegs: 180}` | EE position stays inside a 200mm tube along the straight line between anchors. Orientation is unconstrained along the path. | **Motion length matters far more than tolerance values.** The planner internally generates an intermediate waypoint every 10mm of cartesian motion under a LinearConstraint (see `motionplan/armplanning/plan_manager.go::generateWaypoints` — `defaultStepSizeMM=10`), and each waypoint requires its own IK + cbirrt sub-plan in the total plan budget. A 400mm swing → 40 sub-plans (~75ms each, often times out). A 200mm swing → 20 sub-plans (~150ms each, converges reliably). The tolerance values themselves don't change this step count — they only adjust step size when extremely tight. Also: tight `LineToleranceMm` (<100mm) + offset gripper frame fails frequently because the wrist must trace a parallel line at the gripper offset. Trajectories also come back ~5–10× denser than unconstrained, which is why the module caps preview-ghost emissions via `max_preview_ghosts`. |
| `arm_a3` | `OrientationConstraint{OrientationToleranceDegs: 45}` | EE orientation stays within 45° of the interpolated path orientation. Position is free to take any path between the anchors. | Tight orientation tolerance (<30°) interacts badly with start/end orientation mismatches — if the home pose's wrist orientation differs much from what's natural at the goal, cbirrt cannot find a path that smoothly interpolates and stays within tolerance. Identity-orientation goals are particularly unreliable with offset grippers (same IK-reachability issue as Linear). |
| `arm_a4` | Combined: `LinearConstraint{200mm, 30deg}` | Both: position inside a 200mm tube AND orientation within 30° along the path. | Hardest of the four. Multiplicative restriction on feasible joint configurations. Frequently fails when either individual constraint is borderline. Showcases that constraint stacking is rarely additive in difficulty — it's multiplicative. Watch for `cbirrt timeout` in `stats` `last_error`. |

The pedagogical pattern: a1 → a2 → a3 → a4 goes from "easy to plan, doesn't show much" to "shows a lot, often fails." That's not a bug — it's the actual trade-off of using constraints in production motion planning.

### Coordinated and progressive

| Preset | What it demonstrates |
| --- | --- |
| `multi_arm_choreography` | Each configured arm swings between anchors; sibling arms are injected into the world state as obstacles per plan call, so colliding plans get aborted and red-tinted. |
| `obstacle_progression` | Same anchors; obstacles accumulate each cycle (box → box+floor → box+floor+ceiling). |
| `single_arm_obstacle` | Single arm swinging around one static box — the simplest demonstration scenario. |

## DoCommand verbs

```jsonc
{ "command": "list" }                                  // catalog of preset keys + named bundles
{ "command": "status" }                                // basic config status
{ "command": "stats" }                                 // per-arm cycles, stages, errors, goroutine count
{ "command": "run",   "scenario": "single_arm_obstacle" }  // run a preset once (legacy mode only)
{ "command": "pause" }                                 // freeze scenario loop
{ "command": "resume" }                                // resume the loop
{ "command": "next" }                                  // skip the inter-scenario sleep
{ "command": "clear" }                                 // remove every scene entity
```

## Visual conventions

| Color | Meaning |
| --- | --- |
| Blue (`{r:80, g:80, b:200}`) | Default obstacle. |
| Green at 40% opacity (`{r:0, g:200, b:120}`) | Trajectory ghost waypoint. |
| Gold (`{r:230, g:180, b:0}`) | Goal marker at the trajectory's final pose. |
| Red (`{r:255, g:0, b:0}`) | A collision was detected involving this entity. |
| Coordinate-axes triad | Reference frame at trajectory start, end, and (on long trails) intermediates. |
| Near-black 4-line plaque | Per-arm scenario description, suspended below each arm. |

## Development

```bash
make             # build the module binary
make test        # run unit tests
make assets      # regenerate text-PLY label assets via Python (see scripts/generate_text_assets.py)
make module.tar.gz   # package for upload
make upload PLATFORM=linux/amd64    # upload to registry
```

Bump [`VERSION`](VERSION) before each `viam module upload`. See [CLAUDE.md](CLAUDE.md) for the full release checklist.

## Related modules

- [`viam-labs/example-visualizations-go`](https://github.com/viam-labs/example-visualizations-go) — structural template for this module's scene-publishing code; showcases every supported geometry primitive in the 3D viewer.
