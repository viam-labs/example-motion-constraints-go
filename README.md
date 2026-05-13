# example-motion-constraints-go

A Viam module that exercises the motion service against a grid of simulated arms — collision avoidance, linear/orientation constraints, dynamic obstacles, and multi-arm interaction. Plans are previewed as ghost trajectories in the 3D scene viewer; geometries turn **red** when collisions are detected.

Initial purpose: internal testbed for "what works / what doesn't" in motion planning. Long-term purpose: customer-facing instructional examples.

## Status

Pre-release. APIs, config keys, and DoCommand verbs may change without notice until 0.1.0. See [NOTES.md](NOTES.md) for open questions and known issues.

## Model

| API | Model |
| --- | --- |
| `rdk:service:world_state_store` | `viam:example-motion-constraints-go:planner` |

The service is registered as a `world_state_store` because it both **publishes scene primitives** (trajectories, obstacles, collision tints) to the 3D viewer and **orchestrates the arms** behind a single machine-config block. Rationale documented in [CLAUDE.md](CLAUDE.md).

## Quickstart

1. Add the module via the Viam registry: `viam:example-motion-constraints-go`.
2. Use the [`examples/single-arm-demo.json`](examples/single-arm-demo.json) config to get started with one arm and one obstacle.
3. For the full demo grid, use [`examples/grid-of-arms.json`](examples/grid-of-arms.json) (2×2 mixed-model grid: ur5e, ur7e, xarm6, xarm7).
4. Open the 3D scene viewer in the Viam app to watch scenarios execute.

## Configuration

The service config lives under the service's `attributes` block:

| Key | Type | Default | Description |
| --- | --- | --- | --- |
| `motion_service` | string | _required_ | Name of the builtin motion service (`rdk:builtin:builtin`). |
| `arms` | `[]string` | _required_ | Names of arm components to orchestrate. |
| `loop` | bool | `true` | If true, scenarios cycle indefinitely; if false, the module idles until DoCommand. |
| `interval_s` | float | `3.0` | Pause between scenarios in loop mode. |
| `presets` | `[]string` | `["single_arm_obstacle"]` | Built-in scenario keys to run in order. |
| `scenarios` | `[]Scenario` | `[]` | Custom scenario definitions (see below). |
| `abort_on_collision` | bool | `true` | If a pre-flight collision check fails, skip execution. |
| `tick_hz` | float | `30` | Visualization tick rate (capped at 30). |

## Built-in scenarios

| Key | What it shows |
| --- | --- |
| `single_arm_obstacle` | One arm plans around one fixed box. |
| `linear_constraint` | Same start/goal twice — once unconstrained, once with a tight straight-line tolerance. Both trajectories rendered side-by-side. |
| `orientation_constraint` | EE moves through a path while holding orientation fixed (pouring motion). |
| `dynamic_obstacle` | A box oscillates across the planned path; the arm must replan mid-execution. |
| `multi_arm_choreography` | 2×2 grid of mixed-model arms reach toward a shared center, treating each other as obstacles. |

## DoCommand verbs

```jsonc
{ "command": "run",   "scenario": "single_arm_obstacle" }   // run a specific preset now
{ "command": "pause" }                                       // stop the loop
{ "command": "next" }                                        // skip to next scenario
{ "command": "clear" }                                       // remove all scene entities
{ "command": "list" }                                        // returns the list of preset/scenario keys
```

## Visual conventions

| Color | Meaning |
| --- | --- |
| Blue (`{r:80,g:80,b:200}`) | Default obstacle. |
| Green at 40% opacity (`{r:0,g:200,b:120}`) | Preview-trajectory ghost geometry. |
| Red (`{r:255,g:0,b:0}`) | A collision was detected involving this entity. |

## Development

```bash
make            # build the module binary
make test       # run unit tests
make module.tar.gz   # package for upload
make upload PLATFORM=linux/amd64    # upload one platform
make upload-all                      # upload all supported platforms
```

Bump [`VERSION`](VERSION) before each upload. See [CLAUDE.md](CLAUDE.md) for the full release checklist.

## Related modules

- [`viam-labs/example-visualizations-go`](https://github.com/viam-labs/example-visualizations-go) — the structural template this module borrows from. Showcases every supported geometry primitive in the 3D scene viewer.
