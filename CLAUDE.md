# CLAUDE.md — operational context for example-motion-constraints-go

This file primes future Claude Code sessions on how to work on this module. Read it before making changes.

## What this is

- **GitHub:** `viam-labs/example-motion-constraints-go`
- **Registry:** `viam:example-motion-constraints-go` (public, visibility set in `meta.json`)
- **Model:** `viam:example-motion-constraints-go:motion-playground`
- **API:** `rdk:service:world_state_store`
- **Current version:** see [`VERSION`](VERSION) — bump before every upload.

An educational motion-planning demo: a single `world_state_store` service that orchestrates a configurable grid of simulated arms, runs a preset bundle of scripted scenarios in parallel per-arm goroutines, and publishes the planner's trajectory + collision state + per-arm scenario labels to the 3D viewer.

## Release checklist — always follow this order

Before every `viam module upload`:

1. **Commit.** Never leave the tree dirty between sessions. The `Always commit` rule below is absolute.
2. **Update [`README.md`](README.md)** if the change adds/removes a preset, DoCommand verb, config knob, attribute, or visible behavior.
3. **Update [`meta.json`](meta.json)** if the model list, description, supported platforms, or entrypoint changed.
4. **Bump [`VERSION`](VERSION)** using semver. Rename / model-deprecation = minor bump (0.2.0). Bug-fix = patch (0.2.1).
5. **`make` + `make test`** — module must build cleanly and unit tests must pass.
6. **`make assets`** if any label strings in `scripts/generate_text_assets.py::LABELS` changed. Regenerates the PLY mesh files under `assets/`.
7. **`make module.tar.gz`** to package; **`viam module upload --version=$(cat VERSION) --platform=linux/amd64 module.tar.gz`** to publish.
8. **Push the commit** so GitHub and the registry stay in sync.

## Always-commit rule

After any meaningful change — new file, new function, doc edit, presets added, bug fixed — commit it before doing the next thing. A series of small commits is much easier to bisect than one mega-commit. If a session ends mid-task with uncommitted work, the next session has no idea what was in flight.

## Architecture rationale

The module registers a single `rdk:service:world_state_store` model. This service does **two** things:

1. **Streams scene primitives** to the 3D viewer (the native purpose of `world_state_store`).
2. **Orchestrates motion** on configured arms via per-arm goroutines + a shared scenario loop.

The streaming side uses per-subscriber non-blocking channels (capacity 256) with overflow-warn-and-drop. The orchestration side runs a goroutine per `(arm, scenario)` pair from the active bundle plus a separate animation tick for dynamic obstacles. Both share a single mutex around the scene state map.

If the orchestration side ever needs its own resource dependencies that don't fit `world_state_store` semantics, the right split is a sibling `rdk:service:generic` model that drives this one via DoCommand.

## Configuration model

Two ways to activate scenarios in the machine config:

- `preset_set` (recommended) — a string naming one of the bundles in `service.go::PresetBundles`. Filters bundle entries against the configured `arms` list, so users can declare a subset of arms and pick a heavy bundle without having to also declare every arm.
- `arm_scenarios` — explicit `{arm: preset}` map. Overrides `preset_set` when set.

Every bundle is exactly four arms — always `arm_a1..a4` — so switching bundles reuses the same machine config and the CPU/render cost stays predictable. Earlier wider bundles (rows AB/B/C, an "all" superset) proved heavier than the renderer could keep up with; the 4-arm constraint keeps every `preset_set` responsive.

Default bundle: `ee_only`. Switch to `ee_variations` to add a `LinearConstraint` to each of the same four scenarios, `obstacle_geometry` for the obstacle-shape pedagogy, or `constraint_types` for the linear/orientation/dynamic constraint variety.

Each preset is a closure in `presets.go` returning a `Scenario` with `Setup` (obstacles) and `Plan` hooks. Per-scenario state (cycle counters, etc.) lives in closure-captured atomics — each `presetByKey` call returns a fresh Scenario value so parallel goroutines don't share counters.

## Per-arm text labels (PLY mesh plaques)

Each arm gets a four-line text plaque emitted under its base describing the scenario:

```
Arm Only | Arm + Gripper
Translation / Rotation
Constraint: None | Linear
Collidables: Self Only | Self + Tool
```

The text strings are pre-generated as PLY mesh assets via `scripts/generate_text_assets.py` (matplotlib + shapely + trimesh + mapbox_earcut). The script lives in this repo; the easiest way to run it is via the sibling Python module's venv:

```
~/viam/example-visualizations-python/.venv/bin/python scripts/generate_text_assets.py
```

(Or any matching venv — same deps as that sibling.)

Workflow when iterating on label text:
1. Edit `LABELS` in `scripts/generate_text_assets.py`.
2. Edit `labelTextForArm` in `assets.go` to match — every key the Go switch returns must have a corresponding LABELS entry.
3. Run `make assets` (or invoke the Python script directly).
4. Rebuild and re-upload.

Asset path resolution at runtime: `resolveModuleDir()` in `assets.go` uses `os.Executable()` — the binary is at `<ModuleDir>/bin/<name>`, so assets are at `<ModuleDir>/assets/`. CWD is NOT reliable; viam-server runs the binary from arbitrary working directories.

## Renderer conventions (gotchas)

Inherited from `example-visualizations-go`'s hard-won learnings. Violate one and the entity is silently invisible.

1. **Field-mask paths are camelCase.** `poseInObserverFrame.pose.x`, not `pose_in_observer_frame.pose.x`. The `path*` constants in `animation.go` are the single source of truth.
2. **Metadata struct must include all five keys**: `colors`, `color_format`, `opacities`, `show_axes_helper`, `invisible`. Omit one → invisible entity. Use the `buildMetadata` helper.
3. **`invisible=true` hides everything including axes helpers.** `emitAxesMarker` uses `invisible=false` with a tiny 3mm placeholder sphere so the triad still renders.
4. **Colors and opacities are base64-encoded byte arrays**, not nested structs.
5. **UUID strategy:** stable UUIDs (`UUID = label`) for entities that persist or get color-updated; versioned UUIDs (rotating per emission, with `ts` suffix) for trajectory ghosts that are added/removed per scenario.
6. **Mesh content type** is lowercase: `ply` or `stl`. The viewer only renders PLY; STL must be converted at load time. Our text labels are always PLY.
7. **PLY vertex coordinates are in METERS.** The RDK reader multiplies by 1000 to get mm.

## Motion-planning conventions

- We plan via `motionplan/armplanning.PlanMotion` directly, not the motion service's `Move`. The motion service doesn't return the planned trajectory, which we need for the ghost preview. See NOTES.md OQ1.
- Sibling arms are NOT auto-included in collision checks by the framesystem alone (Phase 3 spike finding, NOTES.md OQ2). Multi-arm scenarios manually inject each sibling's link geometries into `WorldState.GeometriesInFrame` per plan call.
- `planSingleArmToPose` enforces a 6-second `context.WithTimeout` on every PlanMotion call. Struggling scenarios that always hit the budget burn ~6s of CPU per cycle each, which can saturate the host and starve viam-server's WebRTC stream — keep scenario goals reachable.
- Each arm's planner targets the frame named by `r.eeFrame(armName)`. By default that's the arm's own kinematic-output frame; if the user configured an `ee_frames` entry, the planner solves for that frame's pose (a gripper tip child frame).
- Arms get reset to a known pose at the start of each scenario goroutine via `MoveToJointPositions`: `homeJointPositionsCandle` (j1, j3 = -90deg, folded up) for obstacle scenarios, `homeJointPositionsReady` (j1=-90, j2=+90, j3=-90, forward-pointing) for non-obstacle scenarios. Predicate lives in `service.go::scenarioNeedsHome`. Critical because simulated-arm joint state persists across module reloads in viam-server.

## File layout

| File | Role |
| --- | --- |
| `cmd/module/main.go` | Module entry; registers the motion-playground model. |
| `cmd/spike/main.go` | Throwaway exploratory program for RDK motion-API behavior. Not part of `make`. |
| `service.go` | World-state-store service: lifecycle, subscribers, scenario loop, scene state, DoCommand dispatcher, label emit. |
| `config.go` | JSON config schema; `Validate` declares deps. |
| `scenarios.go` | `Scenario` type + `runScenario` lifecycle (setup → plan → preview → collision → execute → cleanup) + `planSingleArmToPose` helper + `emitDenseTrajectoryGhosts`. |
| `presets.go` | All built-in scenarios + the `alternateBetweenAnchors` helper + scenario-key-to-scenario routing. |
| `arms.go` | `resolved` struct + `resolveDeps` (arms, motion, framesystem) + arm-base caching + home-pose helpers. |
| `geometries.go` | Box/sphere proto builders + `buildMetadata` (the five-key metadata struct). |
| `collision.go` | `checkTrajectoryCollisions` (independent collision validation; the educational red-tint feature). |
| `animation.go` | `animationLoop` + `obstacleAnimation` (oscillating dynamic obstacles) + field-mask `path*` constants. |
| `assets.go` | Module-dir resolution + `loadTextPLY` + `labelTextForArm` (scenario-key → multi-line label string). |
| `scripts/generate_text_assets.py` | Offline label-PLY generator (matplotlib + shapely + trimesh). |
| `assets/text__*.ply` | Pre-generated label meshes shipped with the module. |
| `examples/grid-of-arms.json` | Canonical machine config — 4 arms in a 2×2 square running `ee_only`. |
| `examples/single-arm-demo.json` | Minimal one-arm config. |

## Testing

- Unit tests next to each file (`config_test.go`). Run via `make test`.
- The spike in `cmd/spike/main.go` is **not** a test — it's an exploratory program. Build it with `go run ./cmd/spike` to investigate API behavior; never include it in the module tarball.
- End-to-end validation requires a Viam machine. The canonical smallest config is `examples/grid-of-arms.json`; for stress testing scale up via `arm_scenarios` to include rows B and C.
- **Stats DoCommand verb** is the diagnostic workhorse: `{"command":"stats"}` returns per-arm cycle counts, current scenario stage with age, last error per arm, goroutine count, scene size. Use it to identify stuck arms.

## Perf characteristics

- Healthy scenarios complete in <1s per plan; budget of 6s only matters for genuinely hard problems.
- A struggling scenario (one that always hits the plan budget) pegs a CPU core for that duration. A few struggling arms in parallel saturate the host and starve viam-server's WebRTC stream — every arm appears sluggish, not just the offending ones. See NOTES.md "Perf characteristic" for the full writeup.
- Prefer "small motion + tight tolerance" over "big motion + generous budget" when designing scenarios that need to show off harder problems.
- 16 arms running healthy scenarios is comfortable in the browser. 4–8 is the sweet spot for stable demos.

## Don't

- **Don't add a new scenario without adding a label entry in both `scripts/generate_text_assets.py::LABELS` and `assets.go::labelTextForArm`.** Mismatches mean the runtime falls back to the scenario key with underscores replaced by spaces, asset-loading fails, and the label silently doesn't render.
- **Don't ship a label PLY without re-running `make assets` after editing the LABELS list.** Committed PLYs go stale; the Go loader looks up by exact filename.
- **Don't trust `viam-server` to auto-reload local-module binaries.** When iterating with a `local` module type, edit code → `go build -o bin/...` → restart the module from the Viam app's Modules page. The binary on disk changes but the running process doesn't until viam-server respawns it.
- **Don't omit `simulate-time: true` on simulated arms.** Without it, `MoveToJointPositions` blocks forever — no error, no log, the scenario just hangs in `executing` stage.

## Related work

- Structural template: `[[project-example-visualizations]]` (`~/viam/example-visualizations-go`). Read its CLAUDE.md before debugging renderer-format issues.
- Sibling Python module: `~/viam/example-visualizations-python`. Source of the text-PLY generation script.
- NOTES.md tracks open questions about the RDK motion API + the perf characteristic above.
