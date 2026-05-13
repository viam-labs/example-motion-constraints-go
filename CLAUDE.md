# CLAUDE.md — operational context for example-motion-constraints-go

This file primes future Claude Code sessions on how to work on this module. Read it before making changes.

## Release checklist — always follow this order

Before every `viam module upload`:

1. **Commit.** Never leave the tree dirty between sessions. The `Always commit` rule below is absolute.
2. **Update [`README.md`](README.md)** if the change adds/removes a preset, DoCommand verb, config knob, or visible behavior.
3. **Update [`meta.json`](meta.json)** if the model list, description, supported platforms, or entrypoint changed.
4. **Bump [`VERSION`](VERSION)** using semver. Pre-customer phase ships `0.x.y`; risky changes use a `-rcN` suffix (e.g. `0.1.0-rc1`).
5. **`make` + `make test`** — module must build cleanly and unit tests must pass.
6. **`make upload PLATFORM=linux/amd64`** to validate against the registry pipeline. Test on a real machine.
7. **Only once the module is in a known-good state, run `make upload-all`** to publish for all supported platforms.

## Always-commit rule

After any meaningful change — new file, new function, doc edit, presets added, bug fixed — commit it before doing the next thing. A series of small commits is much easier to bisect than one mega-commit. If a session ends mid-task with uncommitted work, the next session has no idea what was in flight.

## Architecture rationale

The module registers a single `rdk:service:world_state_store` model. This service does **two** things:

1. **Streams scene primitives** to the 3D viewer (the native purpose of `world_state_store`).
2. **Orchestrates motion** on configured arms via a background tick goroutine.

This is unusual — the sibling `example-visualizations-go` only does (1). We accept the overload because:

- The streaming side is fully non-blocking (per-subscriber channels, capacity 256, non-blocking sends with a warning log on overflow).
- The orchestration side runs in a single tick goroutine that does not block stream consumers.
- One service block in machine config is easier for the instructional audience to copy-paste than two cooperating models.

If this becomes painful (e.g. the motion side starts needing its own dependencies that don't fit world_state_store semantics), the right split is a sibling `rdk:service:generic` model that calls into this one via DoCommand.

## Renderer conventions (gotchas)

These come from `example-visualizations-go`'s hard-won learnings. Violate one and the entity is silently invisible.

1. **Field-mask paths are camelCase.** `poseInObserverFrame.pose.x`, not `pose_in_observer_frame.pose.x`. The `Path*` constants in `animation.go` are the single source of truth.
2. **Metadata struct must include all five keys**: `colors`, `color_format`, `opacities`, `show_axes_helper`, `invisible`. Omit one → invisible entity. Use the `buildMetadata` helper.
3. **Colors and opacities are base64-encoded byte arrays**, not nested structs.
4. **UUID strategy:** use stable UUIDs (`UUID = label`) with field-mask UPDATEs for arms and dynamic obstacles. Use versioned UUIDs (rotating per emission) for trajectory ghost geoms that are added/removed per scenario, and for any entity whose color must flip mid-scenario if stable UPDATE turns out not to repaint in the viewer.
5. **PCD files** (if used): header must match `pointcloud.ToPCD` byte-for-byte — `VERSION .7`, no leading `#` comments.
6. **Mesh content type** is lowercase: `ply` or `stl`.

## Motion-planning conventions

- The motion service's `Move()` does **not** return the planned trajectory; for preview rendering, use `motionplan.PlanMotion` directly with a manually built `FrameSystem`, or fall back to polling `arm.GetEndPosition` during execution.
- Whether sibling arms are auto-included in collision checks by the builtin motion service is an open question — see [NOTES.md](NOTES.md) OQ2. Until confirmed, always inject sibling arm kinematics as `GeometriesInFrame` entries in `WorldState`.
- Constraint types live in `go.viam.com/rdk/motionplan` — `LinearConstraint`, `OrientationConstraint`, `PseudolinearConstraint`, `CollisionSpecification`.

## File layout

| File | Role |
| --- | --- |
| `cmd/module/main.go` | Module entry; registers the planner model. |
| `cmd/spike/main.go` | Throwaway scripts for testing motion-API behavior (excluded from `make`). |
| `service.go` | World-state-store service: lifecycle, subscribers, tick goroutine, DoCommand dispatcher. |
| `config.go` | JSON config schema and validation. |
| `scenarios.go` | Scenario types + execution loop (setup → plan → preview → check → execute → teardown). |
| `presets.go` | Built-in scenario presets. |
| `visualization.go` | Trajectory preview, label sprites, obstacle ADD/REMOVE. |
| `collision.go` | `CheckCollisions` wrapper + red-tint emission. |
| `arms.go` | Multi-arm dependency wiring, grid-layout helpers. |
| `geometries.go` | Proto builders (box, sphere, capsule, mesh) + metadata struct builder. Mirror sibling. |
| `animation.go` | `ComputeTick` + field-mask `Path*` constants for dynamic obstacles. Mirror sibling. |

## Testing

- Unit tests next to each file (`config_test.go`, etc.).
- The spike in `cmd/spike/main.go` is **not** a test — it's an exploratory program. Build it with `go run ./cmd/spike` to investigate API behavior; never include it in the module tarball.
- End-to-end validation requires a Viam machine; use `examples/single-arm-demo.json` as the canonical smallest config.

## Related work

- Structural template: `~/viam/example-visualizations-go` (in the local workspace).
- Sibling Python module: `~/viam/example-visualizations-python`.
- Pallet-modules-style pre-release versioning lives in `[[feedback_pallet_modules_prerelease]]`.
