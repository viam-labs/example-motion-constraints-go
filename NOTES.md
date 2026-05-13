# NOTES.md — issues, open questions, RDK bugs to file

This is a living document. Append to it as we hit things; promote to RDK issues or README items once resolved.

## Open questions

### OQ1 — How do we get the planned trajectory for preview?

`motion.Service.Move(MoveReq{...})` returns `(bool, error)` — no trajectory. To render a ghost preview we need the waypoint sequence (joint positions and/or cartesian EE poses).

Candidate approaches:

- **(a)** Call `motionplan.PlanMotion` directly with a manually constructed `FrameSystem`. Most flexible; bypasses the motion service entirely. Risk: `motionplan` is closer to internal, signatures may shift.
- **(b)** `MoveOnMap` + `PlanHistory`. Looks base-only; almost certainly the wrong API for arms.
- **(c)** No preview — poll `arm.GetEndPosition` during execution and emit the actual path as the arm moves. Worse UX (no foreknowledge of collisions), but simplest fallback.

**Decision target:** Phase 3 spike (`cmd/spike/main.go`).

### OQ2 — Does the builtin motion service auto-include sibling arms as collision obstacles?

The frame system "knows about" all configured arms via their frame entries. But it's unclear whether the motion planner pulls each arm's link kinematics into `WorldState.GeometriesInFrame` automatically when planning for arm A, or whether we must inject arm B's kinematics manually.

If manual: each scenario's plan step has to enumerate other arms, fetch their current `JointPositions`, build link geometries, and add them to `WorldState`.

**Decision target:** Phase 3 spike — set up two adjacent fake arms and ask motion to move one through the other. If it routes around, auto-inclusion is real.

### OQ3 — Does `metadata.colors` field-mask UPDATE re-paint in the 3D viewer?

The renderer is known to cache REMOVED UUIDs and replay ADDED with the same UUID may or may not re-render. Similar uncertainty for in-place metadata UPDATEs: does sending `Type: UPDATED` with `UpdatedFields: ["metadata.colors"]` actually change the displayed color, or does the viewer only react to geometry-shape changes?

**Decision target:** Phase 6 collision-detection work. Try stable UUID UPDATE first; if it doesn't repaint, switch the red-tint emit to versioned-UUID (REMOVED + ADDED with fresh UUID + new color).

## Customer-facing edge cases

_Reserved — populate as we discover them while building scenarios._

## RDK bugs to file

_Reserved — populate as we identify issues that should go upstream._

## Improvements deferred

_Reserved — features we noticed but didn't ship yet._
