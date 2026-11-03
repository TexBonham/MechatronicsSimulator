# ADR 0006: Headless core, single facade, invariant nine

Date: 2026-11-02
Status: Accepted
Closes rulings: R-15
Supersedes: none

## Context
The architecture doc's Viewer and interface paragraph built the tool around a PySide6 desktop shell. CI, batch runs
(Decisions log row 25) and a replaceable front end (ADR 0007) all need the simulator to run with no display and to be
driven through one stable surface.

## Decision
Invariant 9, added to the architecture doc's invariants list:

| Element | Decision |
|---|---|
| Display dependency | No Qt or display dependency outside viewer code |
| Facade | One serializable facade; every client drives the simulator through it |
| Replay | Reads run archives, not live engine state |
| Recorder | Streaming seam present from milestone 1 |
| First client | The CLI, which is also the client CI uses (commands per ADR 0012, L9) |

The facade service is Python (ADR 0005) and lives in `mechsim/facade/`.

## Consequences
- Every run CI performs is a headless CLI run; nothing in the test tiers needs a display.
- The invariant tier's import-boundary test (ADR 0008) enforces that Qt and display libraries are imported only by
  viewer code (`core/viewer_debug/` and the clients).
- Front ends can be replaced without touching the core, because all of them sit behind the facade.
- The recorder must expose a streaming interface in milestone 1 even though the first consumer may come later.
- Replay works from any archive without the engine that produced it.

## Open items
- Facade protocol: phase 1 ADR (ADR 0007).
