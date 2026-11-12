# Spec: Viewer and clients (VIE)

## Purpose

The viewer (`mechsim/viewer/`) prepares 3D replay, the event timeline, probes and overlays from run archives and serves
them to clients through the facade; the browser client (`web/`) displays them, and a standalone app may replace it
later through the same protocol. Clients hold no domain logic. The engine debug viewer in `core/viewer_debug/` is an
engine development tool and is not specified here.

"Architecture doc" below is `docs/Mechatronic Simulator — Architecture and Technology Recommendation.md`;
"Requirements doc" is `docs/Mechatronic Simulator — Requirements.md`.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| FAC | VIE-012, VIE-013, VIE-014, VIE-015, VIE-018 | FAC-001, FAC-005, FAC-011, FAC-016, FAC-017 |
| REC | VIE-001, VIE-002, VIE-016 | REC-008, REC-009, REC-024 |
| SCO | VIE-002 | SCO-003, SCO-004 |
| INT | VIE-010 | INT-022 |
| DES | VIE-015, VIE-018, VIE-019 | DES-011, DES-012, DES-017, DES-036 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| VIE-001 | The viewer shall replay a run in 3D from its run archive. | Architecture doc, Modules: outputs, Viewer and replay row; Requirements doc, Outputs and acceptance criteria, Outputs per run | M1 | unit | n/a |
| VIE-002 | The replay shall show an event timeline marking every operator command, injected fault, safety event and criterion failure at its simulated time. | Architecture doc, Modules: outputs, Viewer and replay row; Requirements doc, Outputs and acceptance criteria, Outputs per run | M1 | unit | n/a |
| VIE-003 | The viewer shall offer a probe on any body that shows its recorded values over time. | Architecture doc, Modules: outputs, Viewer and replay row | M1 | unit | n/a |
| VIE-004 | The viewer shall offer a probe on any joint that shows its recorded values over time. | Architecture doc, Modules: outputs, Viewer and replay row | M1 | unit | n/a |
| VIE-005 | The viewer shall offer a probe on any pin that shows its recorded values over time. | Architecture doc, Modules: outputs, Viewer and replay row | M1 | unit | n/a |
| VIE-006 | The viewer shall offer a probe on any signal that shows its recorded values over time. | Architecture doc, Modules: outputs, Viewer and replay row | M1 | unit | n/a |
| VIE-007 | The replay shall scrub to any recorded simulated time. | Architecture doc, Modules: outputs, Viewer and replay row | M1 | unit | n/a |
| VIE-008 | The replay shall pause and resume. | Architecture doc, Modules: outputs, Viewer and replay row | M1 | unit | n/a |
| VIE-009 | The replay shall step forward and backward one recorded sample at a time. | Architecture doc, Modules: outputs, Viewer and replay row | M1 | unit | n/a |
| VIE-010 | The replay shall overlay member stress on the structural members. | Architecture doc, Modules: outputs, Viewer and replay row | M3 | unit | n/a |
| VIE-011 | The replay shall overlay the swing angle of suspended loads. | Architecture doc, Modules: outputs, Viewer and replay row | M3 | unit | n/a |
| VIE-012 | The early front end shall be a browser client in `web/` over the local Python facade service. | ADR 0007, Decision, Early front end; Ruling R-16 | M1 | unit | n/a |
| VIE-013 | A standalone app replacing the browser client shall use the same facade protocol and pass the same conformance suite. | ADR 0007, Decision, Later front end; ADR 0007, Consequences | roadmap | unit | n/a |
| VIE-014 | Overlays, probe values and timeline events shall be computed in `mechsim/viewer/` and served through the facade, never in a client. | ADR 0007, Decision, Client role; Architecture doc, Repository starting point, layout (`mechsim/viewer/`) | M1 | unit | n/a |
| VIE-015 | The viewer shall take body geometry from the run's design file through the facade. | Architecture doc, Modules: outputs, Viewer and replay row, Reads | M1 | unit | n/a |
| VIE-016 | Replay shall work from any run archive without the engine that produced it. | ADR 0006, Consequences | M1 | unit | n/a |
| VIE-017 | The viewer shall produce on-screen output only and write no files. | Architecture doc, Modules: outputs, Viewer and replay row, Produces | M1 | unit | n/a |
| VIE-018 | A GUI editor, when it exists, shall write the same design and test case files through the facade. | Architecture doc, Repository starting point, invariant 3 | roadmap | unit | n/a |
| VIE-019 | GUI editor forms shall be generated from the YAML schemas. | Architecture doc, Seams reserved for the roadmap, GUI design and test editors row | roadmap | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Browser rendering library | M2-F2: three.js 0.170.0, vendored as `web/vendor/three.module.js`, version and SHA-256 recorded in `web/README.md`; no CDN at runtime, no build step | VIE module plan |
| When the standalone app replaces the browser client | not set | owner (ADR 0007 open item) |
| Live view during a run (beyond replay) | not set | phase 1 ADR (facade protocol) |
| Probe display form (plot, readout) | M2-F2: readout of the probe's columns at the cursor, each with the unit its column names; plot not set | VIE module plan |
| Browser client serving (VIE-012) | M2-F2: `mechsim serve [--files-root] [--runs] [--port] [--check]` on 127.0.0.1 prints `http://127.0.0.1:<port>/?token=<token>`; plain GETs on the same port serve `/` (`index.html`), top-level `*.html`, `*.js`, `*.css` and `vendor/*` of `web/`, anything else (`..`, absolute, encoded, links out of `web/`) 404; the client opens `/ws?token=` on its own origin. Verified without a browser (ruling 21) plus the manual checklist in `web/README.md` | VIE module plan |
| Replay drawing (VIE-001, VIE-015) | M2-F2: bodies from `geometry` (box, sphere, cylinder, capsule by sizes, cylinder and capsule along the shape's z axis; a `plane` or a shape with `vertices` drawn when the facade sends one; a shape with an open size not drawn and the body named), posed by the `pose:` series (x, y, z, roll, pitch, yaw, Z-Y-X) from `subscribe_series` windows at the cursor | VIE module plan |
| Timeline categories of recorded event kinds (VIE-002) | M2-F1: command = `operator_message`, `operator_command`, `operator_status`; fault = `motor_stall`, `rail_off`, `rail_on` or a kind containing `fault`; safety event = a kind containing `estop` or `safety`; criterion failure = a failed criterion at its worst time (the run's end when it has none), for runs the serving facade scored; other kinds are not marked. Injected faults appear only where the run records an event for them | VIE module plan |
| Probe values (VIE-003 to VIE-006) | M2-F1: body = `pose:` x, y, z, roll, pitch, yaw, plus linear and angular velocity from the archived body states; joint = `joint:` position and velocity plus generalized force columns; pin = a `signal:` of the pins group; signal = any `signal:` | VIE module plan |
| Replay cursor (VIE-007 to VIE-009) | M2-F1: starts paused at the first recorded sample; scrub to the latest sample at or before t (the first when t precedes it); step one sample, held at either end; advance by dt only while playing, from a play time kept beside the sample index so the time past the shown sample carries to the next advance (scrub sets it to the time asked, clamped to the recording; step to the sample reached; brief R). The browser client plays back by sending `advance` with the frame interval times the rate and draws the cursor the facade answers with, keeping no simulated clock of its own (ruling 22, M2-F3) | VIE module plan |
| Overlays (VIE-010, VIE-011) | member stress: per recorded member, bending stress M c / I at the start and end every sample, M = sqrt(M_y^2 + M_z^2) of the recorded end moment (member frame, x along the member), I the member list's second moment of area, c its stated extreme fibre or else sqrt(3 I / A), the solid rectangle with the section's A and I (round 5 ruling 6 placeholder sections); swing angle: per sample atan2(\|r x d\|, r . d) with r from the attachment body's recorded origin (or a world point) to the load body's origin and d the unit vector of the archived design's gravity (-z when none); both served as series through the facade | brief R (M3) |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Engine debug viewer | ENG (`core/viewer_debug/`, ADR 0007, Ruling R-17) |
| Streaming and decimation | FAC |
| File exports | REC, SCO, INT |
| Computing member stress | INT |
