# Spec: Invariants (INV)

## Purpose

The repository-wide invariants every change must keep. INV-001 to INV-009 are the nine invariants of the architecture
doc in order, with their text verbatim and their enforcement as the root `CLAUDE.md` Invariants table states it.
INV-010 onward make the spec process and the invariant test tier (ADR 0008, ruling T1) themselves requirements.
INV-005 and INV-009 are verified by `tests/invariant/test_import_boundaries.py`.

"Architecture doc" below is `docs/Mechatronic Simulator — Architecture and Technology Recommendation.md`; the
enforcement named in each Source is from the Invariants table of the root `CLAUDE.md`.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| REC | INV-004, INV-014 | REC-022, REC-023 |
| TST | INV-003, INV-004, INV-012 | TST-009, TST-028, TST-029 |
| FAC | INV-009 | FAC-001, FAC-016 |
| VIE | INV-009 | VIE-014 |
| ACC | INV-004 | ACC-045 |
| Every module | INV-001 to INV-019 | n/a |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| INV-001 | The controller never sees true state. Everything between physics and controller crosses the signal layer as electrical signals through the pin-binding table. No convenience path from controller code to physics state, in either mode. | Architecture doc, Repository starting point, invariant 1. Enforced by review | M1 | invariant | n/a |
| INV-002 | Nothing design-specific in code. The gantry exists only under designs/, test_cases/ and firmware/. A requirement that can only be met by special-casing a design is a gap in the library and is flagged, not patched. | Architecture doc, Repository starting point, invariant 2. Enforced by review | M1 | invariant | n/a |
| INV-003 | Data files are the source of truth. Every YAML file validates against its schema; SI units internally, units explicit in files; the GUI editor, when it exists, writes the same files. | Architecture doc, Repository starting point, invariant 3. Enforced by the schema-validation hook and the CI invariant tier (schema validity) | M1 | invariant | n/a |
| INV-004 | Determinism. The same design, test case, seed and controller mode give an identical recording. Any source of nondeterminism is seeded or removed. | Architecture doc, Repository starting point, invariant 4. Enforced by the CI invariant tier (run-twice determinism) | M1 | invariant | n/a |
| INV-005 | Engines and emulators stay behind adapters. No MuJoCo or Renode type crosses the mechsim/physics/ adapter boundary or the core/controller/ runtime boundary; the engine-neutral physics model (ADR 0003) is the only thing that crosses into an adapter. | Architecture doc, Repository starting point, invariant 5. Enforced by the import-boundary hook and the CI invariant tier (import boundaries) | M1 | invariant | n/a |
| INV-006 | Fidelity tiers are additive. Adding a higher tier to a component never changes the lower tier's results. | Architecture doc, Repository starting point, invariant 6. Enforced by review | M1 | invariant | n/a |
| INV-007 | Failures are errors. Solver drift, solver failure and emulator faults end the run with a report; they never produce silent numbers. | Architecture doc, Repository starting point, invariant 7. Enforced by review | M1 | invariant | n/a |
| INV-008 | The board interface is the only coupling to firmware. Firmware application code includes the interface header and nothing else from the simulator or any vendor HAL. | Architecture doc, Repository starting point, invariant 8. Enforced by the firmware host build, which rejects other includes (Ruling C2) | M1 | invariant | n/a |
| INV-009 | The core is headless. Nothing outside viewer code depends on Qt or a display. Every client drives the simulator through one serializable facade; replay reads run archives; the recorder's streaming seam exists from milestone 1. The CLI is the first client and the one CI uses. | Architecture doc, Repository starting point, invariant 9; ADR 0006. Enforced by the import-boundary hook and the CI invariant tier (import boundaries) | M1 | invariant | n/a |
| INV-010 | Every behavior change shall cite the requirement IDs it implements or changes and carry its spec edit in the same commit. | Ruling R-18; ADR 0008, Decision, Behavior changes. Enforced by stage review | M1 | invariant | n/a |
| INV-011 | The traceability check shall pass: every non-withdrawn requirement has a citing test, every citation names a defined requirement, and the committed matrix equals the regenerated one. | Ruling R-18; ADR 0008, Decision, Traceability test and Traceability matrix | M1 | invariant | n/a |
| INV-012 | Every shipped YAML file shall validate against its schema in `schemas/`. | Ruling T1; ADR 0008, Decision, test tiers, Invariant; Architecture doc, Repository starting point, invariant 3 | M1 | invariant | n/a |
| INV-013 | Every schema in `schemas/` shall equal the schema generated from its pydantic model. | Ruling L4; ADR 0012 (L4), Consequences | M1 | invariant | n/a |
| INV-014 | Running each shipped design and test case twice with the same seed and controller mode shall give identical recordings. | Ruling T1; ADR 0008, Decision, test tiers, Invariant; Architecture doc, Repository starting point, invariant 4 | M1 | invariant | n/a |
| INV-015 | An executable import-boundary check over the whole repository shall fail on any engine or emulator type outside its adapter and on any Qt or display dependency outside viewer code. | Ruling T1; ADR 0008, Consequences; ADR 0006, Consequences | M1 | invariant | n/a |
| INV-016 | Every requirement shall have a test stub committed as a strict expected failure before any code is written for it. | Ruling T3; ADR 0008, Decision, Stubs | M1 | invariant | n/a |
| INV-017 | A stub that passes without being converted shall fail the test run. | Ruling T3; ADR 0008, Consequences | M1 | invariant | n/a |
| INV-018 | Every test tolerance shall be stated in a spec, and a tolerance shall change only by a reviewed spec edit. | Ruling T4; ADR 0008, Decision, Tolerances | M1 | invariant | n/a |
| INV-019 | CI shall run lint, type checks and every test tier headless on Ubuntu. | Ruling R1; ADR 0012 (R1); ADR 0006, Consequences | M1 | invariant | n/a |
| INV-020 | The cpp, python and firmware CI workflows shall each run on both `ubuntu-24.04` and `windows-2022`, with the python job's lint, type and test steps identical on both. | Ruling R1; open ruling 5 (Windows only, macOS out); brief M2-W | M2 | invariant | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| How review-enforced invariants (INV-001, INV-002, INV-006, INV-007, INV-010) get an executable check, if any | not set | invariant-auditor brief, phase 0c |
| Windows in the CI matrix | joined in milestone 2 (INV-020): MSVC presets `windows-debug`, `windows-release`, firmware `host` | ADR 0012 (R1) |
| Strict expected failures for C++ tests | `[!shouldfail][stub]` tags (tests/README.md) | ADR 0008 open item |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Module-level determinism details (seeding, archive content) | TST, REC |
| Firmware include rule as a build rule | FW |
| Module behavior | Each module's own spec |
