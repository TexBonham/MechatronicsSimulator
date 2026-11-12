# Spec: Mechanical integrity (INT)

## Purpose

Mechanical integrity (`mechsim/integrity/`) checks whether a design's members, mounts, fasteners and bearings survive
the loads a run produces: data-driven, tiered check plugins, each with an analytical reference, evaluated after a run
from the recording and, for tier 1, every tick inside the kernel. It exports load cases and imports FEA results. Its
scope is mechanical integrity, not finite-element structural analysis, which the engine and external tools cover.

"Architecture doc" below is `docs/Mechatronic Simulator — Architecture and Technology Recommendation.md`;
"Requirements doc" is `docs/Mechatronic Simulator — Requirements.md`.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| REC | INT-005, INT-022, INT-024, INT-032 | REC-004, REC-010 |
| MAT | INT-005, INT-008, INT-009, INT-020 | MAT-003, MAT-010, MAT-011, MAT-012, MAT-021 |
| DES | INT-002, INT-019, INT-020, INT-021, INT-033, INT-034, INT-035 | DES-008, DES-031, DES-032, DES-033, DES-034, DES-035 |
| KER | INT-023, INT-026, INT-027 | KER-009, KER-015, KER-018, KER-019 |
| PHY | INT-023, INT-028, INT-029, INT-030, INT-031 | PHY-007, PHY-008, ENG-030, ENG-031 to ENG-034 |
| SCO | INT-022, INT-025 | SCO-014, SCO-015 |
| BLD | INT-034, INT-038 | BLD-008, BLD-009, BLD-019 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| INT-001 | Each integrity check shall be a plugin, added as a plugin class, its schema entry and a reference test with no change to other integrity code. | ADR 0009, Decision, Check model; Architecture doc, Repository starting point, How to extend, integrity check row; Ruling R-19 | M3 | unit | n/a |
| INT-002 | Which checks apply to a design and their parameters shall come from data files, not code. | ADR 0009, Decision, Check model, Definition; Ruling R-19 | M3 | unit | n/a |
| INT-003 | Each check shall be tiered like component plugins, and adding a higher tier shall not change a lower tier's results. | ADR 0009, Decision, Check model, Tiers; Architecture doc, Repository starting point, invariant 6 | M3 | unit | n/a |
| INT-004 | Each check shall ship with an analytical reference test that its result matches within the tolerance this spec states for it. | ADR 0009, Decision, Check model, Verification; ADR 0008, Decision, Tolerances (T4) | M3 | reference | not set (owed by owner, proposed per check: the INT-005 to INT-012 rows) |
| INT-005 | The strength check shall compute each member's and mount's peak stress from the recorded member end-loads using beam and plate theory on its section, and its safety factor against the material's process- and direction-dependent limits. | Architecture doc, Modules: outputs, Structural analysis row; Requirements doc, Physics and structural analysis requirements, Structural checks; ADR 0009, v1 check set | M3 | reference | not set (owed by owner, proposed 1e-9 relative) |
| INT-006 | The deflection check shall compute each member's peak deflection over the run. | Architecture doc, Modules: outputs, Structural analysis row; Requirements doc, Physics and structural analysis requirements, Structural checks; ADR 0009, v1 check set | M3 | reference | not set (owed by owner, proposed 1e-3 relative) |
| INT-007 | The buckling check shall compute each compression member's buckling margin over the run. | ADR 0009, v1 check set; ADR 0010, Decision, Tier 1 quantities | M3 | reference | not set (owed by owner, proposed 1e-9 relative) |
| INT-008 | The fatigue check shall count each member's stress cycles from the recorded loads of a run and compare them with the material's S-N curve. | Architecture doc, Modules: outputs, Structural analysis row; ADR 0009, v1 check set | M3 | reference | not set (owed by owner, proposed 1e-9 relative) |
| INT-009 | Withdrawn: duplicate of INT-008 after the suite-accumulation scope moved to Open parameters. | Architecture doc, Modules: outputs, Structural analysis row | M3 | reference | not set (owed by owner) |
| INT-010 | The fastener check shall compute each fastener's peak load and its margin against the fastener grade's limit. | ADR 0009, v1 check set; ADR 0010, Decision, Tier 1 quantities | M3 | reference | not set (owed by owner, proposed 1e-9 relative) |
| INT-011 | The impact check shall evaluate members and mounts against the peak loads of impact events in the run. | ADR 0009, Decision (v1 check set) | M3 | reference | not set (owed by owner, proposed 1e-9 relative) |
| INT-012 | The resonance screening check shall flag each flexible member whose natural frequency falls within the screening band of an excitation present in the run. | ADR 0009, Decision (v1 check set) | M3 | reference | not set (owed by owner, proposed 1e-9 relative) |
| INT-013 | The bearing life check shall compute each bearing's rated life from its recorded loads. | ADR 0009, v1 check set; Ruling R-20 | M4 | reference | not set (owed by owner) |
| INT-014 | The support spacing design rule shall flag a member whose supports are farther apart than the rule's limit. | ADR 0009, v1 check set, build-time design rules; Ruling R-20 | M4 | unit | n/a |
| INT-015 | The cantilever ratio design rule shall flag a cantilevered member whose length-to-section ratio exceeds the rule's limit. | ADR 0009, v1 check set, build-time design rules; Ruling R-20 | M4 | unit | n/a |
| INT-016 | The clearance design rule shall flag two bodies whose clearance anywhere in their joint range is below the rule's limit. | ADR 0009, v1 check set, build-time design rules; Ruling R-20 | M4 | unit | n/a |
| INT-017 | The misalignment binding design rule shall flag parallel guides or shafts whose misalignment exceeds the rule's limit. | ADR 0009, v1 check set, build-time design rules; Ruling R-20 | M4 | unit | n/a |
| INT-018 | The tip-over design rule shall flag a design whose combined center of mass can leave its support footprint under the design load. | ADR 0009, v1 check set, build-time design rules; Ruling R-20 | M4 | unit | n/a |
| INT-019 | Integrity checks shall apply the design-load rule each design file states, and no rule written into code. | Requirements doc, Physics and structural analysis requirements, Structural checks; Architecture doc, Modules: outputs, Structural analysis row | M3 | unit | n/a |
| INT-020 | Integrity shall flag every member and mount whose safety factor is below the design's minimum safety factor. | Requirements doc, Physics and structural analysis requirements, Structural checks; Architecture doc, Modules: outputs, Structural analysis row | M3 | unit | n/a |
| INT-021 | Integrity shall flag every member whose deflection exceeds the design's deflection target. | Requirements doc, Physics and structural analysis requirements, Structural checks; Architecture doc, Modules: outputs, Structural analysis row | M3 | unit | n/a |
| INT-022 | Integrity shall produce, per test case, a per-member table of peak stress, deflection and safety factor. | Architecture doc, Modules: outputs, Structural analysis row, Produces | M3 | unit | n/a |
| INT-023 | A tier 1 kernel party shall evaluate member stress, buckling margin, fastener load and deflection every tick from the live loads physics publishes. | ADR 0010, Decision, Tier 1; Ruling R-21 | M3 | unit | n/a |
| INT-024 | When a tier 1 quantity crosses its limit, integrity shall raise an event stamped with the simulated time of the crossing. | ADR 0010, Decision, Tier 1, On a limit crossing; Ruling R-21 | M3 | unit | n/a |
| INT-025 | A tier 1 limit crossing shall fail the case. | ADR 0010, Decision, Tier 1, On a limit crossing; Ruling R-21 | M3 | unit | n/a |
| INT-026 | A tier 1 limit crossing shall halt the run when halting is configured, and let the run continue otherwise. | ADR 0010, Decision, Tier 1, On a limit crossing; ADR 0010 amendment 2026-11-11 (halt field, round 5 ruling 4) | M3 | unit | n/a |
| INT-027 | Tier 1 events shall be deterministic: the same design, test case, seed and mode raise the same events at the same simulated times. | ADR 0010, Consequences; Architecture doc, Repository starting point, invariant 4 | M3 | unit | n/a |
| INT-028 | A tier 2 yield shall set a permanent spring offset in the member's model for the rest of the run. | ADR 0010, Decision, Tier 2; Ruling R-21 | M4 | unit | n/a |
| INT-029 | A tier 2 buckling failure shall collapse the member's stiffness for the rest of the run. | ADR 0010, Decision, Tier 2; Ruling R-21 | M4 | unit | n/a |
| INT-030 | A tier 2 fastener break shall disable that fastener's constraint for the rest of the run. | ADR 0010, Decision, Tier 2; Ruling R-21 | M4 | unit | n/a |
| INT-031 | A tier 2 rope snap shall drop the load the rope carries. | ADR 0010, Decision, Tier 2; Ruling R-21 | M4 | unit | n/a |
| INT-032 | Integrity shall export, per test case, the peak force and torque at each joint and mount as a load-case file giving location, direction and magnitude. | Requirements doc, Physics and structural analysis requirements, Load handoff item 1; Architecture doc, Modules: outputs, Structural analysis row | M4 | unit | n/a |
| INT-033 | Every load-case export shall record the design version it belongs to. | Requirements doc, Physics and structural analysis requirements, Load handoff item 3; ADR 0012 (D3) | M4 | unit | n/a |
| INT-034 | Integrity shall import FEA results (stiffness, deflection, natural frequencies) and write the calibrated deflection and first natural frequency of each matching flexible member into the design file, with the design version recorded, for the builder to use. | Requirements doc, Physics and structural analysis requirements, Load handoff item 2 | M4 | unit | n/a |
| INT-035 | An FEA import shall record the design version it belongs to and end with an error when that version differs from the design being calibrated. | Requirements doc, Physics and structural analysis requirements, Load handoff item 3 | M4 | unit | n/a |
| INT-036 | The integrity data model shall carry a thermal check slot that v1 accepts and does not evaluate. | ADR 0009, v1 check set, Seams only; Ruling R-20 | M3 | unit | n/a |
| INT-037 | Integrity shall record cycle counts for the wear seam without integrating wear in v1. | ADR 0009, Consequences; Architecture doc, Seams reserved for the roadmap, wear row | M3 | unit | n/a |
| INT-038 | Build-time design rules shall run on the design before a run and be able to fail a design without simulating it. | ADR 0009, Consequences | M4 | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Analytical reference tolerance of each check (INT-004 to INT-013) | not set; proposed in each row (brief I1): 1e-9 relative where the check evaluates a closed form exactly, 1e-3 relative for deflection (grid maximum, derivation in the test) | owner (T4) |
| Minimum safety factor | per design, in design data; reference design value not set | design team |
| Deflection target, and whether it is static or dynamic | per design, in design data; reference design static or dynamic not set | design team |
| Design-load rule | per design, in design data | design team |
| Cycle-counting method for fatigue (for example rainflow) | Rainflow, ASTM E1049-85 three-point method, residual ranges as half cycles, no mean-stress correction (brief I1) | INT module plan |
| Fatigue accumulation scope (beyond one run: suite or lifetime) | not set | owner |
| Resonance screening band | per design: `band` of the resonance check in `integrity.yaml`, no default; reference design value not set | owner |
| Limits of each build-time design rule | not set | owner |
| Where the optional halt is configured (test case or design) and its default | Test-case field `integrity.halt_on_crossing`, default false (ADR 0010, amendment 2026-11-11; round 5 ruling 4) | owner confirms at the milestone 3 gate |
| Split of tier 2 failure modes between milestones 3 and 4 | not set (M4 shown as the latest) | milestone 3 plan (ADR 0010 open item) |
| Load-case and FEA import file formats | Load case: CSV and JSON, one entry per member end and quantity (brief I1, check rules below); FEA import not set (M4) | INT module plan |

## Check rules (brief I1, milestone 3)

| Topic | Rule |
|---|---|
| Configuration (INT-002) | `integrity.yaml` beside the design's `design.yaml` (ruling 27; `schemas/integrity.json`, worked example `schemas/examples/integrity.yaml`, file kind `integrity` for `validate`): `schema_version` 1, `checks` (each `check`, fidelity `tier`, `parameters`), `halt_on_crossing` (mirror of the test-case field, ruling 4), `thermal` (INT-036 slot). Absent file: the run carries no table. Strength and deflection always run when the file is present (the table needs them). |
| Plugins and tiers (INT-001, INT-003) | A check is a module under `mechsim/integrity/checks/` defining `CHECK` (or an entry point in group `mechsim.integrity_checks`); its schema entry is generated from its parameters model. "Tier" is the fidelity tier: a higher tier adds rows and leaves lower-tier rows unchanged. Strength tier 2 adds average shear V / A and von Mises sqrt(sigma^2 + 3 tau^2). |
| End-loads (ruling 24) | Member frame, x from start to end, y and z the principal axes; each end's wrench is the load applied to the member there, so internal axial force is -F_start.x and +F_end.x (tension positive). |
| Design load (INT-019) | Recorded loads times 1 + `structural.design_load.allowance` from the design file (the run is taken to carry the rated load); 1 when the design has no `structural` section. Table values are at the design load. |
| Strength (INT-005) | Per end and sample abs(N) / A + abs(M_y) c_z / I_y + abs(M_z) c_y / I_z (ruling 25, member frame of ruling 24, the same formula as the VIE-010 overlay; I_z the section's `second_moment_of_area`, I_y `second_moment_y`, c_y `fibre_y`, c_z `fibre_z`; a section missing any y-axis field is evaluated with I_y = I_z and c_y = c_z = the stated fibre or sqrt(3 I / A), and its table row carries the flag `approximation: single-axis section`, never refused); safety factor = yield strength spread along the member axis at the ambient temperature over the peak. Mounts: abs(F) / A + abs(M) max(c) / min(I) of the joint reaction on the held member's section (plate theory owed: no mount sections yet). |
| Deflection (INT-006, INT-021) | Quasi-static per sample from the end-loads: E I v'' from the start wrench plus the interior load resolved to one point load at its statically equivalent position, supports fixing the rigid terms; peak magnitude over 201 points along the member and over the run, against `structural.accuracy_target`. Exact for end and single point loads; reads high for a uniform load. |
| Buckling (INT-007) | Peak compression over the run against pi^2 E I_min / (K L)^2, K 2 (clamped-free), 1 (pinned-pinned), 0.5 (clamped-clamped), 0.699 (clamped-pinned), or the configured override. |
| Fatigue (INT-008, INT-037) | Rainflow on the signed normal stress at the four corners of each end section; Miner sum against the S-N curve (log-log; infinite life below its last amplitude; first sloped segment extended above its first); flagged at `damage_limit` (default 1, Miner). The governing history's cycle count is recorded. |
| Fastener (INT-010) | Peak joint reaction force against the rating in `ratings` (catalog grade ratings owed); margin rating / peak, flagged below the minimum safety factor (1 when the design sets none). |
| Impact (INT-011) | An event is a rise of a member's largest end force (or a mount's reaction) by more than `step_threshold` between samples, one per `window`; peak stress over the window times `impact_factor` against ultimate strength. |
| Resonance (INT-012) | First frequency: the calibrated value (BLD-019), else lambda^2 / (2 pi L^2) sqrt(E I_min / (rho A)) for the end conditions; each configured series gives f = abs(value) `hz_per_unit`; flagged when any sample is within `band` f_n. |
| Rope table (INT-022, ADR 0016) | One `RopeIntegrity` row per rope of the model: peak of the recorded largest segment tension (`physics/ropes/max_tension`) and its time; flag `snap` above the rope's `breaking_tension` (absent: never, ruling 19), flag `design_load` above rated_load (1 + allowance) abs(g) when the design states the design-load rule. Reference: a hanging mass at m g, proposed 1e-9 relative. |
| Flags and table (INT-020 to INT-022) | Flags: `safety_factor_below_minimum` (minimum-strength safety factor), `deflection_over_target`, `buckling_margin_below_minimum`, `fatigue_damage_over_limit`, `fastener_margin_below_minimum`, `resonance`. One table row per member and mount (R's `IntegrityResults`): tier 1 peak stress, deflection (0 for a mount), smallest safety factor over the strength rows, flags. Written beside the archive as `<run>.integrity.json` (table, every check's rows, cycle counts, notes) with `<run>.load_case.json` and `.csv`. |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| General-purpose FEA solving | External tools; engine beam-element tier (Architecture doc Seams table) |
| Publishing live member end-loads | PHY |
| Thermal and wear check implementations | Roadmap (ADR 0009) |
| Rope model behind rope snap | Milestone 3 rope ADR |
| Scoring structural criteria | SCO |
