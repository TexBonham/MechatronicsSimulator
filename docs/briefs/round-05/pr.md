Round 5: milestone 3, the own engine carries the gantry; rope, flexible members, integrity, G12, G15, G16; MuJoCo retired

## What this round delivers

- The own C++ engine (ADR 0013) replaces the MuJoCo bridge as the physics carrier for every case: contact, lumped flexible members with a clamped far end, ropes as lumped chains (ADR 0016) with a slack rule consistent with the step's end state, joint limits, friction and additive drives, mid-run model changes, belt compliance, moving loads on flexible members with a validity flag, structural damping as design data.
- The native adapter is the default; G1 to G16 and the pendulum run on it in one 16-case suite with a 99-entry baseline, byte-identical on a second run. The retirement comparison against the bridge's fixed baselines is per axis and per dimension with every miss listed and the exact list asserted: four misses of record, all G10, each explained as a compliant-against-rigid belt difference; G7 and G9 explained by two named engine mechanisms.
- Integrity: a tier 1 party in C++ (strength, deflection) from node springs with the exact carried moment, tier 2 yield offset and rope snap as Python plugins, not-valid results under one-way coupling, fatigue by damage, fastener and impact checks, the design load and evaluation start as case fields.
- Motor sizing (G16): torque margin and missed steps per motor as scoring quantities read from the record and the catalog, judged between rest samples, in the simulator's report; the real NEMA 17 cannot lift the design load (margin -0.308 N m at 60 rpm), the sizing answer the case exists to give.
- Real components in the gantry team's open slots (owner decision 4): 2040 aluminium extrusion rails with per-rail calibration, StepperOnline 23HS22 and 17HS19 with their published pull-out curves, Gates GT2 6 mm, 1/16 in 7x19 cable, a TAL220B cell with an HX711-class front end, M5 8.8 mounts; every value sourced in `docs/gantry-team-inputs.md` and flagged replaceable. G15's safety factors pass (37.96 and 24.04) and its settled deflections pass against targets derived for the real section; the evaluation start re-derived to 1 s; the lift belt's overload (55 to 64 N against 26 N working tension) recorded as a design finding.
- MuJoCo retired (ADR 0004 close, owner decision 1b): the bridge packages, the fetch, the wheel's libmujoco and the bridge-only tests removed; `physics.adapter: mujoco` refused with an error naming the retired adapter; the fixed bridge baselines and the native baseline at retirement kept as the comparison of record; "no MuJoCo import anywhere" enforced by the import checker and the invariant tier.
- Fast test tier from 38 to 16 minutes (session run cache, long-run marker); verification policy W-08.

## Gate

Fable gate review at 66d8b26: pass with owner confirmations; decisions 1b, 2a and the confirmation batch given by the owner on 2026-11-15 (`docs/queue.md`, Milestone 3 gate report). Reference tier 61 C++ and 105 Python cases, no tolerance miss, the pendulum period identical on both adapters. Final verification at the landing: ctest debug-sanitize 525 of 525 (reference label at the gate: 61 of 61), firmware host 92, Python 4630 passed and 88 strict-xfail stubs in the fast tier, 38 passed and 2 xfail in the long tier, lint, format, types and imports clean, invariant audit clean. Round cost about 19.9M tokens (17.1M at the gate) against 6.0M to 6.8M planned and an 8.0M budget; the three largest overruns are named in the gate report.

## Recorded, not passed

G12's swing settle (no physical swing damping in the model; derived about 1135 s against the 5 s placeholder); ACC-038 (a torque curve cannot be tested against itself); the lift belt finding; E9 (the frozen-rate efficiency damping over-brakes a released rotor, bounded and published as an approximation line, proper fix in milestone 4).

## Carried to milestone 4

By ID in the gate report: ACC-025's step-train companion, ACC-026, ACC-038, PHY-015 retired, LIB-009, SIG-031, SIG-033 (the stall event raised only on a tick with no commanded microstep), SCO-031 and SCO-032 minors, TST-027, ENG-052's moment channel, E9 proper, the static start, two-axis bending, full coupling, the second-order integrator tier, true stick, INT-005 plate theory, release designs, the payload's identity as a document field, the builder's `rigid_members` option, the HX711 digital output's schema field; pytest-xdist for the long tier and the batch feeding the acceptance fixtures (W-08 d); engine step performance.

## Review

Stage reviews of record, the gate review, the Opus gate audit and the reference validator's report are listed in `docs/cost-log-archive.md` (round 5) with their briefs under `docs/briefs/round-05/`. Merge with a merge commit.
