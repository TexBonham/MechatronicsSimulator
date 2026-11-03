# Architecture decision records

Decisions of record for the simulator. Source rulings: `docs/rulings/2026-11-02-planning-rulings.md`. Process rulings
(W-01 to W-07) are not ADRs and stay in that file; product rulings R-01 to R-03 are rows in the architecture doc's
Decisions log.

| ADR | Title | Status | Closes rulings |
|---|---|---|---|
| [0001](0001-own-physics-engine.md) | Own physics engine in C++ | Accepted | R-04, R-07, R-12, R-13 |
| [0002](0002-collision-and-contact-scope.md) | In-house collision detection and v1 contact scope | Accepted | R-05, R-06 |
| [0003](0003-neutral-physics-model-and-adapters.md) | Engine-neutral physics model behind adapters | Accepted | R-10, P2, P3, P4 |
| [0004](0004-mujoco-adapter-lifecycle.md) | MuJoCo adapter as bridge and differential oracle, then retired | Accepted | R-11 |
| [0005](0005-core-runtime-and-language-split.md) | C++ core runtime, Python application layer, tiered plugins | Accepted | R-08, R-09, R-14 |
| [0006](0006-headless-core-and-facade.md) | Headless core, single facade, invariant nine | Accepted | R-15 |
| [0007](0007-front-end.md) | Browser client over a local service, replaceable by a standalone app | Accepted | R-16, R-17 |
| [0008](0008-spec-first-and-test-tiers.md) | Spec before code, strict-xfail stubs, traceability, spec maintenance | Accepted | R-18, T1, T2, T3, T4 |
| [0009](0009-mechanical-integrity-scope.md) | Checks as plugins; v1 check set and milestone placement | Accepted | R-19, R-20 |
| [0010](0010-mid-run-structural-failure.md) | In-loop detection and modeled consequences of structural failure | Accepted | R-21 |
| [0011](0011-firmware-and-native-tick-contract.md) | Firmware language, build, polled native tick, HAL placement | Accepted | C1, C2, C3, C4, C5, C6 |
| [0012](0012-data-conventions.md) | Units, seeding, signal naming, versioning, archives, baselines, tooling | Accepted | L1-L10, D1, D2, D3, D4, R1, R2 |
| [0013](0013-own-engine-integrator.md) | Own-engine integrator | Accepted | P2 (own-engine half) |
| [0014](0014-mujoco-bridge-split.md) | MuJoCo bridge split: Python translation, C++ stepping | Accepted | refines R-11, ADR 0004 |
| [0015](0015-facade-protocol.md) | Facade protocol: JSON over WebSocket, binary series frames | Proposed | R-16 (protocol), FAC-018 to FAC-020 |

To add an ADR, copy `0000-template.md` to the next free number with a short slug and fill every section. Add a row to
the table above and a row to the Decisions log in the architecture doc, marking "Changes doc" Yes where the ADR
contradicts existing doc text.
