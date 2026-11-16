# ADR 0004: MuJoCo adapter as bridge and differential oracle, then retired

Date: 2026-11-02
Status: Accepted; decision implemented 2026-11-15, the adapter retired (owner gate decision 1b; brief M3-RM; owner
reviews this status edit)
Closes rulings: R-11
Supersedes: Architecture doc, Technology recommendation verdict "MuJoCo for v1"; Repository starting point, "replay in
MuJoCo's viewer" for milestone 1 (replay per ADR 0006 and ADR 0007)

## Context
The own engine (ADR 0001) cannot carry the requirements doc's milestones 1 and 2 on day one. MuJoCo was the doc's v1
engine and is the fastest route to a running gantry. A whole-design reference is also needed to check the own engine
beyond the analytical suite.

## Decision

| Aspect | Decision |
|---|---|
| Role 1: bridge | A thin MuJoCo adapter carries milestones 1 and 2 |
| Role 2: oracle | It is the differential oracle for the own engine on whole-design runs |
| Retirement criteria | The own engine passes the analytical reference suite and matches the gantry baselines |
| After retirement | The comparison recordings are kept as fixed baselines |
| Integrator | implicitfast at 1 ms (ADR 0003, P2) |

## Consequences
- Milestone 1 does not wait for the own engine; the engine is built in parallel.
- The adapter stays thin: it translates the neutral model (ADR 0003) and nothing more; no feature is built into it that
  the own engine will not also have.
- Retirement is gated on the criteria above, not on a date. If the own engine's contact solver slips, the MuJoCo adapter
  keeps carrying runs until the criteria are met.
- Fixed baselines are kept as YAML scoring summaries in git; the archives themselves stay out of git (ADR 0012, D4).
- After retirement, MuJoCo is no longer a runtime dependency.

## Retirement (2026-11-15)
- The owner retired the adapter at the milestone 3 gate (decision 1b, condition met when brief M3-X2i landed).
  Criterion 1 met (PHY-023). Criterion 2 decided by the owner: the four G10 misses of record (`stall_fault`,
  `slip_fault`, a compliant-against-rigid x belt difference at the homing latch; `stall_rehomed`,
  `slip_carriage_stopped` their consequences) stand as the record (PHY-024), and the efficiency-damping release
  artefact is bounded and published per run, its end-rate form milestone 4 item E9.
- Removed by brief M3-RM: `core/physics/mujoco/`, `core/cmake/MuJoCo.cmake`, `mechsim/physics/mujoco/`, the libmujoco
  install in the wheel, the bridge tests and the `mujoco` value of `physics.adapter`, which is now refused with an
  error naming the retired adapter. The fixed baselines under `test_cases/*/bridge_baselines/` stay (PHY-025). No
  MuJoCo import or include is allowed anywhere (PHY-011, PHY-026: the import checker, the CI include step and the
  invariant tier). History of the stepping adapter: `core/physics/HISTORY.md`.

## Open items
- Which engine carries milestone 3 if the retirement criteria are not met by then: moot, retired at the gate.
- Status of the Chrono adapter reserved in the architecture doc: not ruled.
