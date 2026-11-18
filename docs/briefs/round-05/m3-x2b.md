# M3-X2b: continue X2 (the suite on native by default, G16, one batch, the retirement check)

Fresh agent (X2 stopped at its cap with items 1 and 2 written but not green). Agent: module-writer (Opus).
Worktree `../MechatronicsSimulator-05-x2` on `wip/05-x2` at 2e40042 (the coordinator committed X2's work in
progress; built and synced). Brief N5 runs concurrently in the native adapter (a mass change on a rope-hung
payload, which G5 and G6 need) in another worktree. Call cap 150 with the handoff rule (at 135 calls stop adding
scope, commit what is green, report what is left by item). Read first: `docs/briefs/round-05/m3-x2.md` (the brief
you continue: its rules, scope rule and items hold), `test_cases/gantry/README.md` (the evaluation start X2
derived), `git show 2e40042 --stat`, round 5 rulings 41, 63 to 83 in `docs/queue.md`. Same build and test rules
as the X2 brief; commit after each item; final message "WIP M3-X2b: suite on native, G16, retirement check", no
attribution trailer, no push. Do not edit `docs/queue.md` or `docs/cost-log.md`. No conditional (ternary)
expressions. In spec table cells write abs(x), never bare pipe characters. Ownership as the X2 brief.

## Where X2 left it

- Done, uncommitted until 2e40042: `physics.adapter` defaults to `native`; baselines regenerated on native;
  bridge results kept as fixed baselines under `test_cases/{gantry,pendulum}/bridge_baselines/` (PHY-025);
  PHY-021, PHY-023 (the reference tier on native: 94 passed, 4 expected failures that are other modules' stubs),
  PHY-025 and the two PHY-026 tests pass; PHY-024 is a strict expected failure naming the misses.
- Red: eight unit tests still assume the bridge default (`test_core_driver` 2, `test_core_wiring_m2` 1,
  `test_driver_errors` 2, `test_integrity_rigid_runs` 2, `test_native_clamped_members` 1); `tests/acceptance` was
  not run with the native default.
- Cases on native against the bridge: G1 to G4, G8, G11, G13, G14 pass within tolerance; G7 `hook_held` and G9
  `load_held` pass but differ (2.18 mm on the bridge, 1.15 mm on native, tolerance 0.783 mm: the rope payload and
  the compliant lift belt against the bridge's fixed payload and rigid drive); G10 passes but its stall, re-home
  and slip events come 8 to 20 ms earlier (tolerance 3 ms: compliant belts move the encoder-mismatch crossing);
  G5 and G6 error on native (the adapter refuses a mass change on the rope-hung payload: brief N5 fixes it; X2
  moved the acceptance fixture for G5 and G6 to the bridge with that reason written in it); P1, P2 identical.
- The evaluation start is 9 s (five loaded time constants of rail_x, 1.74 s each), applied to G2 to G15; G1 ends
  at 2 s and cannot carry it.

## Rulings for what X2 raised (ruling 83)

- G5 and G6 stay on the bridge in the acceptance fixture only until N5 lands; write them so that removing one
  line returns them to the default adapter, and say in the report that the coordinator re-runs them on native
  after N5 merges. Do not wait for N5.
- G1 ends before the evaluation start: it carries no evaluation start and its integrity results are therefore
  judged from the release; if that makes G1 fail on a tier 1 crossing in the first milliseconds, do not lengthen
  the case or special-case it: report it. If instead the case passes because its 2 s hold never crosses, say so.
- Moving G12 and G15 from 8 s to 9 s is accepted (one derived start for the suite); correct the G12 header's
  flag time and belt figure while you are there, and confirm that G12's and G15's verdicts are unchanged by the
  move (report every criterion).
- PHY-024 stays a strict expected failure listing exactly the misses and their causes: the owner decides at the
  gate whether explained model differences satisfy the retirement criterion (ADR 0004).

## Items, in order

1. Finish item 1 of the X2 brief: the eight unit tests state the adapter they mean (the bridge where they test
   the bridge, otherwise a new expectation for native: say which per test); run `tests/acceptance` with the
   native default and fix what is stale (not what is a finding); commit.
2. Finish item 2: nothing known open; re-run its tests after item 1 and commit.
3. Item 3 of the X2 brief: G16 (ACC-031, 032) and LIB-017 end to end; ACC-038 if the stepper model's curve data
   allows.
4. Item 4: one batch (ACC-036), the long-run marker for the long acceptance tests and CI's use of it (the Python
   workflow excludes the marker by default and one job runs it; keep the Windows job's total under 30 minutes:
   say what each job runs and its expected time).
5. Item 5: `docs/milestones.md` retirement table with evidence.
6. Item 6: the written removal plan.

## Report

Under 300 words: commits, filtered test counts, the eight tests' dispositions, the acceptance tier on native
(passed, expected failures, findings), G16's margins and missed steps against the derivation, the batch time and
what CI runs where, each retirement criterion met or not, open issues by item.
