# Hands-on test of the tool (owner, after the round 5 close)

Branch `wip/06-human-test`, cut from main at d619725 (PR #7, milestone 3 landed 2026-11-18); fixes from the test land
on this branch. Every command runs from the repository root. The first command
after a pull rebuilds the extension once (about 20 s warm, 5 min cold); later commands do not. Findings go to the
coordinating session in any form; they feed the round 6 plan before the PR to main opens.

## What runs here

| Command | Does | Takes |
|---|---|---|
| `uv run mechsim validate designs/gantry/design.yaml test_cases/gantry/G3.yaml` | Validates files against their schemas; also catalogs, chips, baselines, materials | seconds |
| `uv run mechsim run test_cases/gantry/G3.yaml --design designs/gantry/design.yaml` | One case: builds the firmware for the host, runs on the own engine, prints every approximation line and the verdict per criterion, writes `runs/G3-seed3.h5` | about 2 min for G3 (25 s of motion) |
| `uv run mechsim replay runs/G3-seed3.h5` | The run's event timeline from the archive | seconds |
| `uv run mechsim suite test_cases/gantry --design designs/gantry/design.yaml --report runs/gantry.md` | G1 to G16 in one batch, the pass/fail report per case and criterion as Markdown | tens of minutes (the long tier runs it) |
| `uv run mechsim serve` | Facade service plus the browser client on 127.0.0.1; prints one URL with a token | until stopped (Ctrl-C) |

`--out <folder>` moves archives and reports out of `runs/`. `--dry` configures and schedules without running. Do not
use `--write-baseline`: `test_cases/gantry/baseline.yaml` is the record the acceptance tier compares against.

## Things worth trying

1. **A case by hand.** G3 (move to 40 cm, 25 cm) is the quickest full run. Read the printed lines: each
   `approximation:` line is the engine saying where it is not exact (the E9 belt lines are the known milestone 4
   item); each `pass` or `fail` line gives the measured value beside its limit and when the worst value happened.
2. **Change the design without editing files.** `--set` binds an open parameter or overrides an instance:
   - `--set rated_load="2 kg"` (the open parameters are listed at the top of `designs/gantry/design.yaml`:
     workspace, lift travel, rated load, materials, rail section, policies, friction and damping values);
   - `--set instance:lift_motor.part=stepperonline_17hs19_2004s1` swaps the lift motor for the NEMA 17 that G16 shows
     cannot lift the design load (the run's report then carries the negative torque margin and the missed steps).
   Open parameters bind by name with units; instance overrides use the `instance:<name>.<parameter>` form from
   `mechsim run --help`.
3. **The report.** Run the suite with `--report`, open the Markdown: every case with its criteria, each with the
   measured value beside its limit, and for G16 the torque margin and missed steps per motor as criteria. The
   approximation lines stay in the console output and the archive, not in the shareable report.
4. **The browser client.** Start `uv run mechsim serve`, open the printed URL. The manual checklist from the
   milestone 2 gate is in `web/README.md` (connected state and the design list, run a case from the page, the run
   list, scrubbing, play rates, timeline marks, joint probes, a wrong token rejected). The pendulum
   (`designs/pendulum/design.yaml`, `test_cases/pendulum/`) runs in seconds and is the best first replay.
5. **Determinism.** Run G3 twice into two folders and compare the archives' scoring summaries; the run id carries the
   seed, and the recordings are byte-identical (invariant 4).
6. **Break something on purpose.** Give an open parameter a value outside its range, misspell a part, point a case at
   the wrong design: every failure should end in a report naming the cause (invariant 7), never a traceback.

## Known results, not defects

| Observation | Why |
|---|---|
| G12 fails its settle criterion | No swing damping in the model; derived settle about 1135 s against the 5 s placeholder (milestone 4 ruling M4-7: realistic air drag) |
| G16's NEMA 17 lift fails with margin -0.308 N m and lost steps | The sizing answer the case exists to give; the NEMA 23 passes |
| The lift belt carries 55 to 64 N against a 26 N working tension | Design finding for the gantry team (`docs/gantry-team-inputs.md`) |
| `approximation: transmission ... efficiency loss ... removed up to N times the loss` | E9, the frozen-rate efficiency damping over-brakes a shaft leaving rest; bounded and published, proper fix in milestone 4 |
| `physics.adapter: mujoco` is refused | The bridge was retired on 2026-11-15 (ADR 0004) |
| No STEP or URDF import yet | Milestone 4 (rulings M4-1, M4-2) |
| Sourced parts flagged replaceable | Owner decision 4: datasheet-backed stand-ins where the gantry team has not supplied a value |

## What to report

The command, what was expected, what happened (paste the output), and whether it blocks the PR. Anything about the
report's wording, the browser client's handling, missing commands or confusing output counts: the tool is for the
gantry team, and this is the first use by a person rather than a test.
