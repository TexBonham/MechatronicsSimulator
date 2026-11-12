# Reference validation: milestone 2 gate

reference-validator (Sonnet). Read-only except the build it needs. Checkout `/home/tex/dev/MechatronicsSimulator`
on `round/04-milestone-2` after the X-wave stage lands (the coordinator says when). Run, through
`tools/locked_build.sh core debug-sanitize -L reference` (the reference tier label) and
`tools/locked_build.sh run uv run --no-sync pytest -q tests/reference`, and the firmware host preset's reference
cases if any. Report every tolerance miss with measured value, limit and the spec row, and the pass counts per
tier. Special attention (ruling 33): the bridge's joint-drive mapping changed to exact backward Euler; report the
PHY-034 sag and lag reference numbers, PHY-028 (pendulum, no drive: must be unchanged to the printed digits), and
the G1 to G4 scoring baselines against the committed `test_cases/gantry/baseline.yaml` (the deltas must stay within
one step angle times the ratio per tick of commanded speed, 0.2 mm on x, as the C2 review set; print the deltas).
Under 300 words.
