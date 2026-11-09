# Reference validation: milestone 1

Read-only. Worktree ../MechatronicsSimulator-m1-x2 at commit 1826496. Call cap 12.

Run the analytical reference tier once each:
- C++: `cmake --build --preset debug-sanitize -j2` then `ctest --preset debug-sanitize -L reference -j2 --output-on-failure`
- Python: `uv run pytest -q tests/reference -p no:cacheprovider`
- The MuJoCo bridge pendulum case: `ctest --preset debug-sanitize -R MuJoCo -j2`
- P1 through the real stack: `uv run pytest -q tests/acceptance -k "P1 or pendulum" -p no:cacheprovider`

For every case report the measured value, the closed-form value, the relative error, the spec tolerance (from the
`docs/spec/<module>.md` row the test cites) and pass/fail. Report any test whose asserted tolerance differs from its
spec row (T4). Numbers once, as one table. No code changes.
