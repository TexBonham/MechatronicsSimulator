# Gate: reference tier on both adapters (milestone 3)

Agent: reference-validator (Sonnet). Read-only; call cap 16. Checkout `/home/tex/dev/MechatronicsSimulator` on
`round/05-milestone-3` at LANDING_HEAD (the build exists; if `build/debug-sanitize/CMakeCache.txt` is missing, run
the configure once). Builds and runs in the foreground through `tools/locked_build.sh` with a 600000 ms timeout:
1. C++ engine reference tier: `tools/locked_build.sh core debug-sanitize -L reference` (one call; the rope pendulum
   references take up to 16 minutes each on a loaded machine: if the call times out, rerun with `-R` by name in
   pieces until every reference-labelled test has run once).
2. Python reference tier on the default adapter (native): `tools/locked_build.sh run uv run pytest tests/reference -q -p no:cacheprovider`.
3. Python reference tier on the bridge, where the tests are parametrized by adapter: the same command with
   `-k mujoco` (if the tier selects the adapter another way, read `tests/reference/conftest.py` and use it;
   PHY-023's enumeration is the acceptance tier's, not yours).
Report per the agent's format: every case one line (ID, closed form, measured, tolerance, pass or miss), misses
first with the spec row, the two Python runs' counts and times, the C++ count, any determinism difference.
