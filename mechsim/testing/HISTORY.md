# mechsim/testing: history

## Round 5 (2026-11-16): round-close sweep

- Moved from `CLAUDE.md` (integrity wiring not yet built):
> (none until I1)

- Moved from `CLAUDE.md` (retired bridge baselines):
> (none now; the fixed bridge baselines keep `-mujoco`)

## 2026-11-05: Round 3 (milestone 1) close sweep

- Runner paragraph: "The real driver, builder, archiver and reader are wired at integration" removed. Stale:
  `core_driver.py` (`CoreDriver`, `RecordingView`, `RecordArchiver`, `RecordArchiveReader`) and `core_wiring.py`
  are in place. The test-fake pointer stays in the contract.
