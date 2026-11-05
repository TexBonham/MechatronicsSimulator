# mechsim/testing: history

## 2026-11-05: Round 3 (milestone 1) close sweep

- Runner paragraph: "The real driver, builder, archiver and reader are wired at integration" removed. Stale:
  `core_driver.py` (`CoreDriver`, `RecordingView`, `RecordArchiver`, `RecordArchiveReader`) and `core_wiring.py`
  are in place. The test-fake pointer stays in the contract.
