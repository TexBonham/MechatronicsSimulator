# Review: M2-F1 (facade service, viewer module, conformance suite)

Reviewer: Fable (general-purpose). Read-only; spec rows and diff only; never the writer's report; no builds or
tests (one machine build slot; the writer's full run is on record); call cap 40. Worktree
`/home/tex/dev/MechatronicsSimulator-04-f1`, branch `wip/04-f1`, commit 112de83, diff
`git -C <worktree> diff round/04-milestone-2...wip/04-f1`. Brief of record `docs/briefs/round-04/m2-f1.md`. Spec rows
FAC-001, 003, 005 to 012, 014 to 020 in `docs/spec/fac.md`; VIE-002 to 009, 014 to 017 in `docs/spec/vie.md`;
TST-034 in `docs/spec/tst.md`; ADR 0006, ADR 0007, ADR 0015; invariant 9 (root `CLAUDE.md`), `tools/CLAUDE.md`
import rules. Contracts: `mechsim/facade/CLAUDE.md`, `mechsim/viewer/CLAUDE.md` (both in the diff),
`mechsim/record/CLAUDE.md`, `mechsim/testing/CLAUDE.md`.

Check:
1. Every row implemented as written and cited; spec edits in the same diff (R-18). The protocol: additive request
   kinds only, schema regenerated, `extra="forbid"`, version checked per message; the writer mapped the brief's
   error-code names onto the protocol's existing codes (unsupported_version, invalid_message, validation_failed,
   busy): say whether the mapping is complete against FAC-020.
2. Streaming and decimation (FAC-006, 007, 018, 019): tick batches sourced from `Recorder.subscribe`, binary frame
   layout and byte-length check, the min-max bucket rule, and the assertion that the recording is unchanged with a
   subscriber. FAC-015: run through the facade byte-identical to the CLI's archive.
3. Threading and determinism: the run worker thread, how events reach the asyncio loop, no wall-clock in any
   message, server shutdown closing every connection; the token check on the handshake.
4. Viewer: probes by `RecordView` refs, timeline categories and labels verbatim, geometry from the archived design
   bytes with no file written (VIE-017), the replay cursor as pure state (scrub to the sample at or before t, step
   both ways, pause). VIE-016: the writer reports that `RecordArchiveReader` lives in `mechsim/testing/core_driver.py`
   which imports `mechsim._core`, so a real replay needs the extension and VIE-016 passes only with a fake reader.
   Judge this against REC-024 and VIE-016 and say what the fix is (an archive reader under `mechsim/record/` that
   `core_driver` re-exports, or another seam).
5. Invariant 9 and the import rule: the writer allowed `mechsim.viewer` under `mechsim/facade/` in
   `tools/check_imports.py` (VIE-014 requires the facade to serve the viewer's values); `tests/unit/test_check_imports.py`
   still forbids it and fails (coordinator ruling 15 allows it; the coordinator updates that test at merge-back).
   Confirm no Qt, OpenGL or display import appears anywhere outside `mechsim/viewer/` and none inside it either.
6. CLI through the facade only (TST-034, FAC-001) with output text unchanged; `run_suite` payload fields the writer
   says are ignored (seed, mode): is that a defect against FAC-010?
7. Ownership and style (no ternaries, braces not applicable, no comments without a why, every edge case tested).

Report, under 400 words: verdict, findings by severity as `file:line, one sentence, spec row`, owner rulings
needed, cost note.
