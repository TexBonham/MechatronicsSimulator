# Review: M2-W and M2-W2 (Windows joins CI, native runtime on Windows)

Reviewer: Fable (general-purpose). Read-only; diff and spec only; never the writer's reports; no builds or tests;
call cap 30. Worktree `/home/tex/dev/MechatronicsSimulator-04-w`, branch `wip/04-w`, head 12a171c; diff
`git -C <worktree> diff round/04-milestone-2...wip/04-w` (note: the round branch moved after W branched; diff
against the merge base, `git merge-base`, so only W's own changes show). Briefs of record
`docs/briefs/round-04/m2-w.md` and `m2-w2.md`; ruling R1 in `docs/rulings/2026-11-02-planning-rulings.md`; open
rulings 3 and 5 in `docs/queue.md`; spec rows INV-020 (new) in `docs/spec/inv.md`, FW-006 to FW-008 in
`docs/spec/fw.md`, CTL-005, 007, 012 in `docs/spec/ctl.md`, PHY-011 in `docs/spec/phy.md`, REC-014, 022 in
`docs/spec/rec.md`. Contracts: `core/controller/CLAUDE.md`, `core/CLAUDE.md`, root `CLAUDE.md`.

Check:
1. Platform code: the `_WIN32` branches in `core/controller/src/host_build.cpp` (CreateProcessW, pipe capture,
   exit code, quoting of the command line) and `native_controller_runtime.cpp` (LoadLibraryW, GetProcAddress,
   FreeLibrary, per-load private copy preserved, error text), and that the POSIX path is byte-identical except the
   shared `native_library_name` helper. Resource handling: every HANDLE closed on every path, no raw owning
   pointers across an API, RAII as the dlopen path has it.
2. `core/cmake/MuJoCo.cmake`: Windows zip hash pinned, DLL placed beside the test binaries and inside the wheel the
   same way the Linux build ships `libmujoco.so`; FATAL_ERROR kept for other platforms (ruling 5).
3. MSVC flags: `/W4 /WX /permissive-` and every `/wd` exclusion has a one-line reason; `FirmwareWarnings.cmake`
   MSVC branch and the ruling 3 reading of FW-008 recorded in its header; `/std:c11`.
4. `core/kernel/src/time.cpp` `_mul128` overflow check: correct for every sign combination and the INT64_MIN
   edge; the GCC path unchanged.
5. `core/bindings/src/errors.cpp` include order fix and `tests/invariant/test_schema_test_case.py` `as_posix()`
   are outside the ownership list; say whether each is the minimal behaviour-free fix.
6. CI: the three workflows' Windows jobs run the same tiers as Linux (cpp: both presets and ctest; python: ruff,
   mypy, full pytest with the MSVC environment active; firmware: host preset), clang-tidy Linux only; the
   `.gitattributes` rule and whether REC-014's blob hash is stable on a Windows checkout; INV-020 row and its tests.
7. README Windows paragraph accuracy and the macOS sentence corrected.
8. Invariants 4, 5, 8 and style.

Report, under 350 words: verdict (accept, accept with fixes, blocked), findings by severity as `file:line, one
sentence, spec row`, owner rulings needed, cost note.
