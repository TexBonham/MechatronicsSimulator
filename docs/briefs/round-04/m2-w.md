# M2-W: Windows joins CI (R1)

Agent: general-purpose (Opus). Worktree `../MechatronicsSimulator-04-w` on branch `wip/04-w`, from
`round/04-milestone-2`. This brief is verified on GitHub Actions, not on this machine: do not run any C++ build,
`uv sync` or pytest locally (the machine allows one build at a time and another brief holds it). Push `wip/04-w`
and read CI with `gh run list --branch wip/04-w`, `gh run watch <id>`, `gh run view <id> --log-failed`.

Read first: root `CLAUDE.md`, `README.md` (Dependencies and First run), `.github/workflows/cpp.yml`,
`python.yml`, `firmware.yml`, `CMakePresets.json`, `firmware/CMakePresets.json`, `core/cmake/MuJoCo.cmake`,
`pyproject.toml` (`[build-system]`, `[tool.scikit-build]`, `[tool.uv]`), `firmware/cmake/FirmwareWarnings.cmake`,
`firmware/CLAUDE.md`, ruling R1 in `docs/rulings/2026-11-02-planning-rulings.md`, open rulings 3 and 5 in
`docs/queue.md`. Call cap 120; each CI round is slow, so batch fixes per push. Commit as you go on `wip/04-w` with
messages starting "WIP M2-W:"; pushing is required for this brief and allowed only on `wip/04-w`. Never open a pull
request. Never touch main or `round/04-milestone-2`.

## Ownership

`.github/workflows/cpp.yml`, `python.yml`, `firmware.yml`, `CMakePresets.json` (new Windows presets only; do not
change the existing `debug`, `debug-sanitize`, `release` presets), `firmware/CMakePresets.json` (same rule),
`core/cmake/MuJoCo.cmake`, `.gitattributes` (new), `pyproject.toml` build sections only, `README.md` Dependencies and
First run sections, `docs/spec/inv.md` (a row for the matrix if INV rows cover CI), `core/CMakeLists.txt` and
`core/**/CMakeLists.txt` and `firmware/CMakeLists.txt` only for compiler-conditional flags. Source fixes for MSVC
compile errors are allowed anywhere under `core/` and `firmware/` and `mechsim/` but must be minimal and listed one
per line in the report; a fix that changes behaviour stops the brief (list it under Open issues).

## Goal

The three workflows run on `windows-2022` as well as `ubuntu-24.04`, green, with the same tiers Linux runs:

| Workflow | Windows job |
|---|---|
| cpp | MSVC (`ilammy/msvc-dev-cmd@v1` or the VS developer shell), Ninja, presets `windows-debug` (Debug, no sanitizers: MSVC has no UBSan) and `windows-release`; ctest both; clang-tidy stays Linux only |
| python | `uv sync --locked` builds `mechsim._core_ext` with MSVC; ruff, mypy, `uv run pytest` (the acceptance tier builds the gantry firmware host library at run time per CTL-007, so the MSVC environment must be active in the pytest step too) |
| firmware | `host` preset under MSVC; `arm-none-eabi` stays Linux only |

## Items

1. `core/cmake/MuJoCo.cmake`: Windows x86_64 branch fetching `mujoco-3.9.0-windows-x86_64.zip`, SHA-256 pinned
   (download it once with `curl -L` and `sha256sum`; put the hash in the file; no system package). Imported target
   over `lib/mujoco.lib` and `bin/mujoco.dll`; the DLL must sit beside the test executables and the Python extension
   (copy at build, or a `RUNTIME` install rule into the wheel: the Linux build already ships `libmujoco.so` in the
   wheel, mirror that path). Keep the FATAL_ERROR for every other platform (ruling 5: macOS out).
2. Presets: `windows-debug`, `windows-release` configure, build and test presets with
   `"condition": {"type": "equals", "lhs": "${hostSystemName}", "rhs": "Windows"}`, `jobs: 2` like the others,
   `CMAKE_BUILD_TYPE` as named, warnings as errors through whatever `core/CMakeLists.txt` already does for GCC and
   Clang, extended with the MSVC equivalent (`/W4 /WX /permissive-`, plus the `/wd` exclusions you need, each with a
   one-line reason in the CMake file). Firmware: `host` must configure under MSVC; `FirmwareWarnings.cmake` gets an
   MSVC branch (`/W4 /WX`, C11 as `/std:c11`); open ruling 3 is the reading of FW-008 that allows this; record it in
   the FirmwareWarnings file header.
3. `.gitattributes`: `* text=auto eol=lf` so YAML, Python, C and the design file hash identically on Windows
   checkouts (REC-014 uses the design file's git blob hash; CRLF would change it). Verify the python job's
   determinism and identity tests pass on Windows.
4. `python.yml`: the Windows job mirrors the Linux steps; cache uv; the MSVC environment active for `uv sync` and
   pytest. If h5py or another wheel needs a system library on Windows, document it in the README table.
5. README: Dependencies table gains the Windows column or a short Windows paragraph (Visual Studio 2022 Build Tools
   with the C++ workload, CMake 3.28+, Ninja, uv; no WSL needed for the native build); correct the sentence that
   names macOS as a milestone 2 CI item (Windows only, ruling 5). Keep the Linux content as is.
6. Known hazards to check before the first push: `$TMP/mechsim-native-build` host-build path on Windows
   (`mechsim/testing/core_wiring.py` builds the firmware host library per process; it must find `cmake` and the MSVC
   compiler from the pytest environment); path separators in any test that compares strings; `flock` is not used in
   CI. Do not edit `core_wiring.py` beyond a minimal path fix; if it needs more, stop and report.

## Done when

All three workflows green on `wip/04-w` for both runners, run twice (the second run proves the MuJoCo hash and
caches hold). Report the run URLs, the MSVC source fixes one per line, and anything you could not make green with
the failing step's log excerpt.

## Report format

```
Files touched: <path> (new|edited), ...
CI runs: <url> -> <result> ...
MSVC source fixes: <path:line one-line reason> ... | none
Open issues: <one line each> | none
```
