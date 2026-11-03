# tools/: contract

Stdlib Python checkers (3.12 floor), run by the tests and `tools/hooks/`.

## check_imports.py

`python3 tools/check_imports.py [--root ROOT] [paths...]`. Parses with `ast`, dynamic imports included. Exit 1 on any violation, else 0.

Rules (`RULES`):
- `mujoco`: only under `mechsim/physics/mujoco/` (invariant 5)
- Qt, PySide, PyQt, OpenGL, tkinter, vispy, glfw, pygame, pyqtgraph: only under `mechsim/viewer/` (invariant 9)
- `hppfcl`, `fcl`, `pycollision`: only under `tests/` (R-05)
- `mechsim.viewer`: only under `mechsim/viewer/` and `tests/` (invariant 9)
- `tests`: never under `mechsim/`

## traceability.py

`python3 tools/traceability.py [--write] [--root PATH]`. Checks spec IDs against Python and C++ citations and compares the generated `docs/spec/traceability.md`.

- `--write`: regenerate the matrix instead of comparing
- `--root PATH`: repository root (default: parent of `tools/`)
- Exit 0: clean. Exit 1: violations, or a stale matrix without `--write`. Exit 2: usage error.

## Section index, `traceability.py`

| Section | File | Line |
|---|---|---|
| Data model | `tools/traceability.py` | 40 |
| File helpers | `tools/traceability.py` | 89 |
| Spec parsing | `tools/traceability.py` | 129 |
| Python citations | `tools/traceability.py` | 240 |
| C++ citations | `tools/traceability.py` | 334 |
| Checks | `tools/traceability.py` | 473 |
| Matrix | `tools/traceability.py` | 518 |
| CLI | `tools/traceability.py` | 592 |
