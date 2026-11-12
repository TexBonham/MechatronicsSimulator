# MechatronicsSimulator

A desktop simulator that runs a mechatronic design's real firmware against a physics model of its structure, actuators
and sensors, and scores the result against test cases. Engineers use it to size hardware, check structures and prove
controller logic before building anything.

## Repository layout

```text
core/                 C++ core runtime: one compiled library, own CMake tree
  engine/             own physics engine: math core, integrators, multibody, collision, contact
  kernel/             co-simulation kernel: time, rates, sync points, seeds
  signals/            built-in signal-layer components: sensors, front-ends, rails, safety hardware, operator link, pin bindings
  controller/         controller runtimes: native shim, Renode bridge
  bindings/           pybind11 binding exposing the core runtime to Python
  viewer_debug/       minimal immediate-mode C++ window for engine development
mechsim/              Python package; one subpackage per module
  library/            component types, plugin interface, shipped catalog (YAML)
  materials/          materials database, process rules, direction-dependent limits
  design/             design-file schema, loader, validation, CAD import (STEP, URDF)
  builder/            model builder: design file -> engine-neutral physics model, wiring, chip config
  physics/            adapter package: mujoco/ (bridge), native/ (own engine binding), chrono/ (reserved)
  testing/            test-case schema, runner, fault fan-out, sweeps, regression
  record/             recorder, run archive (HDF5), CSV/JSON/MAT exports
  integrity/          mechanical integrity check plugins, safety factors, load-case export
  scoring/            pass criteria, suite reports, regression diffs
  facade/             facade service and protocol
  viewer/             replay and event timeline, served to clients through facade/
board_interface/      the C header firmware writes against
chips/                chip definitions: platform description, SVD, pin-map template
schemas/              JSON Schema for every YAML file type, generated from the pydantic models
designs/              design files; designs/gantry/ is the reference design, designs/pendulum/ the second
test_cases/           test suites; test_cases/gantry/ holds G1 to G16
firmware/             controller source trees in this repo; vendor HAL only under firmware/board/<chip>/
web/                  browser client
tests/                the simulator's own unit and reference tests
docs/                 architecture and requirements docs
  adr/                architecture decision records
  spec/               one spec per module, numbered requirements
  rulings/            planning rulings, the handoff records for ADRs
  queue.md            the dispatch queue and next brief
  cost-log.md         session cost log
```

## Setup

The stack builds and runs on Linux x86_64: MuJoCo 3.9.0 is pinned to its Linux x86_64 release archive and the
extension loads `libmujoco.so` beside itself (`core/cmake/MuJoCo.cmake`). On Windows, WSL2 with Ubuntu 24.04 (clone
inside the Linux filesystem) follows the Linux steps unchanged; the native Windows build is the milestone 2 CI item
(R1, Windows only; macOS stays out, open ruling 5) and is described below the table. The versions below are the ones
milestone 1 was gated on.

| Dependency | Version used | Install (Ubuntu 24.04, WSL2 included) |
|---|---|---|
| Git | any recent | `sudo apt install git` |
| C++20 compiler | GCC 13.3 | `sudo apt install build-essential` |
| CMake | 3.28.3 (3.28 minimum) | `sudo apt install cmake` |
| Ninja | 1.11.1 | `sudo apt install ninja-build` |
| clang-tidy | 18 (Ubuntu 24.04 default) | `sudo apt install clang-tidy` |
| Python | 3.12 or newer (3.13 in CI) | `sudo apt install python3 python3-venv` or let uv manage it |
| Python headers (`Python.h`, pybind11 extension build) | matching the interpreter | `sudo apt install python3-dev` (uv-managed Pythons ship them) |
| uv | 0.11 or newer | `curl -LsSf https://astral.sh/uv/install.sh \| sh` |
| arm-none-eabi GCC (optional, cross preset only) | any 12+ | `sudo apt install gcc-arm-none-eabi` |

Native Windows x86_64 (no WSL needed): Visual Studio 2022 Build Tools with the "Desktop development with C++"
workload (MSVC, plus the CMake 3.28+ and Ninja it bundles), Git and uv. Run every command below from a "Developer
PowerShell for VS 2022" (or after `vcvars64.bat`) so `cl`, `cmake` and `ninja` are on `PATH`; that includes `uv run
pytest`, because the acceptance tier builds the firmware host library at run time (CTL-007). Use the presets
`windows-debug` (Debug, no sanitizers: MSVC has no UBSan; no clang-tidy) and `windows-release` in place of
`debug-sanitize` and `release`, and set `CMAKE_GENERATOR=Ninja` before `uv sync`. MuJoCo comes from its pinned Windows
release archive and `mujoco.dll` is shipped beside the extension. `.gitattributes` keeps LF line endings on checkout, so
the design file's git blob hash (REC-014) matches Linux. The h5py, numpy and scipy wheels are self-contained on Windows;
no system library is needed.

Fetched at configure time, so the first build needs network access to github.com and PyPI: pybind11 v3.0.1, Eigen
3.4.0, Catch2 v3.7.1, MuJoCo 3.9.0 (SHA-256 pinned), and the Python packages in `uv.lock` (numpy, scipy, pydantic 2,
ruamel.yaml, pint, h5py, jsonschema, ruff, mypy, pytest). Nothing is installed system-wide by the build.

First run, in order:

```sh
git clone https://github.com/TexBonham/MechatronicsSimulator.git
cd MechatronicsSimulator
uv sync                                  # .venv from uv.lock, then builds mechsim._core_ext (about 5 min cold)
cmake --preset debug-sanitize            # Debug, ASan, UBSan, clang-tidy, warnings as errors
cmake --build --preset debug-sanitize -j2
ctest --preset debug-sanitize -j2        # 257 cases at the milestone 1 gate
cd firmware && cmake --preset host && cmake --build --preset host && ctest --preset host && cd ..   # 62 tests
uv run pytest -q                         # 2909 passed, 282 strict-xfail stubs at the gate
```

`uv sync` compiles the C++ extension through scikit-build-core, so the toolchain above must exist before it runs;
it rebuilds on its own when C++ sources or CMake files change (`[tool.uv] cache-keys`), and
`uv sync --reinstall-package mechsim` forces one. Use `-j2` on a shared or small machine: clang-tidy is
memory-hungry and a parallel build has been killed at four jobs on a 15 GB host.

Everyday commands:

```sh
uv run ruff check . && uv run ruff format --check .   # lint and format
uv run mypy                                           # strict types, mechsim/ and tools/check_imports.py
python3 tools/check_imports.py                        # import boundaries (invariants 5 and 9)
python3 tools/traceability.py --write                 # regenerate docs/spec/traceability.md after spec or test edits
uv run mechsim run test_cases/gantry/G3.yaml --design designs/gantry/design.yaml
uv run mechsim suite test_cases/gantry --design designs/gantry/design.yaml
cmake --preset release && cmake --build --preset release
```

Build trees go to `build/<preset>/`; the Python extension builds under `build/python/`; the native runtime builds
the design's firmware for the host once per process under the system temp directory
(`mechsim-native-build/<target>-<hz>-<tree hash>`). Run archives (`*.h5`) are never committed.

Browser client: `uv run mechsim serve` starts the facade service on 127.0.0.1 and prints one line,
`http://127.0.0.1:<port>/?token=<token>`; open it in a browser to list designs, run a case or open a run archive
(`runs/<run id>.h5`), and replay it in 3D with scrubbing, the event timeline and probes. The page and its WebSocket
share the port; paths are relative to `--files-root` (default the repository root). No build step: `web/` is plain ES
modules with three.js vendored (`web/README.md`).

Working with Claude Code on this repository: the project's agents, commands and hooks are in `.claude/` and come
with the clone; the global workflow rules live in the `claude-config` repository (its `CLAUDE.md` is the user-level
`~/.claude/CLAUDE.md`). Agent worktrees are created beside the clone as `../MechatronicsSimulator-<brief id>`, so
clone into a folder of its own.

## Tests

Four tiers (unit, analytical reference, acceptance, invariant) with requirement citations and strict expected-failure
stubs; see [tests/README.md](tests/README.md).

## Decisions

- Architecture decision records: [docs/adr/README.md](docs/adr/README.md)
- Planning rulings that the ADRs close: [docs/rulings/](docs/rulings/)
