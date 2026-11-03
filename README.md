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

Python 3.12 and [uv](https://docs.astral.sh/uv/):

```sh
uv sync                        # create .venv from uv.lock
uv run ruff check .            # lint
uv run ruff format --check .   # format check
uv run mypy                    # strict type check of mechsim/ and tools/check_imports.py
uv run pytest                  # Python tests
python3 tools/check_imports.py # import-boundary check (invariants 5 and 9)
```

C++ core (CMake 3.28+, Ninja, a C++20 compiler, clang-tidy):

```sh
cmake --preset debug-sanitize          # Debug, address and undefined-behavior sanitizers, clang-tidy
cmake --build --preset debug-sanitize
ctest --preset debug-sanitize
cmake --preset release && cmake --build --preset release
```

Build trees go to `build/<preset>/`. Catch2 v3 is fetched at configure time.

## Tests

Four tiers (unit, analytical reference, acceptance, invariant) with requirement citations and strict expected-failure
stubs; see [tests/README.md](tests/README.md).

## Decisions

- Architecture decision records: [docs/adr/README.md](docs/adr/README.md)
- Planning rulings that the ADRs close: [docs/rulings/](docs/rulings/)
