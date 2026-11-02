# ADR 0005: C++ core runtime, Python application layer, tiered plugins

Date: 2026-11-02
Status: Accepted
Closes rulings: R-08, R-09, R-14
Supersedes: Architecture doc, Technology recommendation, "Language" paragraph (Python for the kernel and signal layer);
"Native mode" paragraph, loading the host build through cffi; Repository starting point, `mechsim/kernel/`,
`mechsim/signals/` and `mechsim/controller/`

## Context
The architecture doc put every tool-owned module in Python, with the signal layer movable to C++ if it became a
bottleneck. With an own C++ engine (ADR 0001), the per-tick path (physics, kernel, signal layer, controller runtimes)
would otherwise cross the binding several times per tick. The rest of the tool is data handling and orchestration, where
Python's development speed matters more.

## Decision
Core runtime (R-08): one compiled library, exposed through one Python binding (pybind11).

| Part of the core runtime | Location |
|---|---|
| Physics engine | `core/engine/` |
| Co-simulation kernel | `core/kernel/` |
| Signal-layer built-in components | `core/signals/` |
| Controller runtimes: native shim and Renode bridge | `core/controller/` |
| Python binding | `core/bindings/` |

Component plugin tiers (R-09):

| Tier | Language | Path |
|---|---|---|
| Built-in types | C++ | Inside the core runtime |
| User-added types, first | Python | Per-tick slow path through the binding |
| User-added types, when stable | C++ | Promoted into the core runtime |

Both paths implement the same interface and pass the same reference tests.

Python application layer (R-14), in `mechsim/`: component catalog, materials, design loading, CAD import, builder, test
runner orchestration, mechanical integrity, scoring, exports, facade service.

## Consequences
- The tick loop runs inside one compiled library; Python enters per tick only for slow-path plugins.
- A plugin's promotion from Python to C++ must not change its results: the shared reference tests are the gate, and
  run-twice determinism (invariant 4) covers both paths.
- ADR 0001's C++ gates (sanitizers, clang-tidy, warnings as errors, no raw owning pointers across APIs) apply to every
  C++ target in `core/`, not only the engine.
- Milestone 1 now depends on the kernel and native runtime being built in C++; mitigation is to build them concurrently
  with the engine packages while the MuJoCo adapter (ADR 0004) carries milestone 1. The risk is tracked in the
  architecture doc's Risks table.
- The Renode bridge sits in the core runtime, so the Renode C client library is used directly from C++ (ADR 0011, C6).

## Open items
- The plugin interface itself: phase 1 contracts.
- Criteria for promoting a plugin from Python to C++ beyond "stable": not ruled.
