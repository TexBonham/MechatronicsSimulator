# ADR 0011: Firmware language, build, polled native tick, HAL placement

Date: 2026-11-02
Status: Accepted
Closes rulings: C1, C2, C3, C4, C5, C6
Supersedes: Architecture doc, Technology recommendation, "Native mode" paragraph in part (shared library loaded through
cffi; the native shim now sits in the core runtime, ADR 0005)

## Context
Decisions log rows 3, 4 and 8 make the controller the real firmware, run natively behind a board-interface shim or
emulated on Renode, against a simulator-owned board interface. The architecture doc's Native mode paragraph and
invariant 8 set the rules for keeping native mode honest but left language, build, tick semantics, HAL placement and the
Renode transport open. Firmware lives in this repo under `firmware/` (R-02).

## Decision

| # | Aspect | Decision |
|---|---|---|
| C1 | Language | C11 for firmware application code. C++ only inside a module, behind a C interface. The board interface is a C header. |
| C2 | Build | CMake with host and arm-none-eabi presets, identical warning flags, warnings as errors. The host preset fails on any include outside the board interface. |
| C3 | Native tick contract | Init and tick entry points called at a fixed rate. Inputs and outputs cross as a plain C struct. Native mode is polled: accumulated counts, latched levels, no interrupts. Interrupt semantics exist only in emulated mode. |
| C4 | Rates | 1 kHz control tick as the gantry default, equal to the physics step. The kernel supports unequal rates. |
| C5 | Vendor HAL | Allowed only under `firmware/board/<chip>/`. |
| C6 | Renode transport | External Control socket versus pyrenode3 is deferred to a milestone 4 ADR. The C client library is favored, for direct use from the core runtime. |

## Consequences
- Invariant 8 is enforced by the build: the host preset rejects any include outside the board interface.
- The native shim in `core/controller/` calls the firmware's init and tick entry points directly; no cffi layer.
- Firmware must be correct under polled semantics in native mode; behavior that depends on interrupt timing is verified
  only in emulated mode (milestone 4).
- The 1 kHz rate is a gantry design default held in data, not a code constant (invariant 2); other designs may run
  control and physics at unequal rates.
- Host and target builds share warnings, narrowing the host-versus-Cortex-M risk in the architecture doc's Risks table.

## Open items
- Renode transport: milestone 4 ADR.
- STM32 part: owed by the gantry team after the peripheral smoke test.
