# firmware/: controller source trees

Real firmware, the controller itself (Decisions log rows 3, 4; R-02). Rules per ADR 0011.

## Rules

| # | Rule |
|---|---|
| C1 | C11 for application code. C++ only inside a module behind a C interface |
| C1 | Board interface is a C header in `board_interface/` (repo root) |
| - | Fixed-width integer types (architecture doc Risks: host vs Cortex-M) |
| 8 | Application code includes nothing outside `board_interface/`: no simulator header, no vendor HAL (invariant 8) |
| C5 | Vendor HAL only under `firmware/board/<chip>/` |
| - | Curly braces on all control flow; no ternary operators |

## Board interface relationship

The simulator defines the board interface; nothing in the firmware depends on a HAL (Decisions log row 8). Native mode: firmware built for the host behind the board-interface shim in `core/controller/`, which calls init and tick directly, no cffi. Emulated mode: the compiled .elf runs on Renode (milestone 4).

## Build (C2)

| Preset | Target | Notes |
|---|---|---|
| host | Host build for native mode | Fails on any include outside the board interface |
| arm-none-eabi | Cortex-M target | Same warning flags as host |

Warnings as errors in both. Preset names in `CMakePresets.json` follow 0c scaffolding.

## Native tick contract (C3)

| Aspect | Contract |
|---|---|
| Entry points | `init` and `tick`, called at a fixed rate |
| Data | Inputs and outputs cross as one plain C struct |
| Semantics | Polled: accumulated counts, latched levels, no interrupts |
| Interrupts | Emulated mode only; interrupt-timing behavior verified there |
| Rate | 1 kHz gantry default, equal to the physics step; held in design data (C4) |

## Where code lives

| Path | Holds |
|---|---|
| `firmware/<controller>/` | A controller source tree; the gantry controller is the first (folder name not set) |
| `firmware/board/<chip>/` | Vendor HAL for a chip; STM32 part not set (owed by gantry team) |

Tests tag requirement IDs with prefix FW.

## Contracts

Empty until phase 1.
