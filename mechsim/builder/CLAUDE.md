# builder/: contract

Model builder (BLD, ADR 0003). Failures raise `BuildError(failures)` listing all; nothing partial (BLD-025).

`build(design, registry, materials, *, chips=None, supported_elements=None, adapter="any", step_importer=None,
base=None, blob=None, rigid_members=False) -> BuildResult`. `supported_elements` None accepts all; `blob` defaults
to source_hash, zeros when unset; `rigid_members` builds flexible bodies rigid, member list kept (BLD-033).

## BuildResult (`result.py`)

| Field | Holds |
|---|---|
| `design`, `module_instances` | the Design; instances in KER-009 tick order (`build.module_order`, D1 seeding) |
| `physics` | neutral `PhysicsModel`; None for a draft with open physics (run step stays the 1 ms default) |
| `wiring` | instances (parameters, emits, consumes, mounts, rails, `senses` `Sensing(joint, payload, joint_reaction, body, position, actuate_above, reversed)`, `acts_on` joint, `torque_speed` ((rad/s, N m), ...) from the part curve; BLD-035), `core_type` (the type's `plugin.cpp`), `length_parameters` (quantity parameters of dimension [length]), bindings (pin, mode, channel), rails, gating, operator_links |
| `chip` | name, placeholder, bindings, `channels_header` (`channels.h`) |
| `members`, `lumped` | structural members; `LumpedRecord` (spans, masses, E I / l, calibration) |
| `pending`, `notes` | open-value paths (sorted); non-fatal notes |

`to_json()` is canonical: run-twice byte equality (BLD-026).

## Build order

1. `physics.build_physics`: pending scan, topology (DFS from world), materials, bindings, supports (not rigid);
   stop on pending; bodies (declared mass, else closed forms in `mass.py`), joints, members, contacts, floor,
   transmissions, overlap; validate; supported elements.
2. `wiring.build_wiring`: rails (power_out wires through safety instances), instances, bindings (channel per mode in
   binding order), operator links, chip checks, `channels.h`.

## Files

`build.py` (`build`, `module_order`); `payload.py` (`with_payload(model, name, carrier, mass, fixed=False)`, BLD-036: box body below the carrier, a fixed child `<name>_fixed` of it when `fixed` (attached at t = 0, never released, ruling 24), else free under world and welded by the adapter); `physics.py` (`build_physics`, `primitive_mass`, `lumped_record`,
`element_kinds`); `mass.py` (closed forms, frames, `z_to`); `wiring.py`; `result.py`.

| Section | File | Line |
|---|---|---|

## Rules

Neutral quantities are written `repr(value) unit` in SI, validated once. Element kinds: `body`, `joint.<type>`,
`shape.<shape>`, `contacts`, `constraint.<type>`, `lumped_member`, `transmission.<type>`.

## Milestone 2 and later

Loops as constraints (BLD-004), fasteners as constraints (BLD-031), ropes (BLD-032), lumped calibration (BLD-015
to 019), mounts in the member list (BLD-008), clamped-clamped beams.

## Section index

| Section | File | Line |
|---|---|---|
| Checks | physics.py | 80 |
| Bodies and joints | physics.py | 191 |
| Overlap | physics.py | 317 |
| Lumped members | physics.py | 373 |
| Transmissions and signals | physics.py | 444 |
| Physics model | physics.py | 580 |
