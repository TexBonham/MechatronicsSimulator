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
| `members`, `lumped` | structural members (BLD-008: the field brief R records; each with `mounts`, the fixed joints on it); `LumpedRecord` (spans, masses, E I / l of the calibrated section, `calibrated`, recorded calibration, `stiffness_scale`, `mass_scale`) |
| `pending`, `notes` | open-value paths (sorted); non-fatal notes |

`to_json()` is canonical: run-twice byte equality (BLD-026).

## Build order

1. `physics.build_physics`: pending scan, topology (DFS from world), materials, bindings, supports (not rigid);
   stop on pending; bodies (declared mass, else closed forms in `mass.py`), joints, members (section from the design
   or the catalog part, calibrated per BLD-019), contacts, floor, transmissions, ropes, fastenings, overlap;
   validate; supported elements.
2. `wiring.build_wiring`: rails (power_out wires through safety instances), instances, bindings (channel per mode in
   binding order), operator links, chip checks, `channels.h`.

## Files

`build.py` (`build`, `module_order`); `payload.py` (`with_payload(model, name, carrier, mass, fixed=False)`, BLD-036: box body below the carrier, a fixed child `<name>_fixed` of it when `fixed` (attached at t = 0, never released, ruling 24), else free under world and welded by the adapter); `physics.py` (`build_physics`, `primitive_mass`, `lumped_record`,
`element_kinds`, `member_constants`); `mass.py` (closed forms, frames, `z_to`); `structure.py` (`chain_layout`,
`chain_deflection`, `chain_first_frequency`, `calibrate` -> `CalibrationScales`, `load_point`, `fastenings`,
`member_mounts`, `rope_element`, `lumped_section`; the linearized engine chain in hinge angles, numpy/scipy);
`wiring.py`; `result.py`.

| Section | File | Line |
|---|---|---|

## Rules

Neutral quantities are written `repr(value) unit` in SI, validated once. Element kinds: `body`, `joint.<type>`,
`shape.<shape>`, `contacts`, `constraint.<type>`, `lumped_member`, `transmission.<type>`, `rope`.

- Calibration (BLD-019): E scaled by chain deflection / recorded deflection, then rho by (chain omega / recorded
  omega)^2, both exact on the chain; lumped member only. No calibration: built as is, with a note. Pinned-free with
  calibration: build failure.
- Fastenings (BLD-031): `PhysicsModel.fastenings`, one per fixed joint (`mount`, the joint's name) and per body
  fastener entry (`fastener`, `<body>_<kind>[_<index>]`); `PhysicsModel.fastening_index(name)`. Not an element kind.
- Ropes (BLD-032): a design rope with both ends on bodies becomes a neutral `Rope`; a `to` end at a case payload is
  a note, not an element (X1 places it).
- Reference tests: `tests/reference/lumped_beam_support.py` steps the built lumped member on `_engine` (tests only).

## Later

Loops as constraints (BLD-004), clamped-clamped beams, ropes to a case payload (X1).

## Section index

| Section | File | Line |
|---|---|---|
| Checks | physics.py | 83 |
| Bodies and joints | physics.py | 194 |
| Overlap | physics.py | 320 |
| Lumped members | physics.py | 376 |
| Transmissions and signals | physics.py | 517 |
| Physics model | physics.py | 653 |
