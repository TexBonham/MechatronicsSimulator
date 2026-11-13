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
| `payload_ropes` | neutral `Rope` of each design rope ending at a case payload (`to_body` = the payload's name, `to_point` = the design's point, from the payload's top face centre); empty without physics (BLD-032) |

`to_json()` is canonical: run-twice byte equality (BLD-026).

## Build order

1. `physics.build_physics`: pending scan, topology (DFS from world), materials, bindings, supports (not rigid);
   stop on pending; bodies (declared mass, else closed forms in `mass.py`), joints, members (section from the design
   or the catalog part, calibrated per BLD-019), contacts, floor, transmissions, ropes, fastenings, overlap;
   validate; supported elements.
2. `wiring.build_wiring`: rails (power_out wires through safety instances), instances, bindings (channel per mode in
   binding order), operator links, chip checks, `channels.h`.

## Files

`build.py` (`build`, `module_order`); `payload.py` (`with_payload(model, name, carrier, mass, fixed=False, rope=None, starts=None)`, BLD-036: box body below the carrier, a fixed child `<name>_fixed` of it when `fixed` (attached at t = 0, never released, ruling 24), else free under world and welded by the adapter; with `rope` (BLD-032) a shackle `<name>_shackle` (`SHACKLE_MASS` 1 g, no shapes) on `<name>_fixed` at the rope's from point, the payload free on `<name>_free` with its top face centre the rest length below it at the `starts` pose, the rope from the shackle to that point, never welded); `physics.py` (`build_physics`, `primitive_mass`, `lumped_record`,
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
  omega)^2, both exact on the chain; lumped member only; a scale outside [0.5, 2] adds an approximation note. No
  calibration: built as is, with a note. Calibrated below `structure.calibration_min_segments` (8; 16 clamped-clamped, rulings 20, 32) or pinned-free: build failure.
  Clamped-clamped with calibration: a note, calibration chain layout owed (ruling 16). Segments: the design's, else
  `structure.default_segments` (8; 16 clamped-clamped, BLD-018), always written to the neutral member. Supports:
  root clamped or pinned, end free, roller or clamped (a clamped end needs a clamped root). Rigid builds calibrate nothing.
- Carried joints (BLD-037, ENG-052): `flexible.carries` joints become `LumpedMember.carries` (their child bodies);
  `_check_carried_travel` (not rigid) fails a joint whose origin over its travel leaves [0, L] along the member frame x
  (prismatic between limits, an open limit unbounded; fixed, revolute, spherical at the placement; planar, free never),
  beyond `TRAVEL_TOLERANCE` (1e-9 m, a placement rotation's rounding).
- Bending plane (BLD-038, ruling 65): `_check_bending_plane` (not rigid) adds a note per flexible body horizontal at
  the reference configuration (frame x normal to gravity) with gravity along its frame z (`PLANE_TOLERANCE` 1e-9 on unit
  vectors): the chain bends about frame z only, so state the frame with y vertical.
- Materials: callers pass `design.schema.design_materials(design)` (the design's overrides applied, MAT-020).
- Fastenings (BLD-031): `PhysicsModel.fastenings`, one per fixed joint (`mount`, the joint's name) and per body
  fastener entry (`fastener`, `<body>_<kind>[_<index>]`); `PhysicsModel.fastening_index(name)`. Not an element kind.
- Ropes (BLD-032): a design rope with both ends on bodies becomes a neutral `Rope`; a `to` end at a case payload is
  a note, not an element, and is listed in `payload_ropes`; the native run hangs the case payload on it
  (`with_payload(..., rope=, starts=)`, M3-X1a), the bridge keeps the payload fixed to its carrier with a line.
- Reference tests: `tests/reference/lumped_beam_support.py` steps the built lumped member on `_engine` (tests only).

## Later

Loops as constraints (BLD-004), clamped-clamped calibration layout; full coupling of a carried joint (riding the deflected member, milestone 4).

## Section index

| Section | File | Line |
|---|---|---|
| Checks | physics.py | 132 |
| Bodies and joints | physics.py | 295 |
| Overlap | physics.py | 421 |
| Lumped members | physics.py | 477 |
| Transmissions and signals | physics.py | 673 |
| Physics model | physics.py | 809 |
