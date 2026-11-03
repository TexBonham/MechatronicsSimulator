# design/: contract

Design files (DES). Schema `schemas/design.json` from `Design`; regenerate with
`uv run python -m mechsim.design.schema --write-schema`, never hand-edit. Examples: `designs/pendulum/design.yaml`
(release), `designs/gantry/design.yaml` (draft).

| File | Holds |
|---|---|
| `schema.py` | models, `cross_check`, `validate_design_file`, `load_design`, `design_identity`, `blob_hash`, CLI |
| `cad.py` | abstract `StepImporter`, `UrdfImporter`, result models, `cross_check_mass` |

## Model (`Design`, `extra="forbid"`, strict)

| Section | Key points |
|---|---|
| header | name, semver `version`, `source_hash` (40 hex or `unset`), `status` draft or release |
| `open_parameters` | name, dimension (pint or `choice`), owed_by, note; release: none |
| `bodies` | material, process, geometry (primitives or step), mass_properties, catalog_part, flexible (A, I, L, supports root/end, calibration), fasteners |
| `joints` | six types, DOF lists per `DOF_COUNTS`; each body child of exactly one joint, no cycles |
| `components` | type, part, overrides, mechanical (shaft mounts to joints), signals `instance.pin` |
| `pin_bindings` | every signal bound once, MCU pins unique, mode suits direction |
| others | power rails gated by safety instances; environment; firmware path, chip, tick; operator messages (no `heartbeat`); structural |

## Rules

- `${name}` (`OpenValue`) stands in for a quantity (dimension-checked) or a choice field.
- One parse and validation: field, reference and missing-CAD-file errors return together, prefixed by the file.
- Context keys: `registry` (else `default_registry()`), `base` (folder STEP files resolve against).
- Mass cross-check: an error equal to the tolerance passes.

## Section index

`schema.py`: Open values 100, Bodies 226, Joints 465, Components 556, Power/environment/firmware/messages 594,
Design 689, Cross-reference checks 730, Files and schema 1026.

`tests/unit/test_design_schema.py`: Shipped examples 92, Header 180, Open parameters 291, Bodies 387, Joints 761,
Components 937, Signals 1044, Power 1157, Environment/firmware/structural 1194, Operator messages 1295, Errors 1375,
CAD mass cross-check 1432.

## Open

Chip pin-map check (DES-029), importers (M4), instance wiring (DES-041), heartbeat pin (LIB-019).
