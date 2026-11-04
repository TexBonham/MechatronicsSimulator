# design/: contract

Design files (DES). Schema `schemas/design.json` from `Design`; regenerate with
`uv run python -m mechsim.design.schema --write-schema`, never hand-edit. Chip definitions: `schemas/chip.json` from
`Chip`; regenerate with `uv run python -m mechsim.design.chip --write-schema`. Examples: `designs/pendulum/design.yaml`
(release), `designs/gantry/design.yaml` (draft).

| File | Holds |
|---|---|
| `schema.py` | models, `cross_check`, `validate_design_file`, `load_design`, `design_identity`, `blob_hash`, CLI |
| `chip.py` | `Chip` (`chips/<name>/chip.yaml`), `PeripheralMode`, `load_chip`, `validate_chip_file`, `load_chips`, `shipped_chips`, CLI |
| `cad.py` | abstract `StepImporter`, `UrdfImporter`, result models, `cross_check_mass` |

## Model (`Design`, `extra="forbid"`, strict)

| Section | Key points |
|---|---|
| header | name, semver `version`, `source_hash` (40 hex or `unset`), `status` draft or release |
| `open_parameters` | name, dimension (pint or `choice`), owed_by, note; release: none |
| `bodies` | material, process, geometry (primitives or step), mass_properties, catalog_part, flexible (A, I, L, supports root/end, calibration), fasteners |
| `joints` | six types, DOF lists per `DOF_COUNTS`; each body child of exactly one joint, no cycles |
| `components` | type, part, overrides, mechanical (shaft mounts to joints), signals `instance.pin` |
| `wiring` | `Wire{source, destination}` as `instance.port`; ports are the type's electrical pins; connectivity only in v1 (DES-041) |
| `pin_bindings` | every signal bound once, MCU pins unique, mode suits direction; pin and mode in the chip's pin map (DES-029) |
| `power` | rails: supply (power instance), `gated_by` (safety instances), `consumers` (instances); controller rail |
| `environment` | gravity, ambient; optional `contact`: `default`, `pairs`, `exclusions` as the neutral `ContactModel`, plus `floor` (normal, offset, world only) (DES-042) |
| others | firmware path, chip, tick; operator messages (no `heartbeat`); structural |

## Chip (`Chip`, `chip.py`)

name (folder name), family, `status` placeholder or verified, `platform_description` and `svd` (path in the chip
folder or `placeholder`; verified needs both files), `pins` (MCU pin name, `modes` from `PeripheralMode`, no duplicates).

## Rules

- `${name}` (`OpenValue`) stands in for a quantity (dimension-checked) or a choice field.
- One parse and validation: field, reference and missing-CAD-file errors return together, prefixed by the file.
- Context keys: `registry` (else `default_registry()`), `base` (folder STEP files resolve against), `chips` (name to
  `Chip`, else `shipped_chips()`); an open `firmware.chip` skips the pin-map check.
- Wiring directions: output to input or bidirectional; bidirectional to input or bidirectional; power_out to power_in;
  ground to ground. A signal port carries one connection (its controller-facing signal counts); power and ground
  ports may be shared.
- Mass cross-check: an error equal to the tolerance passes.

## Section index

`schema.py`: Open values 110, Bodies 237, Joints 476, Components 567, Power, environment, firmware, messages 616,
Design 762, Cross-reference checks 807, Files and schema 1220.

`tests/unit/test_design_schema.py`: Shipped examples 99, Header and version 187, Open parameters 298, Bodies 394,
Joints 768, Components 944, Signals and pin bindings 1051, Power 1164, Environment, firmware, structural 1201,
Wiring and rail consumers 1302, Contact environment 1502, Chip pin map 1747, Operator messages 1807, Errors 1887,
CAD mass cross-check 1944. Chip file tests: `tests/unit/test_chip_schema.py`.

## Open

Importers (M4); physical wire properties (a later component type).
