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
| `open_parameters` | name, dimension (pint or `choice`), owed_by, note, optional `default` (quantity string, choice value, or a mapping for a structured choice; DES-044); release: none |
| `bodies` | material, process, geometry (primitives or step), mass_properties, catalog_part, flexible (section: `shape` rectangle/circle/tube/rectangular_tube or `area`+`second_moment`, plus `length`, omitted when the catalog part carries `SECTION_PARAMETERS` (`section_part_values`); optional calibration (load at a free tip, else midspan); supports root/end; segments default 8, ruling 17), fasteners; `Section.area_and_moment()` gives SI A and I; `Section.properties()` / `shape_properties(shape)` give `SectionProperties` (A, I_y, I_z, c_y, c_z, torsion constant, shear areas; None for the area-and-moment form; ruling 25) |
| `joints` | six types, DOF lists per `DOF_COUNTS`; each body child of exactly one joint, no cycles |
| `components` | type, part, overrides, mechanical (shaft mounts to joints), signals `instance.pin`; `senses` (DES-043): required on sensor types, forbidden elsewhere; `joint` (OpenName), limit_switch `position` + `actuate_above` (required), incremental_encoder `reversed` (`SENSES_FIELDS`, `SENSES_REQUIRED`); load_cell may name `payload` (case payload name, weld force) or `joint_reaction` (a design joint, else a case payload read at its fixed joint `<payload>_fixed`, ruling 24) and imu `body` instead of `joint`, exactly one target (`SENSES_TARGETS`); `acts_on: {joint}` required on brake types, forbidden elsewhere (`ACTS_ON_TYPES`) |
| `wiring` | `Wire{source, destination}` as `instance.port`; ports are the type's electrical pins; connectivity only in v1 (DES-041) |
| `pin_bindings` | every signal bound once, MCU pins unique, mode suits direction; pin and mode in the chip's pin map (DES-029) |
| `power` | rails: supply (power instance), `gated_by` (safety instances), `consumers` (instances); controller rail |
| `environment` | gravity, ambient; optional `contact`: `default`, `pairs`, `exclusions` as the neutral `ContactModel`, plus `floor` (normal, offset, world only) (DES-042) |
| `ropes` | name, rope catalog `part` (`ROPE_PARAMETERS`), `from` (body or world, point), `to` (body and point, or a case `payload`), segments 8 (ADR 0016), optional rest_length (DES-046, BLD-032) |
| `materials` | `overrides`: path relative to the design file, loaded with `MaterialSet.from_files` at validation; `design_materials(design)` (shipped records with the overrides applied; the folder is the private `_source_dir` set when loaded with a `base`) is what every build caller passes |
| others | firmware path, chip, tick, `state_codes`/`fault_codes`/`event_codes` (DES-045; event entries `{name, argument}`, argument `axis`/`fault`/`state`/`number`/`none`, ruling 28; `state_name`, `fault_name`, `event_name`, `event_argument`); operator messages (no `heartbeat`); structural |

## Chip (`Chip`, `chip.py`)

name (folder name), family, `status` placeholder or verified, `platform_description` and `svd` (path in the chip
folder or `placeholder`; verified needs both files), `pins` (MCU pin name, `modes` from `PeripheralMode`, no duplicates).
Electrical facts, all required (CTL-013, SIG-014 consume them): `core_voltage`, `brownout_threshold` (both voltage
strings > 0, threshold strictly below core), `adc` (`resolution_bits` 1..32, `reference_voltage` > 0), `counter_bits`
1..64 (timer encoder counter width); SI floats inside.
`PeripheralMode` is the runtime vocabulary (`core/signals/CLAUDE.md`): digital_input, digital_output, counter_input,
adc_input, pulse_output, uart_rx, uart_tx; READ_MODES and WRITE_MODES in `schema.py` split them by direction.

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

| Section | File | Line |
|---|---|---|
| Open values | schema.py | 110 |
| Bodies | schema.py | 261 |
| Joints | schema.py | 612 |
| Components | schema.py | 703 |
| Power, environment, firmware, messages, ropes | schema.py | 790 |
| Design | schema.py | 1044 |
| Cross-reference checks | schema.py | 1091 |
| Files and schema | schema.py | 1608 |

Chip file tests: `tests/unit/test_chip_schema.py`.

## Open

Importers (M4); physical wire properties (a later component type).
