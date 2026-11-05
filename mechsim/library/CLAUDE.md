# library/: contract

Component types and shipped catalog (LIB). Schema `schemas/component.json` from `CatalogFile`; regenerate with
`uv run python -m mechsim.library.schema --write-schema`, never hand-edit.

## Files

| File | Holds |
|---|---|
| `schema.py` | pydantic models, `load_catalog_file`, `--write-schema` |
| `catalog/*.yaml` | One file per type: `types:` plus `parts:`; hook validates against `component.json` |

## Model

| Class | Key points |
|---|---|
| `CatalogFile` | `types` (min 1), `parts` (may be empty); parts resolve against types in the same file |
| `ComponentType` | name, semver version, category, parameters, tiers, mechanical, electrical, ratings, thermal, failure_modes, plugin |
| `QuantityParameter` | `dimension` (pint string), optional default/minimum/maximum as unit strings, stored SI; `sweepable` default true |
| `FlagParameter` | `kind: flag`, bool default |
| `Tier` | ordered lowest first; each parameter in exactly one tier; `tier_parameters(t)` is the additive union |
| Facets | name parameters by reference (mass, pin voltage/current ratings, absolute maximums, holding torque) |
| `Pin` | name, direction (input, output, bidirectional, power_in, power_out, ground), signal_kind; the pins are the ports design wiring connects (DES-041) |
| `PartEntry` | a part as written: `values` unit strings or bools (file schema) |
| `CatalogPart` | resolved part, SI floats; only via `CatalogPart.resolve(entry, type, overrides=None)` |
| `CatalogFile.resolved_parts()` | every part resolved at file validation; `component_type(name)` lookup |

## Section index, `schema.py`

| Section | File | Line |
|---|---|---|
| Parameters | `mechsim/library/schema.py` | 88 |
| Facets | `mechsim/library/schema.py` | 166 |
| ComponentType | `mechsim/library/schema.py` | 239 |
| CatalogPart | `mechsim/library/schema.py` | 334 |
| CatalogFile | `mechsim/library/schema.py` | 504 |

## Rules

- No default means required in every part whose tier includes the parameter.
- Category `transmission` needs ratio, efficiency, stiffness, backlash, self_locking, slip (slip default 0).
- Category `sensor` needs resolution, noise, delay, sample_rate and at least one failure mode.
- `thermal.held_constant` must be true in v1; plugin needs `python` (`pkg.mod:Class`), `cpp`, or both.
- `load_catalog_file` errors start with the file path.
- Safety types `estop_relay` and `watchdog` both carry a `heartbeat` input port; `watchdog` has `timeout` and gates its
  rail through `contact_in`/`contact_out` (LIB-019 watchdog half; load cell, amplifier, IMU still owed).
