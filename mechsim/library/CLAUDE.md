# library/: contract

Component types and shipped catalog (LIB). Schema `schemas/component.json` from `CatalogFile`; regenerate with
`uv run python -m mechsim.library.schema --write-schema`, never hand-edit.

## Files

| File | Holds |
|---|---|
| `schema.py` | pydantic models, `load_catalog_file`, `--write-schema` |
| `catalog/*.yaml` | One file per type: `types:` plus `parts:`; hook validates against `component.json` |
| `registry.py` | `Registry` (types and parts merged from every catalog file), `load_registry(dir)`, `default_registry()`; `resolve_part(name, overrides)`; `parameter_override(type, parameter, value)` (LIB-016: dimension or flag checked, returns the value to store) and `substitute_part(type, part)` (LIB-017: part exists and is of that type), both raise `RegistryError` |

## Model

| Class | Key points |
|---|---|
| `CatalogFile` | `types` (min 1), `parts` (may be empty); parts resolve against types in the same file |
| `ComponentType` | name, semver version, category (adds `front_end`, LIB-026), parameters, tiers, mechanical, electrical, ratings, thermal, failure_modes, plugin |
| `QuantityParameter` | `dimension` (pint string), optional default/minimum/maximum as unit strings, stored SI; `minimum_exclusive` (default false) makes the minimum strict; `sweepable` default true |
| `FlagParameter` | `kind: flag`, bool default |
| `Tier` | ordered lowest first; each parameter in exactly one tier; `tier_parameters(t)` is the additive union |
| Facets | name parameters by reference (mass, pin voltage/current ratings, absolute maximums, holding torque) |
| `Pin` | name, direction (input, output, bidirectional, power_in, power_out, ground), signal_kind; the pins are the ports design wiring connects (DES-041) |
| `PartEntry` | a part as written: `values` unit strings or bools (file schema) |
| `CatalogPart` | resolved part, SI floats; only via `CatalogPart.resolve(entry, type, overrides=None)` |
| `CatalogFile.resolved_parts()` | every part resolved at file validation; `component_type(name)` lookup |

## Section index

| Section | File | Line |
|---|---|---|
| Parameters | schema.py | 111 |
| Facets | schema.py | 202 |
| ComponentType | schema.py | 275 |
| CatalogPart | schema.py | 383 |
| CatalogFile | schema.py | 553 |

## Rules

- No default means required in every part whose tier includes the parameter.
- Category `transmission` needs ratio, efficiency, stiffness, backlash, self_locking, slip (slip default 0); belt_pulley and lead_screw also take `damping_ratio` (tier compliant, default 0, LIB-035, ruling 78).
- Category `sensor` needs resolution, noise, delay, sample_rate and at least one failure mode.
- Categories `payload` (needs `mass`) and `end_effector` (needs `capacity` [mass], `events: [attach, release]`) both
  need an `attach` mount; only an end_effector lists `events` (LIB-031).
- `linear_encoder`: the incremental encoder with length `resolution`/`noise` (ruling 32); the wiring builds it from its
  `plugin.cpp` with `linear: true` from the length dimension.
- `thermal.held_constant` must be true in v1; plugin needs `python` (`pkg.mod:Class`), `cpp`, or both.
- `load_catalog_file` errors start with the file path.
- Safety types `estop_relay` and `watchdog` both carry a `heartbeat` input port; `watchdog` has `timeout` and gates its
  rail through `contact_in`/`contact_out` (LIB-019 watchdog half).
- Milestone 2 types (LIB-019, LIB-025, LIB-027): `load_cell` (sensor; `overload_limit` is the force absolute maximum;
  tiers ideal, nonlinear), `load_cell_amplifier` (front_end; `out` binds to an adc_input pin), `imu` (sensor; analog
  output per axis until a serial peripheral mode exists), `brake` (safety; `brake_torque` is the holding torque; the
  joint it acts on is design data, `acts_on`, owed). C++ plugins for them owed by brief A2.
- Milestone 3 structural types (LIB-020, LIB-030): `flexible_beam` (category body; `SECTION_PARAMETERS` area,
  second_moment, length, elastic_modulus, density; mass from geometry; a design body takes the section by naming the
  part), `hook` (end_effector; mounts attach and rope), `rope` (category rope; `ROPE_PARAMETERS` axial_stiffness,
  damping (strain-rate, N s), mass_per_length, breaking_tension, rest_length are required; breaking_tension in tier
  `breaking`). Plugins point at `mechsim.builder.structure`.
