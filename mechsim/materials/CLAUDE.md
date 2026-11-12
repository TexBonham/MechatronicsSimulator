# materials/: contract

Material records (MAT). Schemas `schemas/material.json` from `Material` and `schemas/material_overrides.json` from
`MaterialOverridesFile`; regenerate with `uv run python -m mechsim.materials.schema --write-schema` and
`uv run python -m mechsim.materials.overrides --write-schema`, never hand-edit. Lookup rules: `docs/spec/mat.md`.

## Files

| File | Holds |
|---|---|
| `schema.py` | pydantic models, `SPREAD_PROPERTIES`, `sn_monotone`, `load_material_file`, `--write-schema` |
| `lookup.py` | `lookup`, `strength`, `direction_factor`, `allowable_amplitude`, `property_spread`, `ProcessLike` |
| `overrides.py` | overrides file models, `load_overrides`, `MaterialOverrides`, `DESIGN_FIELD`, `METADATA_KEY`, `--write-schema` |
| `database.py` | `load_shipped`, `MaterialSet`, `IntegrityMaterialView`, `SpreadValues` |
| `data/*.yaml` | One material per file |

## Model

| Field | Form |
|---|---|
| density, youngs_modulus, poissons_ratio, yield/ultimate/shear_strength | Spread |
| thermal_conductivity, specific_heat, thermal_expansion, max_service_temperature | Spread |
| `sn_curve` | >= 2 points `{cycles, stress_amplitude}`; cycles strictly increasing, stress non-increasing |
| `process` | optional: `printed` (build_direction, layer_height, walls >= 1, infill 0-100 percent, strength_factors in_layer/interlayer in (0, 1]), `machined`, `molded` |
| `placeholder`, `wide_spread_default` | required bools; printed shipped materials set both true |

Spread: `min <= typ <= max` unit strings stored SI; optional `temperature_dependence` of >= 2 `{temperature, factor}`
points, temperatures strictly increasing, factor > 0. Poisson's ratio is a bare-number string in (-1, 0.5).

## API (SI in and out, temperatures in kelvin; every failure a `MaterialError`)

| Call | Does |
|---|---|
| `lookup(material, property, temperature, spread="typ")` | spread value times the linearly interpolated temperature factor; outside the table an error naming property and range; unknown property an error naming material and property |
| `strength(material, kind, direction_body, process, temperature, spread="typ")` | kind yield, ultimate, shear; printed: times f = f_inter cos^2 theta + f_in sin^2 theta (theta to the build direction; placeholder, owner owes the axes model); `process=None` uses the material's own; machined, molded, extrusion isotropic |
| `allowable_amplitude(material, cycles)` | log-log linear S-N; outside the first to last point an error |
| `ProcessLike` | protocol: `.kind`, and for printed `.build_direction`; design's `PrintedProcess` and `MachinedProcess` satisfy it |
| `load_overrides(path) -> MaterialOverrides` | `.path`, `.text` (the recorded input), `.file`, `.sha256` |
| `MaterialSet(shipped, overrides=None)`, `MaterialSet.from_files(overrides_path=None)` | overrides applied before shipped values; unknown material in the file an error naming file and name |
| `MaterialSet.material(name)`, `.records()` | merged records; `records()` is a drop-in for `design.schema.shipped_materials()` (builder density) |
| `MaterialSet.lookup/strength/allowable_amplitude(name, ...)`, `.source(name, field)` | by name; `source` is the override's source or the record's |
| `MaterialSet.integrity_view(name) -> IntegrityMaterialView` | integrity's API (MAT-021) |
| `IntegrityMaterialView.strength_spread(kind, temperature, direction_body=None, process=None)` | `SpreadValues(min, typ, max)`; direction-dependent when a direction is given |
| `.yield_strength(T)`, `.ultimate_strength(T)`, `.shear_strength(T)` | isotropic record spreads |
| `.sn_curve()`, `.allowable_amplitude(cycles)`, `.source(field)`, `.material` | S-N points (cycles, Pa), allowable, provenance |

## Overrides file (`material_overrides.yaml`, worked example `schemas/examples/material_overrides.example.yaml`)

`material_overrides: {<material>: {<spread property>: {source, value | min, typ, max, temperature_dependence?},
sn_curve: {source, points}, strength_factors: {source, in_layer, interlayer}}}`; optional `description`. A design
references it with `materials.overrides: <path>` (`DESIGN_FIELD`, added by brief D); a run records its text under the
metadata key `material_overrides` (`METADATA_KEY`, brief R).

## Open

Strength axes model for printed parts is owed by the owner (cos^2 blend placeholder). S-N curves carry no spread.
