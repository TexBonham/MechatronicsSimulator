# materials/: contract

Material records (MAT). Schema `schemas/material.json` from `Material`; regenerate with
`uv run python -m mechsim.materials.schema --write-schema`, never hand-edit.

## Files

| File | Holds |
|---|---|
| `schema.py` | pydantic models, `load_material_file`, `--write-schema` |
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

## Open

Lookups (temperature evaluation, S-N interpolation, direction strength, overrides) are M3 stubs; strength axes model
for printed parts is owed by the owner (in-layer / interlayer used now).
