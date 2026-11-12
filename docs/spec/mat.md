# Spec: Materials database (MAT)

## Purpose

The materials database (`mechsim/materials/`) holds per-material properties with min, typical and max spreads and
temperature dependence, process attributes with direction-dependent strengths, and S-N fatigue curves, shipped as YAML
with conservative defaults the team can override. It answers property lookups for the builder (mass and inertia) and
for integrity (limits). It does not compute stress or safety factors; that is INT.

## Interfaces with

| Module | This spec's requirements | Their requirements |
|---|---|---|
| BLD | MAT-020, MAT-022, MAT-023 | BLD-010, BLD-020 |
| INT | MAT-010, MAT-011, MAT-012, MAT-021 | INT-005, INT-008, INT-009, INT-020 |
| DES | MAT-009, MAT-017 | DES-001 |

## Requirements

Requirement: one shall-statement. Source: a document and section, an ADR, or a ruling ID. Milestone: M1 to M5,
spike, phase 1, or roadmap; "spike" is the calibration spike before phase 1 (R-12): engine packages 1 and 2 plus their
analytical references. Tier: unit, reference, acceptance or invariant. Tolerance: a value with its unit, "<value>
(<qualifier>)", "identical", "bitwise identical", "n/a", "not set (owed by <who>)", or "not set (owed by <who>,
proposed <value>)".

| ID | Requirement | Source | Milestone | Tier | Tolerance |
|---|---|---|---|---|---|
| MAT-001 | Every material record shall hold density, Young's modulus, Poisson's ratio, yield strength, ultimate strength and shear strength. | Architecture doc, Modules: inputs, Materials database row | M1 | unit | n/a |
| MAT-002 | Every material record shall hold thermal conductivity, specific heat, thermal expansion coefficient and maximum service temperature. | Architecture doc, Modules: inputs, Materials database row | M3 | unit | n/a |
| MAT-003 | Every material record shall hold an S-N fatigue curve. | Architecture doc, Modules: inputs, Materials database row | M3 | unit | n/a |
| MAT-004 | Every material property shall carry a min, typical and max value. | Architecture doc, Modules: inputs, Materials database row | phase 1 | unit | n/a |
| MAT-005 | A property whose min, typical and max are not in non-decreasing order shall be rejected at load with an error naming the file, the material and the property. | Architecture doc, Modules: inputs, Materials database row; Requirements doc, Component library requirements (validation on load) | phase 1 | unit | n/a |
| MAT-006 | Every material property shall accept a temperature dependence, and a lookup shall evaluate the property at the requested temperature. | Architecture doc, Modules: inputs, Materials database row | M3 | unit | n/a |
| MAT-007 | A lookup at a temperature outside a property's stated temperature range shall end with an error rather than an extrapolated value. | Architecture doc, Repository starting point, invariant 7 | M3 | unit | n/a |
| MAT-008 | A lookup shall return the min, typical or max value of a property as the caller requests. | Architecture doc, Modules: inputs, Materials database row | M3 | unit | n/a |
| MAT-009 | The database shall accept the process attributes printed (with print orientation, wall count and infill), machined and molded. | Architecture doc, Modules: inputs, Materials database row; Decisions log row 20 | M3 | unit | n/a |
| MAT-010 | A material with a process shall hold strengths that depend on loading direction relative to the process orientation. | Decisions log row 20; Architecture doc, Modules: inputs, Materials database row | M3 | unit | n/a |
| MAT-011 | A strength lookup shall take a loading direction in the body frame and the body's process attributes and return the strength that applies to that direction. | Decisions log row 20; Architecture doc, Repository starting point layout, `materials/` (direction-dependent limits) | M3 | unit | n/a |
| MAT-012 | An S-N curve lookup shall return the allowable stress amplitude for a cycle count inside the curve's data range and end with an error for a cycle count outside it. | Architecture doc, Modules: inputs, Materials database row; Repository starting point, invariant 7 | M3 | unit | n/a |
| MAT-013 | The shipped database shall contain aluminum extrusion grades, steel fastener grades and common bearing steels. | Architecture doc, Modules: inputs, Materials database row | M1 | unit | n/a |
| MAT-014 | The shipped database shall contain PLA, PETG and ABS as printed materials with orientation and infill. | Architecture doc, Modules: inputs, Materials database row | M3 | unit | n/a |
| MAT-015 | Shipped process-aware properties for printed materials shall be conservative defaults with wide min to max spreads. | Architecture doc, Risks and open questions (process-aware material data row) | M3 | unit | n/a |
| MAT-016 | The team shall be able to override any property value and spread of a shipped material, for example from coupon tests, without editing the shipped database file. | Architecture doc, Risks and open questions (process-aware material data row) | M3 | unit | n/a |
| MAT-017 | An override shall be held in a data file that the run's design or project references, so that a run's material values are reproducible from its recorded inputs. | Architecture doc, Repository starting point, invariants 3 and 4 | M3 | unit | n/a |
| MAT-018 | Every shipped materials file shall be YAML and shall validate against its JSON Schema, which is generated from the pydantic v2 material model. | Decisions log row 29; ruling L4; Architecture doc, Repository starting point, invariant 3 | phase 1 | invariant | n/a |
| MAT-019 | Every quantity in a materials file shall be a unit string parsed by pint at load and held as a plain SI float afterward. | Ruling L5; ADR 0012 | phase 1 | unit | n/a |
| MAT-020 | The database shall supply density to the builder for mass and inertia computation. | Architecture doc, Modules: inputs, Materials database row (consumed by model builder); Modules: core, Model builder row | M1 | unit | n/a |
| MAT-021 | The database shall supply yield, ultimate and shear strength, the S-N curve and the direction-dependent strengths to integrity checks, with their spreads. | Architecture doc, Modules: inputs, Materials database row (consumed by structural analysis) | M3 | unit | n/a |
| MAT-022 | A lookup of a property absent from a material shall end with an error naming the material and the property, never a default value. | Architecture doc, Repository starting point, invariant 7; Modules: core, Model builder row (fails loudly on a missing material) | M1 | unit | n/a |
| MAT-023 | A reference to a material name the database does not hold shall be rejected at load with an error naming the reference and the file it appears in. | Architecture doc, Modules: core, Model builder row (fails loudly on a missing material) | M1 | unit | n/a |
| MAT-024 | A process attribute value outside its valid range, such as an infill above 100 percent or a wall count below one, shall be rejected at load. | Requirements doc, Component library requirements (validation on load, impossible values) | M3 | unit | n/a |

## Open parameters

| Parameter | Current value | Owed by |
|---|---|---|
| Shipped grades within each v1 material family (which aluminum alloys, fastener classes, bearing steels) | not set | owner |
| Default spreads for printed materials (MAT-015) | not set | owner; team coupon tests to replace |
| S-N curve interpolation scheme between data points | log-log linear between adjacent points (round 5, brief M) | settled |
| Temperature at which v1 lookups are evaluated while the thermal slot is constant (design ambient proposed) | not set | owner |
| Strength axes model for printed parts (in-layer versus interlayer, or full orthotropic) | placeholder (round 5 ruling 7): f = f_inter cos^2 theta + f_in sin^2 theta, theta the angle between loading and build direction | owner |

## Lookup rules

Implementation rules for the M3 rows; `mechsim/materials/CLAUDE.md` holds the API.

| Rule | Rows |
|---|---|
| A temperature table multiplies min, typ and max alike; between points it is linear in temperature, a property without a table is temperature independent, a non-finite temperature is an error | MAT-006, MAT-007 |
| A printed body's strength is the record's strength times the direction factor above, using the body's build direction (the material's own when the body gives no process); machined, molded and extrusion are isotropic; a printed process on a material without strength factors, or a non-printed process on a printed material, is an error | MAT-010, MAT-011 |
| An S-N lookup at a cycle count outside the first to last point, or non-finite, is an error | MAT-012 |
| An overrides file (`material_overrides:` mapping material name to overridden fields, each with a non-empty `source`) overrides a spread property by `value` (min = typ = max) or the full `min`, `typ`, `max`, the S-N curve by `points`, printed strength factors by `in_layer`, `interlayer`; a property override without `temperature_dependence` keeps the shipped table; an unknown material name is rejected with the file and the name | MAT-016, MAT-023 |
| A design names its overrides file in `materials.overrides`; a run records the file's text under the metadata key `material_overrides` | MAT-017 |

## Out of scope

| Item | Seam it belongs to |
|---|---|
| Stress, buckling, fastener load and safety-factor computation | INT |
| Choice of min, typical or max for a given check | INT |
| Mass and inertia computation from geometry | BLD |
| Body material and process assignment | DES |
| Thermal simulation | Roadmap |
