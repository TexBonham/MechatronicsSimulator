# core/integrity/: contract

Tier 1 structural integrity party (INT-023 to INT-028, ADR 0010, round 5 rulings 5, 24 to 28, 34, 35). Namespace
`mechsim::integrity`, target `mechsim_integrity` (-> `mechsim_kernel`), tests `mechsim_integrity_tests` (ctest names
`integrity.*`; links `mechsim_physics_native` for the native-run cases). Nothing design-specific: every limit arrives
in the limits table that `mechsim/integrity/tier1.py` writes (invariant 2). Rules of record: `docs/spec/int.md`,
check rules rows "Tier 1 party" to "Tier 2 rope snap".

| Header | Signatures |
|---|---|
| `limits.hpp` | `LimitsError`; `SupportCondition` (clamped, pinned; roller arrives as pinned); `Support{at, condition}`; `CarriedJoint{body, joint}`; `NodeChain{first_body, count, root, parent_body (optional), position, orientation, carries, carried, hogging_bound, hogging}` (`carries` and `carried` required in the table; carried joints with carries false refused; optional `hogging` {`bound` >= 0 N m, `intervals` [[lo, hi]] m}, ruling 79, additive); `MemberLimits{name, index (PhysicsOutputs::members), area, i_y, i_z, c_y, c_z, length, elastic_modulus, effective_length_factor, allowable_stress, yield_strength, deflection_limit (optional), supports, first_spring, segments, segment_length, node_stiffness, nodes (optional: none = deflection not evaluated)}`; `FasteningLimits{name, joint (PhysicsOutputs::joints), load_limit, member (optional: a mount's held member, required key, null otherwise)}`; `Limits{members, fastenings, minimum_safety_factor, bodies (optional: exact published body count, the party faults on another), stress, buckling, deflection, fastener, yield_offset}`; `limits_format` = `mechsim-integrity-limits-1`; `parse_limits(json)` (strict reader: duplicate keys, trailing text, non-finite or non-positive sizes refused) |
| `beam.hpp` | `end_stress(wrench, m)` = abs(N) / A + abs(M_y) c_z / I_y + abs(M_z) c_y / I_z; `node_moments(bodies, m, offsets = {})` (k times each node body's rotation about z relative to the one before, minus the spring's yield offset, root to tip; root frame before a clamped node 0, a pinned root's springs from node 1), `CarriedAt{position, force, couple}`, `carried_at(bodies, joints, m)` (one tick's (s, F . y, C) per carried joint), `CarriedMoment{value, position}`, `CarriedReads{reads, bound}`, `carried_reads(loads, moments, m, carried)` (ruling 75, the algorithm of beam.py `row_reads`), `peak_normal_stress(loads, bodies, m, offsets = {}, carried = {})` (ends, every node spring and, when the chain carries a joint, every carried read plus the bound times c_y / I_z; M_y interpolated between the ends; ends only without nodes); `compression(loads)` = max(F_start.x, -F_end.x, 0); `critical_load(m)` = pi^2 E I_min / (K L)^2; `node_deflection(bodies, m)` (ruling 28: largest distance of the node origins and the far end, last node + l / 2 along x, from the clamped root's tangent or the pinned chord; `LimitsError` without nodes or out of range); `start_node_moment(start, l)` = -M_z + (l / 2) F_y, `end_node_moment(end, l)` = M_z + (l / 2) F_y; `couple_floor` 1e-9; `hogs(m, carried)` (ruling 79: a carried couple applied inside one of the chain's `hogging` stretches with a positive `hogging_bound`) |
| `integrity_party.hpp` | `Crossing{time, element, quantity, value, limit}`; `IntegrityTrace{peak_stress (NaN with stress off), peak_deflection (NaN for a member without nodes), min_buckling_margin, peak_fastener_load, crossings, before_evaluation, not_valid, requests, hogging_bound, hogging_read}` (the last two per member, ruling 79) (shared, readable after the kernel owns the party); `IntegrityPartyConfig{name "integrity", rate, physics_outputs "physics.outputs", halt_on_crossing, evaluate_from (SimTime, TST-044)}`; `IntegrityParty(Limits, config, shared_ptr<IntegrityTrace>)`: role integrity, consumes `physics_outputs`, publishes `<name>.model_requests` (`kernel::ModelChangeRequests`) only with yield on, `finished()` after a crossing when halting; `watch_validity(shared_ptr<const ValiditySource>)` (`validity.hpp`: `ValiditySource::not_valid(index)`, `ValidityError` for an index it does not hold, which ends the run; the physics binding fills it, `CarriedRecord.validity_source()`, so nothing here includes an adapter header, invariant 5, ruling 79; the engine's live flag per member index: a crossing of a flagged member or its mount is a `not_valid_kind` event, never counted or halting, `IntegrityTrace::not_valid`; ruling 75), `crossing_detail(c)` (the JSON of `testing.driver.IntegrityCrossing`, keys sorted, tier 1) |

## Conventions

- A step evaluates the outputs physics wrote in the same tick (registered after physics); events are stamped
  `t + dt`, the row's time (the state the outputs hold), and go to the row through `record`.
- One event per (element, quantity) per run at the first tick strictly past the limit, no re-arm (ruling 34):
  quantities `stress`, `buckling_margin` (below `minimum_safety_factor`), `deflection`, `fastener_load`; order per tick
  members in table order (stress, buckling, deflection), then fastenings.
- Evaluation start (TST-044): rows stamped before `evaluate_from` update no peak and issue no yield request (ruling 72); their crossings raise one `integrity_before_evaluation` event per (element, quantity), never counted, never halting.
- Live loads, no design-load factor; allowable stress and fastener limits carry the minimum safety factor already.
- Yield (ruling 35): end with the larger abs(M_z) (start on a tie), spring `first_spring` or `first_spring + N - 1`;
  when abs(M_n) passes its running peak and the peak exceeds M_y = sigma_y I_z / c_y, one `SpringOffset` request
  sign(M_n) (peak - M_y) / k (a larger reversed peak takes its own sign; none while abs(M_n) stays at or below the
  peak); requests of a tick go out under a new sequence number (KER-028). The party never emits a
  model_change event: the physics party records the adapter's listing (ruling 26).
- Yielded springs (INT-028): the party applies its own requests to `MemberState::offsets` at the start of its next step (physics applied them before that step), and reads stress from the elastic rotation.
- Carried loads per member are read from the previous tick's outputs (the engine places the pair at the step's start); the first tick uses its own.
- No unordered containers, threads or wall clock.

## Section index

| Section | File | Line |
|---|---|---|
| Fixtures | tests/test_integrity_party.cpp | 35 |
| Crossing oracles (INT-023, INT-024; node-pose deflection, INT-006) | tests/test_integrity_party.cpp | 159 |
| Events, halt and determinism | tests/test_integrity_party.cpp | 329 |
| Yield requests (INT-028; at-limit boundary, INT-024) | tests/test_integrity_party.cpp | 399 |
| Limits table | tests/test_integrity_party.cpp | 528 |
| Cantilever on the native adapter (yield oracle, additivity) | tests/test_integrity_native.cpp | 33 |
