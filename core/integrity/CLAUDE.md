# core/integrity/: contract

Tier 1 structural integrity party (INT-023 to INT-028, ADR 0010, round 5 rulings 5, 24 to 28, 34, 35). Namespace
`mechsim::integrity`, target `mechsim_integrity` (-> `mechsim_kernel`), tests `mechsim_integrity_tests` (ctest names
`integrity.*`; links `mechsim_physics_native` for the native-run cases). Nothing design-specific: every limit arrives
in the limits table that `mechsim/integrity/tier1.py` writes (invariant 2). Rules of record: `docs/spec/int.md`,
check rules rows "Tier 1 party" to "Tier 2 rope snap".

| Header | Signatures |
|---|---|
| `limits.hpp` | `LimitsError`; `SupportCondition` (clamped, pinned; roller arrives as pinned); `Support{at, condition}`; `NodeChain{first_body, count, root, parent_body (optional), position, orientation}`; `MemberLimits{name, index (PhysicsOutputs::members), area, i_y, i_z, c_y, c_z, length, elastic_modulus, effective_length_factor, allowable_stress, yield_strength, deflection_limit (optional), supports, first_spring, segments, segment_length, node_stiffness, nodes (optional: none = deflection not evaluated)}`; `FasteningLimits{name, joint (PhysicsOutputs::joints), load_limit}`; `Limits{members, fastenings, minimum_safety_factor, bodies (optional: exact published body count, the party faults on another), stress, buckling, deflection, fastener, yield_offset}`; `limits_format` = `mechsim-integrity-limits-1`; `parse_limits(json)` (strict reader: duplicate keys, trailing text, non-finite or non-positive sizes refused) |
| `beam.hpp` | `end_stress(wrench, m)` = abs(N) / A + abs(M_y) c_z / I_y + abs(M_z) c_y / I_z; `peak_normal_stress(loads, m)` (ends and each plane's resolved interior point, I1's `_plane`); `compression(loads)` = max(F_start.x, -F_end.x, 0); `critical_load(m)` = pi^2 E I_min / (K L)^2; `node_deflection(bodies, m)` (ruling 28: largest distance of the node origins and the far end, last node + l / 2 along x, from the clamped root's tangent or the pinned chord; `LimitsError` without nodes or out of range); `start_node_moment(start, l)` = -M_z + (l / 2) F_y, `end_node_moment(end, l)` = M_z + (l / 2) F_y |
| `integrity_party.hpp` | `Crossing{time, element, quantity, value, limit}`; `IntegrityTrace{peak_stress, peak_deflection (NaN for a member without nodes), min_buckling_margin, peak_fastener_load, crossings, requests}` (shared, readable after the kernel owns the party); `IntegrityPartyConfig{name "integrity", rate, physics_outputs "physics.outputs", halt_on_crossing}`; `IntegrityParty(Limits, config, shared_ptr<IntegrityTrace>)`: role integrity, consumes `physics_outputs`, publishes `<name>.model_requests` (`kernel::ModelChangeRequests`) only with yield on, `finished()` after a crossing when halting; `crossing_detail(c)` (the JSON of `testing.driver.IntegrityCrossing`, keys sorted, tier 1) |

## Conventions

- A step evaluates the outputs physics wrote in the same tick (registered after physics); events are stamped
  `t + dt`, the row's time (the state the outputs hold), and go to the row through `record`.
- One event per (element, quantity) per run at the first tick strictly past the limit, no re-arm (ruling 34):
  quantities `stress`, `buckling_margin` (below `minimum_safety_factor`), `deflection`, `fastener_load`; order per tick
  members in table order (stress, buckling, deflection), then fastenings.
- Live loads, no design-load factor; allowable stress and fastener limits carry the minimum safety factor already.
- Yield (ruling 35): end with the larger abs(M_z) (start on a tie), spring `first_spring` or `first_spring + N - 1`;
  when abs(M_n) passes its running peak and the peak exceeds M_y = sigma_y I_z / c_y, one `SpringOffset` request
  sign(M_n) (peak - M_y) / k (a larger reversed peak takes its own sign; none while abs(M_n) stays at or below the
  peak); requests of a tick go out under a new sequence number (KER-028). The party never emits a
  model_change event: the physics party records the adapter's listing (ruling 26).
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
