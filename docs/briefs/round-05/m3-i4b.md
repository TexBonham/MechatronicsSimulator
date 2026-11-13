# M3-I4b: the moment under a load between two nodes, the yielded spring, the first-ramp bound

Fresh agent (the I4 transcript is at its cap). Agent: module-writer (Opus). Worktree
`../MechatronicsSimulator-05-x1a` on `wip/05-x1a` at 4bd6609 (built and synced). Brief E7d runs concurrently in
`core/engine/`: do not edit engine, adapter or `mechsim/physics/`. Call cap 80 with the handoff rule (at 70 calls
stop adding scope, commit what is green, report what is left). Read first: `docs/briefs/round-05/m3-i4.md`,
`mechsim/integrity/CLAUDE.md`, `core/integrity/CLAUDE.md`, the strength check and the party's beam code as I4
left them, rows INT-005, INT-023, INT-028, ACC-025, round 5 rulings 35, 57, 63, 70, 71 in
`/home/tex/dev/MechatronicsSimulator/docs/queue.md`. Builds in the foreground through `tools/locked_build.sh`
(600000 ms timeout; no detached jobs; ctest filter in plain double quotes). Iterate with filtered pytest and
`core debug -R "integrity|Integrity"`; end with ONE `tools/locked_build.sh core debug-sanitize -R
"integrity|Integrity"` waited for in the foreground (clang-tidy clean), then ruff check, ruff format --check,
mypy, `python3 tools/check_imports.py`; no full pytest run of your own. Commit as you go; final message "WIP
M3-I4b: moment under a carried load", no attribution trailer, no push. Do not edit `docs/queue.md` or
`docs/cost-log.md`.

## Ownership

`mechsim/integrity/**`, `core/integrity/**`, `tests/acceptance/test_acc_stubs.py` and `test_cases/gantry/G12.yaml`
(item 3 only), their tests, `docs/spec/{int,acc}.md` rows, `docs/spec/traceability.md`, the contracts.

## Items

1. The moment under a load between two nodes (ruling 71; INT-005, INT-023). The engine puts a carried load on
   the two adjacent nodes by the lever rule, so the chain's node moments are exact at the nodes but the peak
   under the load, between them, is missing: G15's rail_y reads 11 percent low at mid-span (worst case a load at
   mid-segment: low by F l / 4, a fraction l / L of P L / 4). No force or foot point is needed to recover it.
   Derive in the header: outside the loaded segment the lever-rule split is statically equivalent to the point
   load, so the node moments M_k are exact and the moment is linear on each side of the load with the shear of
   the neighbouring segment; the true moment under the load is where the line through (s_{k-1}, M_{k-1}),
   (s_k, M_k) extended to the right meets the line through (s_{k+1}, M_{k+1}), (s_{k+2}, M_{k+2}) extended to the
   left. Rule for both tiers: for every segment take the intersection of its two neighbours' moment lines; when
   it lies inside the segment its moment is a candidate for the member's peak (when the two lines coincide there
   is no load in the segment and nothing is added); at the first and last segment use the end moment and the
   support condition for the missing neighbour (say exactly how for pinned, clamped, roller and free ends). Say
   why it is conservative for a load distributed within a segment (the moment is concave there, so the tangents'
   intersection is at or above the true maximum for one sign of load) and what it does when two loads share a
   segment. Oracles: a single carried load at a = 0.1 L, 0.3125 L (mid-segment at eight segments), 0.5 L and
   0.77 L on a pinned-pinned member: P a b / L within 1e-9 relative of the chain statics (exact, not 2 percent);
   clamped-clamped with a load at mid-segment against the closed form within the row; a member with no carried
   load: unchanged to the last bit; own weight only: the peak is not raised above the continuum value by more
   than the row; the two tiers agree within 1e-9. Use the engine through a small data-only design with
   `carries` to produce the loads (read only; do not edit the engine).
2. A yielded spring (INT-028, ruling 35): the node moment must use the spring's elastic rotation (total relative
   rotation minus the plastic offset applied by tier 2), not the total; the party knows the offsets it requested
   and the post-run check reads them from the recorded `model_change` events. Test: after a yield offset the
   moment equals k times the elastic part and the stress does not exceed the yield stress by more than the
   placeholder model allows (state it).
3. ACC-025's first-ramp bound is still computed from the recorded hook motion. Make it a-priori: derive the
   allowance on the trolley's acceleration from the drive (the commanded ramp a, plus what the drive's
   spring-damper lag permits at the start of a ramp: the peak acceleration of a second-order follower of a ramp
   in rate, in terms of its damping ratio; state the gantry's values from the data), assert the recorded hook
   acceleration within that allowance, and bound the swing by 2 a_max / g with the pendulum length to the
   payload's centre; the acc row states the bound.
4. Re-run G15 on native and give the per-rail peak stress and safety factor before and after item 1, against
   M c / I by hand at the carriage position (settled part only; the stop is not valid, ruling 69).

## Report

Under 200 words: commits, filtered test counts and the sanitizer build result, item 1's oracles (measured against
closed form, both tiers) and the end-condition rules, item 2's test, item 3's allowance and measured acceleration,
item 4's numbers, open issues.
