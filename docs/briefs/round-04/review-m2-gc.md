# Review: M2-G (chip data, catalog types) and M2-C (bridge physical faults)

Reviewer: Fable (general-purpose). Read-only. Review from the spec rows and the diff only; never read the writer's
report or transcript. Call cap 40.

| Brief | Worktree | Diff | Spec rows | Brief of record |
|---|---|---|---|---|
| G | `/home/tex/dev/MechatronicsSimulator-04-g` | `git -C <worktree> diff round/04-milestone-2...wip/04-g` (commit ab1f65e) | DES-029 in `docs/spec/des.md`; LIB-019, 025, 027, 026, 021 in `docs/spec/lib.md` | `docs/briefs/round-04/m2-g.md` |
| C | `/home/tex/dev/MechatronicsSimulator-04-c` | `git -C <worktree> diff round/04-milestone-2...wip/04-c` (commit f29a552) | PHY-006, 016, 017, 019, 033, 034 in `docs/spec/phy.md`; ENG-035; TST-015, 020 | `docs/briefs/round-04/m2-c.md` |

Read first: root `CLAUDE.md` (invariants, C++ and Python rules), the brief of record for each, the spec rows, then
the diff. Contracts: `mechsim/library/CLAUDE.md`, `mechsim/design/CLAUDE.md`, `core/physics/mujoco/CLAUDE.md`,
`mechsim/physics/mujoco/CLAUDE.md`, `core/kernel/CLAUDE.md`. Do not build or run tests (the machine has one build
slot and writers hold it); read the test code instead and judge whether each asserted number is derived from
physics or the spec, not from the implementation.

Check, per brief:
1. Every spec row the brief names is implemented as written, cited in a test, and its spec edit (if the row text
   changed) is in the same diff (R-18).
2. Correctness of the physics and the derivations: C's slip semantics in the reflection mapping (rest increments
   scaled by 1 - s, shaft position reported as driven / c plus accumulated slip, the (1 - s) force scaling for a
   torque-driven shaft), the backlash play element with memory, the payload weld (force d m g after one step with
   MuJoCo's impedance d, m g at rest), release and re-attach. Say whether the approximations recorded in the
   translation report are the right ones and whether any unrecorded approximation exists.
3. G's chip fields and bounds, the `front_end` category and `minimum_exclusive` addition, catalog parameter
   dimensions and tiers against LIB-025 and LIB-027, placeholders flagged, SI inside and units in files.
4. Invariants 2, 3, 5 and 7 on these diffs (design-specific code, data as source of truth, engine types behind
   the adapter, failures as errors) and the style rules (braces, no ternaries, no comments without a why).
5. Ownership: list any file outside the brief's ownership list and whether the edit is acceptable.
6. Cross-brief: C adds `payloads` to `PhysicsOutputs` and a `payloads` argument to `translate` and
   `prepare_model`; say what the integration wave must do for the gantry to carry a payload, and whether the
   index-based `set_payload_attachment(p, kWorldBody)` release is acceptable against PHY-006.

Report, under 400 words, in this order: verdict per brief (accept, accept with fixes, blocked), findings by
severity (high, medium, low) each as `file:line, one sentence, the spec row`, rulings you need from the owner
(one line each), and a one-line cost note (calls used).
