# M3-A: ADR 0016 rope model; ADR 0010 open items closed

Agent: docs writer (general-purpose, Opus). Worktree `../MechatronicsSimulator-05-a` on branch `wip/05-a` from
`round/05-milestone-3`. Read first: root `CLAUDE.md`, `docs/adr/0000-template.md`, `docs/adr/README.md`, ADR 0001,
0003, 0009, 0010, 0013, ruling P4 in `docs/rulings/2026-11-02-planning-rulings.md`, round 5 rulings 2, 3, 4 in
`docs/queue.md` (the planned defaults are the proposals), spec rows ENG-047 (`docs/spec/eng.md`), BLD-032 (`bld.md`),
PHY-004 (`phy.md`), REC-026 (`rec.md`), LIB-020 (`lib.md`), ACC-037 (`acc.md`), `core/engine/CLAUDE.md` (lumped
beam, transmissions, constraints: what the engine has), the requirements doc's rope and swing text (search
"rope" and "swing" in `docs/Mechatronic Simulator — Requirements.md`) and the architecture doc's Decisions log
(search "Decisions log"). Call cap 40. No code, no tests, no builds. Commit once ("ADR 0016 rope model; ADR 0010
open items"), no trailer, no push.

## 1. ADR 0016: rope model (P4)

Template sections. Options to weigh (at least): (a) lumped chain of N point masses joined by unilateral springs
(tension only, slack below the rest length, viscous damping, EA from the catalog part, length change at a driven
end for a winch); (b) a single massless rope constraint with unilateral length (a point-to-point inequality,
no sag, no transverse dynamics); (c) a rigid link with a spherical joint (the milestone 1 stand-in, kept). Criteria:
swing dynamics for G12 (period, decay, settle), slack and snatch loads for the E-stop case, rope snap as a tier 2
failure (load release), determinism and cost per step, what the engine already has (lumped beam chain machinery,
constraints with unilateral handling absent today), the reference tests ACC-037 needs (rope pendulum period and
decay against the closed forms with the chain's added mass derived), and what the gantry team's real lift (a belt,
with a hook and a short rope or sling to the load) needs. Decision table as in ADR 0015: element shape in the
neutral model (`Rope{name, from_body, from_point, to_body, to_point, segments, EA, damping, mass_per_length,
rest_length, breaking_tension}`), tension published per step (PHY-004), slack rule, length change, the snap rule
(tension over breaking_tension for one step ends the attachment: tier 2, ENG-034), the pendulum reference set
(rigid link stays; rope pendulum with N segments converges to the point-mass period as N grows: state the
expected convergence and the tolerance proposal for ACC-037, T4), and the gantry placement (one rope from the
hook to the payload, replacing the spherical joint; the hook stays a body on the lift carriage). Consequences:
cost per step, the adapter interface (native only; the bridge is retired this round), builder (BLD-032), recorder
(REC-026), viewer swing overlay (VIE-011). Open items: multi-rope loads, rope on pulleys (M4 or later).

## 2. ADR 0010 open items closed (amendment section in ADR 0010, dated)

Tier 2 split: milestone 3 yield (permanent spring offset, ENG-031) and rope snap (load release, ENG-034);
milestone 4 buckling collapse (ENG-032) and fastener break (ENG-033). Halt configuration: a test-case field
`integrity.halt_on_crossing`, default false; the case fails on a crossing either way (INT-025, INT-026). State
both as decisions with the round 5 ruling numbers (3 and 4) as their source; the owner confirms at the gate.

## 3. Rows and indexes

Align the ENG-047, BLD-032, PHY-004, REC-026, LIB-020 (rope part parameters) and ACC-037 row texts with the ADR
where they differ (R-18; wording only, no tolerance change except the ACC-037 proposal marked "proposed"); ADR
README rows for 0016 and the 0010 amendment; Decisions log rows (next free numbers) in the architecture doc for
the rope model, the tier 2 split and the halt field; `docs/milestones.md` M3 "Other scope" names ADR 0016.

## Report

Under 200 words: files, the decision in three sentences, the ACC-037 tolerance proposal, Open issues.
