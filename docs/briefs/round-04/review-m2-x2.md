# Review: integration wave, second part (X4, A4, X5; C2 reviewed separately)

Reviewer: Fable (general-purpose). Read-only; spec and diff only; never a writer's report; no builds, tests or CLI
runs; call cap 50. Worktree `/home/tex/dev/MechatronicsSimulator-04-x1`, branch `wip/04-x1`, range db7e1de..HEAD
excluding C2's commits (6032236, 0814bb6, already accepted): review `git -C <worktree> diff db7e1de...88aa526`
and treat the bridge and brake changes from C2 as accepted context. Briefs `m2-x4.md`, `m2-a4.md`, `m2-x5.md`;
rulings 22, 28 to 33 in `docs/queue.md`; the first-part review's findings (its high on G7 and G9 hold criteria,
its mediums) must be closed by this diff. Spec rows ACC-013 to ACC-028, PHY-016, DES-045, SIG-011, SIG-014,
SIG-020, BLD-035, BLD-036, SCO-021, SCO-025 and the `rail:` and `fault:` refs, REC-015 to REC-018, LIB-031,
LIB-017, FAC-011, VIE-009, FW-026 to FW-028.

Check: (1) G7 and G9 now lift before the fault and the hold criteria measure the brake, not the joint limit; G10's
slip and stall variants with frame-side linear encoders (ruling 32; resolution as metres per count) assert the
mismatch fault by name (ruling 30); G13 and G14 criteria against ACC-027 and ACC-028 with `no_overlap` and
`interval` on real events; every criterion window derived, none chosen from measured values. (2) Event argument
kinds (ruling 28), the brake build error (ruling 29), no stand-in payload (BLD-036 row amended; load cell with no
target reads zero), first-tick firmware events kept. (3) Export headers with per-column names and units. (4) The
carry-overs X5 reached: LIB-031 types, `list_runs` and `advance` with the client, LIB-017 through the real builder.
(5) Invariants 1, 2, 3, 4, 7; style; ownership; R-18 spec edits; `docs/queue.md` must not appear in the writers'
diffs (W-05).

Report, under 500 words: verdict, findings by severity as `file:line, one sentence, spec row`, owner rulings
needed, cost note.
