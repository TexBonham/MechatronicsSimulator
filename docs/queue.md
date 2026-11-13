# Dispatch queue

State for the coordinating session. A new session reads this, then `docs/rulings/`, then the two source docs.

## Current checkpoint

Round 5 (milestone 3) in flight on `round/05-milestone-3`, cut from main at cf5c74f on 2026-11-11 (round 4 merged by
PR #6). Plan of record `docs/briefs/round-05/plan.md` (signed off 2026-11-11, with the owner's amendment: milestone
3 retires MuJoCo in favour of the own C++ engine). Landed on the round branch: A (ADR 0016), M+D (f85803d), R
(163ad24), I1 (dafaa6c), E3 (bef4841: engine contact, members, ropes, mid-run changes, bindings, clamped far end;
ctest debug-sanitize 426, Python 4252 passed and 130 xfail), N with its fix rounds N2 and N2b (30e8fd9: the native
adapter over the own engine, opt-in through the run-configuration field `physics.adapter`, default `mujoco`; G3
and P1 pass on it with flexible rails, beam references within 2 percent; ctest debug-sanitize 454, Python 4320
passed and 117 xfail). E4 (a04a371: engine joint limits, joint friction, additive
per-step joint drive, zero-stretch coupling fix; ctest debug-sanitize 491, Python 4325 passed and 115 xfail). N3 (the adapter on the engine's limits, friction and drive; Python
4340 passed), I2 with its fix round I2b (bb27093: tier 1 integrity party, tier 2 yield and rope snap, end reason
`integrity_halt`; ctest debug-sanitize 526, Python 4377 passed and 107 xfail). E5 (regularised efficiency switch, screw lock
row), N4 (17f3d18: adapter follow-up, run-time mode lines, substep rest interpolation, node pose series; ctest
debug-sanitize 545, Python 4387 passed and 107 xfail). X1b with its fix round X1c (e83666a: structural acceptance
on native, node-pose deflection; ctest debug-sanitize 546, Python 4461 passed and 100 xfail). Integration branch `wip/05-x1a` (not merged to the round branch
yet): X1a's gantry work, E6 (lift jump fix, ruling 66), I3 with I3b (integrity and scoring fixes, swing angle),
E7 (moving load on a flexible member, rulings 63, 65, 67); verified there at ctest debug-sanitize 552, Python 4535
passed with 3 failures that only list shipped cases. In flight: E7b (carried-load stability, `wip/05-e7`), X1d
(finish G12 and G15, on `wip/05-x1a`). Next: land the integration branch, X2, gate (the MuJoCo removal commit
follows the owner's retirement decision). Briefs of record
in `docs/briefs/round-05/`; worktrees `../MechatronicsSimulator-05-<letter>`. Round 4's rulings and gate report
are below for reference; round 4's cost rows are in `docs/cost-log-archive.md`.

## Round 5 rulings (owner answers when available; the planned default applies until then)

| # | Question | Planned default | Status |
|---|---|---|---|
| 1 | Milestone 3 retires MuJoCo: every case on the native adapter, PHY-025 baselines, PHY-026 MuJoCo out of runtime and CI; fallback per ADR 0004 only if the contact solver or the differential check fails by the gate | retire | owner 2026-11-11 (amendment) |
| 2 | Rope model: lumped chain of point masses, unilateral tension springs with damping, slack below rest length, driven-end length change; the gantry payload hangs on one rope from the hook; a rope pendulum joins the rigid one (ACC-037) | ADR 0016 | open (owner reviews the ADR) |
| 3 | Tier 2 split (ADR 0010): M3 yield offset and rope snap; M4 buckling collapse and fastener break | as stated | open |
| 4 | Halt on a tier 1 crossing: test-case field, default false | as stated | open |
| 5 | Tier 1 integrity party in C++ (`core/integrity/`); tier 2 checks as Python plugins over the recording | as stated | open |
| 6 | Gantry structural placeholders: PETG printed 0.5 m rails with a placeholder rectangular section, static 1 cm target with dynamic reported, safety factor 1.5, swing limit 0.1 rad, settle 5 s, four segments per rail | as listed | open, gantry team |
| 7 | Materials: conservative printed defaults with wide spreads; tabulated temperature dependence, out-of-range lookups are errors; overrides in a design-referenced file; printed strength axes in-layer across the build direction, interlayer along it, cosine-squared blend | as stated | open (strength axes model owed by the owner) |
| 8 | G16 sweeps the two shipped stepper parts; LIB-017 end to end with it | as stated | open |
| 9 | Milestone 2 cases stay on the bridge until X2 switches the suite in one commit | as stated | open |
| 10 | Belt compliance in the own engine; the bridge keeps rigid belts | as stated | open |
| 11 | Models and reviewers as round 4 | as stated | open |
| 12 | Windows: a native-adapter test MSVC blocks is Linux-only, recorded | as stated | open |
| 13 | Differential tolerance (PHY-021): 1e-6 relative for drive-free references; one step angle times the ratio per tick of commanded speed for driven cases | proposed | open, owner at the gate |
| 14 | ADR 0016 (A's open issue): the rope damps only along its length, so swing decay for G12 comes from rotational damping at the hook attachment: the rope's top end hangs from the hook through a damped spherical joint whose damping is the design's `hook_damping` open parameter (and `hook_friction`), so the swing settle time is a design input, not a numerical artefact. E2 implements the attachment; the ADR gains the sentence. | as stated | coordinator ruling, for confirmation |
| 15 | ADR 0016: snatch peaks are underestimated at 1 ms when the segment frequency times the step exceeds 0.2; recorded as an approximation line by the adapter when a rope's segment frequency crosses it. | record | coordinator ruling, for confirmation |
| 16 | D: the gantry's X rail is clamped at both ends and the engine's lumped beam supports only a clamped or pinned root with a free or roller end. The lumped beam gains a clamped far end (end rotation locked, translation constrained), with the clamped-clamped closed forms as references (midspan deflection P L^3 / (192 E I), first frequency 22.37 / (2 pi) sqrt(E I / (rho A L^4))); implemented in the native-adapter wave (brief N or a short E3). | add | coordinator ruling, for confirmation |
| 17 | D: lumped beam segment count stays the engine's default of eight per member (ruling 6's "four per rail" withdrawn; ENG-040's convergence sets eight); a design may override per member (BLD-018). | eight | coordinator ruling, for confirmation |
| 18 | D: design ropes have no DES requirement row; DES-046 "a design file shall declare ropes with their attachments, part and rest length (ADR 0016)" is added at merge-back and cited by D's existing schema tests. | add | coordinator ruling, for confirmation |
| 19 | D review: `breaking_tension` is optional on ropes; absent means the rope never snaps (ADR 0016); BLD-032 and LIB-020 rows say so. | optional | coordinator ruling, for confirmation |
| 20 | D review: lumped-beam calibration scales E to the recorded deflection and density to the recorded frequency (exact on the chain); a scale factor outside [0.5, 2] is recorded as an approximation line naming the member and the factor (the section or material is probably wrong); the lumped member's mass then differs from the real one, recorded. The 2 percent tolerance (T4) holds for eight or more segments; fewer segments are refused for a calibrated member. | as stated | coordinator ruling, for confirmation |
| 21 | Integrator (E1 and E2 review derivation): backward Euler on the springs loses amplitude (1 + x^2)^(-pi / atan x), about e^(-pi x) per period with x = omega h, and runs x^2 / 3 low in frequency; a midpoint rule on K and D alone cannot remove it, a global trapezoidal update destabilises the explicit bias terms (gravity, Coriolis: the rigid pendulum references), and it is not L-stable for contact. Ruled: keep backward Euler in milestone 3; rope and belt cases run at a smaller physics step (ACC-037 at 50 microseconds with the step stated in its row; G12 at 100 microseconds, 0.1 percent per period at 1 m); every rope swing and belt mode publishes pi omega h as an approximation line; ADR 0013's reserved second-order tier (Newmark average acceleration or generalized-alpha with a bias Jacobian, contact dissipation by rho_inf) is opened as a scoped milestone 4 item. Fidelity statement for the owner: belt and beam first modes stay over-damped under backward Euler at any practical step (10 percent per period at 100 microseconds). | as stated | coordinator ruling on the review's recommendation, for confirmation |
| 22 | E2 review (ruling 15 reworded): the snatch flag compares the rope's fundamental sqrt(EA / (L M)) times the step with 0.2 (the segment wave frequency would flag every run); at that threshold the first snatch peak reads about 15 percent low, which the approximation line states. ADR 0016 amendment. | as stated | coordinator ruling, for confirmation |
| 23 | E1 and E2 review: ENG-019's oracle is the closed-form contact set (no reference library installed); ENG-042 asserts 2 percent (the N = 8 lumped critical load is 0.9872 of Euler's); the payload weld stays a penalty spring in milestone 3 and becomes a six-row fixed loop constraint with disable when the clamped far end lands (ruling 16); box-box and hull-hull contact keep one witness point in milestone 3 (payload-on-floor uses the per-corner plane form), noted in ENG-016. | as stated | coordinator ruling, for confirmation |
| 24 | R review: member end-loads (ENG-030, REC-004) are published in the member frame: x from the start to the end node, y and z the section's principal axes; the native adapter publishes that frame; the bridge's world-axis wrenches are rotated into it. | member frame | coordinator ruling, for confirmation |
| 25 | R review: a section carries both bending axes: `I_y`, `I_z`, `c_y`, `c_z` (extreme fibres) beside area, torsion constant and shear areas; bending stress is abs(M_y) c_z / I_y + abs(M_z) c_y / I_z (plus axial); the rectangular, circular and tube placeholders fill all of them from their dimensions. | as stated | coordinator ruling, for confirmation |
| 26 | R review: only tier 1 crossings emit events of kind `integrity`; tier 2 model changes (yield offset, rope snap) emit `model_change` events naming the change; scoring fails a case on `integrity` events only. | as stated | coordinator ruling, for confirmation |
| 27 | Integrity configuration lives in `integrity.yaml` beside the design file (no design field); absent file means no tier 2 checks and no table for that run, so every existing run is unchanged. X1 ships `designs/gantry/integrity.yaml`. Fastener grade ratings in the catalog and mount plate theory are owed rows (gantry team, M4). | as stated | planned default |
| 28 | I1 review: milestone 3 reports the static deflection from the recorded end-loads with E I per support condition (resolving the interior load to one point reads low for a cantilever under a distributed load, 5/48 against 1/8; the clamped-clamped case is a least-squares fit), flagged `approximation: static deflection from end loads`; deflection from the recorded chain node poses is an X1 item on the native adapter. Tolerances 1e-9 relative (strength, buckling, fatigue, fastener, impact, resonance, rope) and 1e-3 (deflection) as proposed (T4). | as stated | planned default |
| 29 | I1 review: `core_builder` keeps `rigid_members=True` on the bridge until X2 (ruling 9), so a design with members and an `integrity.yaml` has no member series on the bridge today; the FAC-013 conformance test uses a flexible bridge build; the native adapter builds lumped members per the design. | as stated | planned default |
| 30 | I1 review: impact rows carry their own flag `impact_margin_below_minimum`; the fastener check reads ratings from configuration only (catalog grade ratings owed); the member axis for direction-dependent strength is the member frame x of ruling 24. | as stated | planned default |
| 31 | E3 review (rewords ruling 17): the lumped chain's clamped-clamped deflection error is exactly -4/N^2 (first frequency +2.49 percent at N = 8, +0.61 percent at 16), so the segment default is eight per member and sixteen for a clamped-clamped member; the engine refuses a clamped far end below N = 8. | as stated | planned default |
| 32 | E3 review (rewords ruling 20): the 2 percent tolerance holds from N >= 8 for cantilever and pinned-pinned members and from N >= 16 for clamped-clamped; calibrating a clamped N = 8 member would scale E by 1.067 and density by 1.120 (12 percent mass), so a calibrated clamped-clamped member below 16 segments is a build error. The builder rows (BLD-017 support conditions, BLD-018 defaults) move with brief N. | as stated | planned default |
| 33 | E3 review: at a clamped far end the twist about the member axis and the axial slide stay free in milestone 3 (planar bending tier; a fixed slide locks the inextensible chain); the torsion lock lands with the six-row constraint of ruling 23 in milestone 4. A payload release is `release_load(Payload)` emitting `LoadReleased` with source Payload (no `PayloadReleased` event); a second release throws. | as stated | planned default |
| 34 | I2 brief: a tier 1 crossing raises one event per (member or fastening, quantity) per run, at the first tick strictly over the limit, with no re-arm; tier 1 runs on the native adapter only (lumped members), and a bridge run with an `integrity.yaml` carries an approximation line saying tier 1 was not evaluated. | as stated | planned default |
| 35 | I2 brief: milestone 3 tier 2 yield is a placeholder plasticity rule: at the member end with the larger moment, the nearest node spring takes a permanent offset (peak |M| - M_y) / k with M_y = sigma_y I / c and k = E I / l, growing only when the peak grows; each change is a `model_change` event and the run names the rule as an approximation. | as stated | planned default |
| 36 | CI after the E3 landing: the Windows debug test step took 57 minutes (unoptimised MSVC on the 50 microsecond rope reference; the release test step takes 32 seconds), so Windows debug runs `ctest -LE reference` and the reference tier runs on windows-release only. Linux keeps both tiers on both compilers under sanitizers (about 20 minutes). | as stated | planned default |
| 37 | N review: joint travel limits, joint friction and the per-step joint drive belong in the engine, not the adapter (ADR 0013: limits are solved at the velocity level with position stabilization; ENG-013, ENG-014 and ENG-051 are the rows of record). Brief E4 adds them to the engine; brief N3 then removes the adapter's penalty limit, damper stick and drive terms. This also answers the owner's 2026-11-05 note that the stepper hold is re-decided when the native adapter replaces the bridge: the hold is the engine's implicit joint spring, set per step. | as stated | planned default; owner re-decision owed at the gate |
| 38 | N review: joint friction stays regularised in milestone 3, the same law as the rope attachment and contact friction (rulings 14, 23): a joint loaded below its friction force L < F creeps at v_s L / F with v_s the slip threshold (1e-4 by default), published as an approximation per run; a true stick state is a milestone 4 item with the second-order tier (ruling 21). | as stated | planned default |
| 39 | N review: a joint drive acts through its transmission's efficiency by power direction on the native adapter, lossless at zero shaft speed so the static sag K q = tau stays exact; the bridge's lossless drive (PHY-034 row) is a stated bridge approximation. | as stated | planned default |
| 40 | N review: `RopeTension` across the kernel carries both values, `tension` at the load end and `max_tension` over the segments, with `slack`; `PhysicsOutputs` gains the step's model-change events, accumulated across engine steps within a tick and delivered once. Kernel contract change, owner-reviewed at the gate. | as stated | planned default |
| 41 | N review (rewords ruling 13 for driven cases): native against bridge on G3 differs by exactly three quanta of 12.5 micrometres in x (a homing-latch shift of two ticks) and 1.4e-5 m on the lift; the differential tolerance for a driven case is one full motor step times the axis ratio in position, three control ticks in time, plus the computed coupling sag; drive-free references keep 1e-6 relative. The bridge deviates from the kernel contract in two places (joint positions from the start coordinate, linear velocity at the centre of mass); X2 converts both before comparing. The adapter is selected by the run-configuration field `physics.adapter` (case or `--set`), recorded in the run identity and the archive, never by the environment. | as stated | planned default |
| 42 | E4 review: the engine's per-step joint drive is additive: it adds a spring-damper on top of the joint's model stiffness, damping and rest and never overwrites them; `clear_joint_drive` removes it; the rest passed is the end-of-step rest. | as stated | planned default |
| 43 | E4 review: a joint limit is an anticipatory velocity row s v >= -gap / h that lands the step on the bound with no position projection, so the crossing step adds no energy (the overshoot-then-clamp form could add up to (omega h)^2 of a coupled inertia's kinetic energy once per engagement); a start coordinate outside its limits is a model error. | as stated | planned default |
| 44 | E4 review: `scheme_decay` is published for driven joint DOFs only, each step, with omega from the total stiffness and the effective inertia; a DOF coupled through a compliant transmission uses the lower root of the two-DOF pair; beam nodes and undriven sprung joints publish nothing. Per-joint friction, damping, limit and drive values are model values under ENG-049 (`InvalidModelError`). | as stated | planned default |
| 45 | N re-review (rewords the published line of ruling 21): a run's approximation line for an oscillatory mode publishes x = omega h, the amplitude kept per period (1 + x^2)^(-pi / atan x) and the frequency ratio atan(x) / x, never the small-x form pi omega h (a gantry flexible rail at 1 ms has x = 1.30: 3.4 percent of the amplitude kept per period, frequency 30 percent low); a mode with x above 0.2 is flagged unresolved at this step. | as stated | planned default |
| 46 | N re-review: the contact rest penetration and friction stop references (PHY-029, PHY-030) hold on the native adapter only; the bridge differs by its model (its normal stiffness is not a spring, its friction loss is a soft constraint), recorded as a bridge approximation, with the adapter-neutral assertions (rest, drift, summed force m g) still run on the bridge. | as stated | planned default |
| 47 | N3 review: ruling 39 is unachievable as worded (the engine switches the transmission gain on the sign of the power, so a settling shaft never sits at exactly zero speed and every sag in [n eta, n / eta] F / K is an equilibrium; measured +3.2 % belt, +0.28 % gear). Options: reword ruling 39 to the band and publish it, or regularise the switch. | Regularise: the gain passes linearly through n inside a threshold shaft speed (the engine's `friction_slip_threshold`, frozen at the step's start rate, as ruling 38 does for friction), so the static sag is exactly n F / K and the approach creep is published; brief E5 | planned default |
| 48 | N3 review: the self-locking lead screw holds only while the shaft velocity stays bitwise zero (inertia condition J > n (h^2 k + h c)); a realistic rotor coasts once round-off moves it: an engine defect against ENG-044. | The engine holds a self-locking screw by a one-sided velocity row on the shaft (no motion in the back-driving direction unless the shaft's own drive or applied torque pushes that way); lowering under a drive stays unresisted in this tier (ENG-044 as written); brief E5 | planned default |
| 49 | N3 review: the adapter's `substeps` has a producer and a test but no run-configuration field sets it; with substeps above one every substep gets the tick-end drive rest, so steady lag is not exact. | `substeps` stays (the small-step rope and belt cases of ruling 21 use it); the rest is interpolated per substep; a run-configuration field for it is X1's when a case needs it; brief N4 | planned default |
| 50 | I2 review: the writer let the `integrity.yaml` halt value apply when a case leaves `integrity.halt_on_crossing` unset, so a case can halt without saying so. | Ruling 4 stands: halt is the test case's field, default false; no fallback to `integrity.yaml`; brief I2b | planned default |
| 51 | I2 review: a halted run reports end reason `end_time` with the halt line among the approximations. | A real end reason `integrity_halt` through the driver, archive, facade and scoring; the halt line is a report line, not an approximation (INT-026, invariant 7); brief I2b | planned default |
| 52 | I2 review: tier 1 evaluates live loads with no design-load factor. | As INT-023 to 025 are written: no factor in the party; a case that tests the design load carries it (rated load + 15 % on the gantry is the case's data) | planned default |
| 53 | I2 review: rope snap is not gated by `integrity.yaml`. | Rope snap is a tier 2 entry of `integrity.yaml` like yield: absent = no snap (ruling 27); brief I2b | planned default |
| 54 | I2 review: the yield oracle is asserted at 1e-3 against a derived bound of 2.3e-4 and no spec row carries it. | The INT tolerance row carries the derived bound 2.3e-4 (measured 8.1e-5), the crossing ticks exact, the cross-check and free fall 1e-9 (T4); the yield offset follows the running peak (sign of the peak; a larger reversed peak replaces it), stated in the row as the ruling 35 placeholder; brief I2b | planned default |
| 55 | E5 review: the writer made the screw's lock row two-sided when the output force and the shaft torque are both zero (needed to hold from an unloaded start), with no rest condition, so a moving unloaded screw is stopped dead and a body-wrench torque on an unloaded shaft is blocked. | The two-sided row exists only at exactly zero start-of-step shaft rate with zero output force and zero own torque; own torque counts the applied generalized force, body wrenches, the joint drive and the model joint spring and damping; at zero speed the implicit damping takes the larger loss of the two sides (dissipative, no shift of the equilibrium); E5 fix round | planned default |
| 56 | Pace (owner question 2026-11-12): the briefs are sequential by dependency and the wall-clock goes to the build lock (one build at two jobs; each writer ended with a full sanitizer and Python run that the coordinator repeats at landing). | Owner: do both: writers keep filtered runs only and the coordinator's landing verification is the one full run; `tools/locked_build.sh` runs builds and ctest at eight jobs (`MECHSIM_JOBS`), still one build at a time; the presets keep two jobs for CI | owner ruling 2026-11-12 |
| 57 | X1b review: node-pose deflection is asserted against the continuum closed forms within the lumped chain's 2 percent and against the chain's own value within 1e-6; ruling 28 had 1e-3 for the end-load formula. | INT-006 tolerance: 2 percent against the continuum (rulings 31, 32) plus 1e-6 against the chain value; supersedes ruling 28's 1e-3 for deflection | planned default |
| 58 | X1b review: the `second_moment` shorthand leaves the fibre distances optional (the brief said required); strength then uses c = sqrt(3 I / A) with the flag `approximation: single-axis section` (exact for a rectangle, 13 percent unconservative for a solid circle). | Optional with the flag, never silent; a design that wants a strength result of record states the fibres (the gantry's rails do, brief X1a) | planned default |
| 59 | X1b review: the ACC-039 fixture matches the scheme-corrected settle time, 9 percent under the continuous closed form. | The acceptance fixture runs at a step where the scheme's correction is inside the tolerance, so the case is judged against the continuous closed form; the corrected value stays as a published line (ruling 45) | planned default |
| 60 | X1b review: the acceptance cases' own criteria are one-sided (a wrong period, decay, settle time, a zero deflection or a wrong frequency passes the case; only pytest fails). | The oracle lives in the case: two-sided criteria for period, decay, settle time, settled deflection and frequency, with the scoring statistics they need (a period about the mean of a pose-derived series, a settled final-window value); pytest asserts the score and that a perturbed model fails; a NaN or not-evaluated quantity is an unevaluable error, never pass or plain fail (invariant 7); brief X1c | planned default |
| 61 | X1b review: ACC-037's 2 percent amplitude tolerance over 20 periods is half consumed by the scheme's known loss at 50 microseconds (measured -0.98 percent). | Kept as ADR 0016 proposes; the owner confirms the ACC-037 to 043 tolerance rows at the gate | planned default |
| 62 | X1b re-review (accept): the reference designs (rope pendulum, rope spring, cantilever, simple beam) are drafts with open parameters so a perturbed run can be made with `--set`; they validate (every parameter has a default) but the release guards no longer cover them and the oracle constants become settable against fixed criteria. Reviewer suggests release designs with the perturbed copies made in the test's temporary folder. Also carried, low: the S1 perturbed run fails on the target band, not the settle time; the tier 1 limits table still indexes nodes by layout; perturbed runs do not exercise the frequency criteria or R1's amplitude; no shipped design uses the catalog section `steel_bar_10mm_1m` now (LIB-030 lost its acceptance coverage); a structural steel material is owed before a fixture safety factor is quoted (the bars use the placeholder bolt steel, E and density only). | Keep the drafts for milestone 3; the conversion to release designs with temporary perturbed copies and the low items go to the round's close-out list for the owner | planned default, owner decides at gate |
| 63 | X1a finding: a carriage joint on a flexible rail loads the rail's parent body, not the beam: the engine has no way to put a moving load on a lumped member, so the gantry's rails carry only their own weight and G15's stress and deflection do not see the payload. Options: (A) load-path coupling: the design names the joints a member carries; each step the engine reroutes the carried joint's reaction onto the two adjacent beam nodes (lever rule) and takes it off the parent, so end loads, stress, deflection and tier 1 see the moving load, while the carriage's own motion ignores the rail's deflection (published as an approximation with the measured deflection); (B) full coupling: the carriage rides the deflected beam through a sliding constraint between node bodies (a new constraint type; milestone 4 size); (C) leave it and state that G15 covers self-weight only. | A, as brief E7 after E6: it needs a neutral-model field (which joints a member carries), an engine change and an ADR 0010 note; B recorded for milestone 4 | planned default, dispatched after E6 landed; the owner's ruling 69 (A1) builds on it; owner confirms at gate |
| 64 | X1a finding: the gantry's payload swing has almost no physical damping in the model (rope and attachment damping placeholders give at most 1e-4 of the energy per period), so the derived settle time is about 882 s against the 5 s placeholder limit; the numerical loss at 100 microseconds (0.18 percent per period) is larger than the physical loss, so ruling 21's one-tenth rule cannot be met at a runnable step. Peak swing passes its bounds. | ACC-026 (G12 settle time) stays a strict expected failure with the finding recorded; neither the physics nor the limit is tuned; the swing damping, the settle limit and whether the firmware gets a settling or anti-sway policy are owed by the gantry team (`docs/gantry-team-inputs.md`); the case states its numerical loss | planned default, owner decides at gate |
| 65 | I3 review: G15's rail_x reads a node-pose deflection of exactly zero. The lumped chain bends in one plane only (about the member frame's z; two-axis bending and torsion are the engine's next tier), and the gantry's rail frames put gravity along the frame's z, out of that plane, so the rails cannot sag at all. | Milestone 3: the design states each flexible member's frame so that the load of interest lies in its bending plane (the gantry's rails: vertical bending, frame y vertical); bending in the other plane is rigid and published as an approximation line per member; a build warning when gravity is entirely out of a horizontal member's bending plane; two-axis bending stays milestone 4. Brief X1d sets the gantry's frames; E7 reroutes only in-plane components | planned default |
| 66 | E6 (the lift jumping upward on an emergency stop at its lower limit; review: accept). Two causes: (a) adapter: with substeps above one, a drive whose spring law changes between ticks (a brake going from slipping to gripping) had its old rest interpolated under the new stiffness, a 405 N m kick; the adapter now blends the two ticks' spring laws per substep (stiffness and damping linear, stiffness times rest linear), identical to ruling 49 at equal stiffness; (b) engine: above the threshold speed the efficiency gain multiplied the end-of-step force, a non-symmetric coupling that adds energy when the power flow reverses within a step; the loss is now an implicit shaft damping c = alpha n F / max(omega, omega_s) with the lossless gain n in the stiffness term. | Amends ruling 47: the gains n / eta and n eta_back hold exactly for steady motion and the explicit torque; transients follow ruling 38's frozen-rate form (start-up from rest is slowed while c exceeds J / h); static sag, creep and the lock row are unchanged. An energy oracle (no drive, no input: kinetic plus gravitational plus elastic energy never increases) now covers limits, transmissions, the screw lock and a rope load | planned default (engine behaviour, owner confirms at gate) |
| 67 | E7 review (accept after a small fix round): the rerouted pair is explicit and lags one step; it is pure feed-forward when the rail's parent cannot move in the bending plane (the gantry's case), but on a parent free in that plane the lag is a negative damping in the member's mode and an undamped member grows when the carried mass exceeds the parent's. Also: at eight segments an off-centre carried load reads low by L^2 / (4 N^2 a b) (2 percent holds within about 0.27 L to 0.73 L); the carriage does not descend with the beam, so the beam's elastic energy P u is not taken from the carriage. | The engine refuses, at load, a carried joint whose parent is free in the bending plane beyond the derived stability boundary (an error naming member, joint and mass ratio; no clamp, no filter), and publishes the ratio below it; implicit (full) coupling is milestone 4; the position-dependent error is stated in the ENG-052 row; brief E7b | planned default |
| 68 | E7b confirmation (accept). Derived: an undamped member mode under the lagged pair grows exactly when r > (1 + x^2) / 2, r the carried share of the mass moving in the bending plane, x = omega h; the engine refuses on the full coupled spectral radius. Open, none blocking the gantry (its rail parents are fixed in the bending plane): the check is made once at the start position although r varies with the carriage's position along the span; the moment channel (a parent free in rotation about the bending axis) is not checked and can be unstable by the same recursion; the ENG-052 row's wording on mode coupling; an end-to-end `CoreDriver` test with a carrying member (comes with G15). | Close-out list for the gate: evaluate the check over the carried joint's travel and cover the moment channel before any design with a free-parent rail ships; until then the row states both limits | planned default, owner decides at gate |
| 69 | X1d finding on G15 (gantry on native, carried rails): the static checks hold (rail deflections under own weight, carriage and payload within 1.6 percent of the closed forms; end loads equal the weights carried), and after E6 the fastening forces at the stop are 1.39 and 1.34 kN against the 1.33 kN placeholder limit. But at the emergency stop rail_y, ringing at 50 Hz, drives rail_x, at 49.7 Hz, through the one-way coupling of ruling 63: rail_x stores 10.7 J where about 0.06 J is available (peak deflection 16.8 mm, safety factor 0.11). The one-way pair does work on the carrying member that the source never loses, and at matched frequencies it pumps without bound other than damping; the placeholder rails have equal sections, no stated damping and by accident nearly equal frequencies. Options: (A1) keep one-way coupling in milestone 3, publish the pair's work per member and flag the run unresolved when it is not small against the system's energy; G15's stop part stays an expected failure marked not valid until full coupling (milestone 4); (A2) pull two-way coupling into milestone 3; (A3) change the placeholder rails so the frequencies differ (rejected as tuning). Separate integrity defects found by the same run (design factor applied twice, strength from end moments only, no fastener rows, no window for the start transient) are fixed either way. | A1; brief E7c (work figure and flag) and brief I4 (integrity fixes) | **owner ruling 2026-11-13: A1; continue, the token budget is not a constraint for this round** |

## Round 4 rulings, reference (owner 2026-11-11: every ruling confirmed at its planned default; ruling 2 stays a placeholder with I2C and SPI peripheral modes added to the milestone 4 scope)

| # | Question | Planned default | Status |
|---|---|---|---|
| 1 | G9 hold on the bridge: the self-locking screw is refused by the bridge, so a de-energised lift falls. Brake on the lift as the `power_loss_hold` placeholder (a), a bridge self-locking approximation (b), or both (c)? E's firmware now turns the lift driver off in NOT_HOMED, so the hold rests on the brake output (reviewer: (a) or (c) needed for ACC-018). | (a) brake | confirmed 2026-11-11 |
| 2 | IMU electrical interface: the chip vocabulary has no I2C or SPI mode. Model the IMU with analog outputs (accel x, y, z; gyro x, y, z) until a serial peripheral mode exists? | analog outputs, placeholder | confirmed 2026-11-11 as a placeholder; I2C and SPI peripheral modes (chip vocabulary, signal layer, emulated peripherals) added to milestone 4 |
| 3 | FW-008 identical warning flags: on MSVC the host preset cannot share GCC flags. Read FW-008 as GCC host versus arm-none-eabi only, with the closest MSVC equivalent (`/W4 /WX`) on Windows? | yes | confirmed 2026-11-11 |
| 4 | Re-tag ENG-016 to 021 (own-engine collision and contact) and LIB-009 (Python slow-path plugin) from M2 to M3? | re-tag | confirmed 2026-11-11 |
| 5 | macOS stays out of the CI matrix for milestone 2 (R1 names Windows; the MuJoCo dmg cannot be extracted by CMake)? | out | confirmed 2026-11-11 |
| 7 | After an E-stop `reset` the firmware returns to NOT_HOMED (steps may be lost while power was cut), not READY? | NOT_HOMED | confirmed 2026-11-11 |
| 8 | Lift operator message is `lift <z>`, z in metres from the lift home switch; lowering is a lower z? | yes | confirmed 2026-11-11 |
| 9 | C review: for a drive-driven shaft under slip, only the explicit-torque path is scaled by (1 - s); the reported transmission force is the force actually applied. Recorded as an approximation. | as stated | confirmed 2026-11-11 |
| 10 | C review: a drive disabled then re-enabled under slip or backlash resets the slip and play references to the new rest, counting no increment. | as stated | confirmed 2026-11-11 |
| 11 | A1 review (high): the stepper detent tangent is taken at the predicted lead delta_p = theta_cmd - (theta + omega h), rest theta + omega h + tau(delta_p) / K_t(delta_p), pull-out on delta_p; exact in steady tracking, identical to milestone 1 at rest. SIG-033 row edited. | adopted | confirmed 2026-11-11 |
| 12 | A1 review (low): encoder noise and noisy-fault magnitude are in radians of shaft angle (the sensed quantity), not counts; alters the M1-D reading. | radians | confirmed 2026-11-11 |
| 13 | A1 review (low): debounce passes a pulse of N = periods_ceil(debounce_time) samples (the brief's rule), although an N-sample pulse is only known to have held (N - 1) periods. | N samples | confirmed 2026-11-11 |
| 14 | E review: FW-021 trips on any end switch active outside its own slow zone while READY or MOVING (wider than "moving away"); consistent with G11 "position tracking treated as wrong". | accept, row edited | confirmed 2026-11-11 |
| 15 | F1: `mechsim.viewer` may be imported under `mechsim/facade/` (the facade serves the viewer's probes, timeline and geometry, VIE-014); invariant 9 keeps display code inside `mechsim/viewer/` and that package still imports no Qt or OpenGL. `tools/check_imports.py` rule and its test updated. | allowed | confirmed 2026-11-11 |
| 16 | F1 review: series decimation is nearest-centre per equal bucket (the first sample when points >= samples); accepted as the ADR 0015 algorithm and written into FAC-018. | accept | confirmed 2026-11-11 |
| 17 | F1 review: the archive reader (`Kinematics`, `RecordingView`, `RecordArchiveReader`) moves under `mechsim/record/` with no import of `mechsim._core`, re-exported by `core_driver`; replay runs without the extension (VIE-016, REC-024). `run_suite` drops its seed and mode fields. | as stated | confirmed 2026-11-11 |
| 18 | B review (REC-018): index-column series for joints, transmissions and payloads carry per-column `names` and `units` group attrs supplied by the driver from the neutral model; archive format version 2, reader accepts 1 and 2. | as stated | confirmed 2026-11-11 |
| 19 | B: CTL-020 latency counts whole control ticks; CTL-021 slow-down ticks the controller once per `factor` periods (tick skipping), outputs hold between. | as stated | confirmed 2026-11-11 |
| 20 | A2 review: SCO-013 widens to layer `approximation` events (the load cell's saturation line) so they reach the score report; spec edit and scoring change in X2. | widen | confirmed 2026-11-11 |
| 21 | F2: the browser client is verified without a browser (protocol-kind scan against the schema, same-origin check, vendored three.js hash, a coarse domain-logic screen) plus a manual checklist the owner runs once at the gate; a playwright-driven test and its dependency are deferred to M3 unless the owner wants them now. | deferred | confirmed 2026-11-11 |
| 22 | F2 review: `list_runs` and `advance` are added as request kinds in X2 (additive); until then the client plays back on its own clock issuing `scrub` per frame, and lists only runs it started. The X2 client fix also keeps the cursor's own sample in the decimated window. | X2 | confirmed 2026-11-11 |
| 23 | X1 blocker: the bridge refuses a drive on a transmission's driven joint, so the lift brake was not applied. The brake acts on the motor shaft joint (a motor brake), and the signal layer combines drives published on one joint as parallel springs (K = sum K_i, rest = sum K_i r_i / K, D = sum D_i; exact for linear springs) before publishing, so KER-027's one-drive-per-joint rule holds at the kernel. | as stated | confirmed 2026-11-11 |
| 24 | X1 blocker: a payload attached in the case's initial state is built as a fixed child body of its carrier (rigid, no equality); the weld equality is used only for a mid-run attach or release (`set_payload_attachment`). BLD-036 row as X1 wrote it, amended. | as stated | confirmed 2026-11-11 |
| 25 | G7: the gantry's `estop_relay` instance runs `manual_reset: false` as the placeholder (the motor rail returns when the E-stop loop closes); the firmware's `reset` message gates motion after an E-stop (FW-024), which is what ACC-015 observes. Gantry team owes the real relay's reset arrangement. | as stated | confirmed 2026-11-11 |
| 26 | X2: with the catalog 8 mm lift lead the lift parked on its lower joint limit under the stepper's hold diverges on the bridge (K A0 h^2 = 2.94; soft limit against a stiff implicit drive at 1 ms). The design keeps a 40 mm lead placeholder (K A0 h^2 = 0.50) until the limit handling is fixed (bridge limit softness, or the own engine). The gantry team's real lead needs that fix. | 40 mm placeholder | confirmed 2026-11-11 |
| 27 | X2: pass criteria gain a `rail:<name>` ref (the archive's `signals/rails/<name>` series) so G7 and G8 can assert the motor rail off within the cutoff delay (ACC-013, ACC-016) instead of the weaker "no step pulses". | add in X3 | confirmed 2026-11-11 |
| 28 | X review and gate audit (DES-045): `firmware.event_codes` entries carry an argument kind (`axis`, `fault`, `state`, `number`, `none`); the decoder labels an axis argument with the step channel's instance, a fault or state argument with the design's code name, a number verbatim. | as stated | confirmed 2026-11-11 |
| 29 | X review (invariant 7): a brake the physics carrier cannot apply ends the build with an error naming the brake and the joint, never an approximation. | error | confirmed 2026-11-11 |
| 30 | X review (ACC-020, ACC-023): pass criteria gain a `fault:<name>` ref (the controller's fault code by name at a time or over a window) beside `state:` so a case asserts the fault kind itself. | add in X4 | confirmed 2026-11-11 |
| 31 | X review: the placeholder NEMA 17 on the 40 mm lead cannot lower a 4 kg overload or take a 1 to 4 kg step (G5, G6 use 3.6 kg in 1 kg steps); recorded in `docs/gantry-team-inputs.md` as an owed motor and lead sizing item. The 5 N axis friction placeholder keeps the coasting carriage inside G7's homing window (the alternative is a longer window). | recorded | confirmed 2026-11-11 |
| 32 | C2 and G10: the gantry's encoders sense the axis joints (frame side; the inputs record says so), not the motor shafts, so belt slip and stall both show as a step-count versus encoder mismatch. Design change in X4. | frame side | confirmed 2026-11-11 |
| 33 | C2 (PHY-034): the bridge applies each joint drive as exact backward Euler (`eq_data = rest c - qvel h`, `d = Z / (1 + Z)`, `Z = (K_d h^2 + D_d h) A0`), consistent with the stepper's end-of-tick rests; this changes milestone 1 drive behaviour (the reference tier and the G1 to G4 baselines are re-validated at the gate). The brake slip phase is a soft slip drive (`K_s = 1e-5 K_b`). | adopted | confirmed 2026-11-11 |
| 34 | Gate review: ACC-038 (stepper pull-out versus the published curve) is M2-tagged but was not in the plan's scope and stays a strict-xfail stub; re-tag to M3 with brief P's engine work, or keep and note the omission. | re-tag M3 | confirmed 2026-11-11 |
| 35 | X7 (G6): with the 10 lb rated load and the NEMA 23-class placeholder motor on a rigid belt, the load cell overshoots about 20 percent at the start of a lift and trips the two-tick overload filter before the real mass step. The amplifier's placeholder bandwidth is 2 Hz (gantry team owes the real filter or overload rule), the firmware filter stays, and G6's bound is derived with the detection latency included. | as stated | confirmed 2026-11-11 (option 1 now; belt compliance in the model in milestone 3, G6 re-derived then) |
| 6 | Placeholders set by the coordinator for the gantry team's slots: overload limit rated + 15 percent (3.45 kg), heartbeat period 10 ms, watchdog timeout 50 ms, brownout threshold 2.0 V, encoder mismatch threshold 2 mm, ADC 12 bits at 3.3 V | as listed | confirmed 2026-11-11 |## Milestone 2 gate report (coordinator, 2026-11-11)

Fable gate review at f01c3a7: pass with owner confirmations. Reference tier 27 of 27 C++ and 9 Python, no
tolerance miss; the 13-case suite passes with a 74-entry baseline; G3 byte-identical on a rerun; debug-sanitize 347
ctest cases; firmware host 92; Python 3707 passed, 191 xfailed. Round cost about 6.0M tokens against the 5.5M to
6.1M estimate (the integration wave cost 2.5M against a planned 1.0M: the drive-mapping and payload rework).

CI on the merged round branch: green on all three workflows, both runners, at feb19dd (2026-11-11) after three
Windows-only fixes in the facade's file handling and its tests (path form and line endings). The gate's CI condition is met.

Owner confirmations needed (change milestone 1 behaviour or the architecture): rulings 33, 23, 11, 24, 25, 29, 12,
15, 17, 18, 32. Confirm in a batch: 9, 10, 13, 14, 16, 19, 20, 22, 27, 28, 30. Gantry team placeholders: 1, 2, 6, 7,
8, 26, 31. Process: 3, 4, 5, 21, 34.

Carry to milestone 4: I2C and SPI peripheral modes in the chip vocabulary, the signal layer and the emulated
peripherals (ruling 2), after which the IMU and any serial sensor drop their analog stand-in.

Carry to milestone 3: transmission (belt) compliance in the physics model so the load-cell transient at lift start is
realistic (ruling 35, owner 2026-11-11); G6's amplifier placeholder and bound are re-derived then.

Carry to milestone 3 by ID: ENG-016 to ENG-021, ENG-035 (engine side), LIB-009, ACC-038 (ruling 34), PHY-020,
PHY-021, the playwright client test (ruling 21), the limit handling for the real 8 mm lead (ruling 26), LIB-017
through the real builder (needs a second shipped stepper part), the G6 `fault:overload` assertion (G6 proves the
position bound only), G13's "defined policy" once the gantry team rules.

Silent dependencies (gate review item 4): the case windows hard-code sums of placeholders (G7 12.021 s = press +
relay cutoff; G8 9.07 s = freeze + heartbeat + timeout + cutoff; G14 0.5 to 2.3 s from accel, cruise and slow zone;
G6 25 cm from the lift speed; command times from the homing speed), all riding on `gantry_config.h`,
`watchdog.yaml`, `estop_relay.yaml`; a placeholder change breaks a window silently until the windows become open
parameters (M3 housekeeping). `docs/gantry-team-inputs.md` contradicts itself on the encoder mismatch threshold
("not set" versus the ruling 6 row, 2 mm): the 2 mm row stands.

## Next

1. Wave 2 reviews (A2 with B, F1 alone), merge-backs, then wave 3: F2 (after F1) and X1 (merges `wip/04-e`, binds the
   load cell, amplifier, E-stop status, brake and heartbeat channels, event-code table, payload in the neutral model,
   fault routing through B's scheduling; G5 to G8). Then X2, X3, the gate.
2. Owner ruling 2026-11-05: the implicit joint drive stays while the MuJoCo bridge is the physics carrier; the
   question of how the stepper hold is applied (drive input versus the own engine's implicit springs, ADR 0013) is
   re-decided when the native adapter replaces the bridge (PHY-020, ADR 0004 retirement).
3. Contract files over 4 500 characters after the sweep (core/bindings 6.7K, mechsim/testing 6.4K, core/kernel 6.2K, firmware 4.9K, core/physics/mujoco 4.8K, core/signals 4.7K, core/engine 4.6K): current contract text, split or accept at the milestone 2 close.
4. Owed from reviews (round 5, R re-review 2026-11-11): the design's explicit section form (`area` plus one
   `second_moment`) lacks `second_moment_y`, `fibre_y`, `fibre_z`, so the stress overlay refuses such members;
   acceptable while every shipped placeholder is a shape (ruling 6); X1 adds the fields.
   FAC-013 acceptance conformance stub stays: a flexible bridge run of a pendulum with one lumped member diverges
   (MuJoCo QACC NaN clamped-free, constraint drift pinned-roller) while the facade path works; X1 converts it on the
   native adapter with the gantry integrity file (ruling 29).
5. Owed from reviews: two actuators on one shaft (builder sums rotor inertias, SIG-037 then refuses); PHY-020 native
   adapter and PHY-021 differential tolerance; mypy lambda note at tests/acceptance/test_acc_stubs.py:30.

## Milestone 1 rulings (owner, 2026-11-04)

Fault parameter forms (noise absolute SD, stall available-torque fraction, slip fraction); controller precondition
checked before t = 0, mismatch is an error; watchdog type and E-stop heartbeat input added now; DES-041 wiring is
connectivity only, physical wires a later component type; ASCII line operator codec for M1; MuJoCo pinned at the latest
3.9 patch via CMake; pybind11 through scikit-build-core; engine package 7 in parallel, not gating M1; chip definitions
as data with an STM32-family placeholder, real parts drop in by file. MVP: milestone 2 for the gantry team, milestone
3 plus CLI and replay for the standalone product.

## Rulings made during milestone 1 waves 1 and 2 (coordinator, 2026-11-04), confirmed by the owner 2026-11-05

- Direction and enable: the pin-table-bound digital outputs are the single source; the native shim derives the
  stepper arrays from them; firmware holds the stepper arrays low.
- Frozen controller: latched levels hold, per-tick buffers are empty (no pulses, no bytes, no events); timer-driven
  pulses that continue on a hang are a milestone 2 option.
- Reset and brownout recovery reload a private copy of the firmware library, so no firmware static survives.
- Board interface 2.2: `mechsim_app_board_interface_version()` entry point implemented by every app.
- Physics step is run configuration, not design data; the builder no longer ties it to the firmware tick.
- Milestone 1 runs rigid: BLD-033 rigid-members option; the gantry ships placeholder primitive geometry until CAD.
- Kernel: control_rate maps onto the controller party by role; a physics party whose rate differs from
  physics_rate is a config error.
- MuJoCo bridge transmissions (from the C review): a joint equality leaks metres with a light shaft on a heavy
  load, so the bridge maps a rigid transmission by reflection: no shaft hinge in MJCF, rotor inertia reflected
  as `armature = J / c^2` on the driven joint, shaft torque applied as `tau / c`, shaft position and velocity
  reported as `q / c`, `qd / c`. Efficiency by power direction (driving `eta tau / c`, back-driven `tau / (eta c)`).
  `self_locking: true` refused in the bridge (own engine handles it); finite stiffness treated as rigid and recorded
  as an approximation; backlash and slip stay refused until milestone 2.
- Shaft load torque (ENG-048, SIG-033): `tau_load = tau_applied - J_rotor * alpha_shaft`, positive along the shaft's
  + axis, published as the shaft joint's generalized force; the stepper's demand is `J alpha_commanded + tau_load`.
  Rotor inertia has one source, the motor instance's `rotor_inertia`: the builder copies it into the shaft body and
  the signal layer reads the same value.
- Explicit MuJoCo contact pairs bypass the parent-child filter: the bridge skips pairs between joint-connected bodies
  and records them.
- PHY-028 pendulum period tolerance set to 0.1 percent (T4 spec edit; the reviewer measured 0.0135 percent).
- Signal layer: counters refuse a forced-level fault (recommendation accepted); relay floor, debounce ceiling and
  watchdog rounding to whole ticks accepted; UART pacing is 8N1 at the configured baud.
- Catalog alignment owed to the integration brief: `stepper_behavioral` gains a torque-speed table parameter
  (holding torque alone stalls any motion above zero speed); `estop_relay` keeps no heartbeat input, the watchdog's
  trip output drives it; power supply output named `v_out` to match the pin.
- Milestone 3 note: MuJoCo implicitfast treats member-node stiffness explicitly, stable only for omega h < 2 per
  segment, so the bridge cannot be the oracle for fine-segment beams; retirement baselines stay inside that envelope.
- Engine binding surface (from the G review, invariant 5): engine types live in the submodule `_core_ext.engine`,
  importable only from `mechsim/physics/native/` and `tests/`; the import-boundary check enforces it. Neutral types
  stay flat in `mechsim._core`.
- Own-engine load torque (J): `shaft_load_torque = -g F` is the torque the transmission applies to the shaft,
  evaluated at the end state; the bridge publishes `tau_applied - J alpha` (the torque the shaft must supply). The
  integration brief aligns both to PHY-033 (the shaft-must-supply sign) so the stepper reads one convention.
- ENG-045 and ENG-046 (gear pair, slip parameter) pulled forward from milestone 2: delivered in package 7.
- Self-locking screw lowering under drive is unresisted in this tier (no nut-friction model); recorded in eng.md.
- Sensor placement is design data (DES-043, from the X2 blocker): a sensor instance carries `senses` (joint; limit
  switch position and side; encoder reversed) and the builder maps it to the signal layer's joint coordinate.
- Open parameters resolve case value, then the design's `default`, else an error (DES-044); the gantry draft ships
  placeholder defaults flagged until the gantry team rules.
- Gantry lift screw runs with zero backlash in milestone 1 (bridge refuses backlash until PHY-006, M2).
- Generated channel header names (`MECHSIM_CH_<SIGNAL>`) are the firmware convention; the committed gantry header is
  the builder's output, identity-tested.
- Kernel coupling values may be typed immutable payloads beside scalars (KER-008 row); KER-023 scheduling rule as
  written in ker.md; the physics party tests drift against the larger of the two thresholds until health carries
  per-kind drift (KER-016 row).
- Behavioral stepper is position-commanded (SIG-033 model defect found by G1 and G4: the feedforward torque law
  let the enabled lift creep under gravity): restoring torque `tau_hold sin(pi delta / (2 step_angle))` on the
  commanded-minus-shaft angle, viscous term, capped by the torque-speed curve; pull-out at two step angles
  re-synchronizes and counts missed steps, stall when it happens at zero commanded speed; disabled is zero torque.
- Design firmware section declares `state_codes` and `fault_codes` name maps (DES-045); the controller state the
  runner reports is `{state, fault}` by name and a case precondition names a state. Operator status message carries
  `state` and `fault` fields. App target `mechsim_fw_<design name>` is the convention.
- Stepper hold through physics (X4 found the signal-layer hold spring explicit and unstable on the lift at 1 ms,
  omega h 2.3): `PhysicsInputs.joint_drives` {joint, rest, stiffness, damping, enabled} applied implicitly by the
  adapter (MuJoCo: soft joint equality with direct solref; own engine: ADR 0013 implicit spring); the stepper
  publishes its commanded angle as the drive rest with the linearized detent stiffness and reads the drive torque back
  for pull-out and stall. Owner 2026-11-05: keep the drive on the bridge; re-decide with the native adapter (PHY-020). The 0.25 ms
  alternative (holds, 4x slower) stays recorded; the UART byte-loss defect it exposed is fixed (SIG-038).
- No-motion criterion (ACC-001, ACC-007): no step pulse commanded on any axis for the whole run and no axis leaving
  the power-on hold band, placeholder 1 mm (hold sag plus ringing; the gantry team owes the number). Hold ringing of
  a few milliradians is physical, not motion. Stepper damping placeholder 2e-3 N m s/rad (damping ratio near 0.1 at
  the lift).
- Signal layer accumulates controller-facing per-tick buffers (UART bytes, counter deltas) across its own steps and
  clears them at controller-tick boundaries (the root cause of the UART loss at 4 kHz physics with a 1 kHz tick).
- Owed after the gate: recorded joint positions and drive torques series; stepper sinusoidal detent (M2).
- Stepper above-curve torque (X review Q1): the motor loses synchronism whatever the lead; re-synchronize by at least
  one step toward the load, count it, stall at zero commanded speed.
- Shaft body axial inertia is the motor's rotor inertia, set by the builder (BLD-034 reworded); a design shaft body
  declaring its own is a build error; SIG-037 refusal wired through the Python wiring.
- PHY-034 drive mapping envelope: K_d A0 h^2 < 400 (omega h < 20), AdapterError beyond; the gantry lift sits at 2.7.
- Reference tier (validator): BLD-010 test asserts 1e-12 against a "not set, proposed 1e-9" row; SIG-011 asserts an
  exact count against "not set". Owner sets both rows at the gate (T4).
- Owner question for the gate (C re-review): where the transmission loss sits. The bridge reflects the rotor
  lossless, `(m + J/c^2) a = eta tau / c`; if the loss is in the nut or belt the exact rigid relation is
  `(m + eta J/c^2) a` driving and `(m + J/(eta c^2)) a` back-driven (5 percent of acceleration on the review
  fixture). Both adapters must use the same reading; the lossless one stands until ruled.

## Rulings made overnight 2026-11-03/04, confirmed by the owner 2026-11-04

- Kernel time: integer ticks of a per-run base period (1 / lcm of all party and physics rates), rates as rational
  hertz, instead of int64 nanoseconds with whole-nanosecond periods (which rejected 3 kHz, 60 Hz and sub-hertz rates).
  Reviewer's recommendation; cheaper now than after milestone 1 builds on it.
- Firmware-declared tick rate (`mechsim_app_tick_hz`) kept as a consistency check: the runtime errors at load when it
  differs from the design's control rate; the host build injects the design's value so 1 kHz lives only in data (C4).
- Board interface sizes are fixed contract constants shared by the C header and the C++ PinInputs/PinOutputs: 32
  digital in, 8 counters (per-tick deltas), 16 ADC, 64-byte UART each way, 8 axes, 32 digital out, 16-entry event log
  with state and fault codes (CTL-018 channel). The rail voltage is not a firmware input.
- PhysicsAdapter.load takes a prepared model (neutral JSON plus an engine artifact such as MJCF) per ADR 0014.

## Owner rulings 2026-11-04 (from the C3 and C4 reviews)

- Operator message labels: split into two messages, confirmed.
- Heartbeat routing: a pin signal into the E-stop relay or watchdog, not an operator message (applied in the C3 fix
  round). The catalog has no watchdog type yet (LIB-019, M2) and estop_relay has no heartbeat input; both are owed.
- Contact model parameters and the floor live in the design environment (DES-042, M1).
- Instance wiring and rail consumers are DES-041 (design file), owed before the builder brief.
- G3 targets carry units: 40 cm, 25 cm in the homed origin frame (G2). Per-axis 1 cm criterion confirmed. P1 damped
  period at 0.1 percent confirmed. ADR 0015 accepted.
- Needs discussion with the owner: fault parameter forms, the controller precondition check, heartbeat wiring once a
  watchdog type exists.
- Gantry kinematic chain, hook on a spherical joint standing in for the M3 rope, joint limits 0 to workspace: writer
  defaults until the gantry team's CAD arrives; record the hook placeholder in the M3 rope ADR.

## Rulings made at merge-back, confirmed by the owner 2026-11-03 with the round 1 commit

- Steppers and drivers are signal-layer models (SIG-032 to SIG-036); transmissions, ropes and lumped beams are
  engine-side neutral-model elements (ENG-043 to ENG-048). Shaft torque crosses out, load torque crosses back.
- ENG-011: the sixth joint type is the free joint (six degrees of freedom) for unattached bodies.
- Flexible-member tuning: the builder tunes (BLD-019); integrity imports FEA results into the design file (INT-034).
- Tier 1 integrity reads live loads through the kernel; everything else reads the recording (REC-001 carve-out).

## Parked

- C++ strict expected-failure convention for Catch2 stubs (ADR 0008 open item; `[!shouldfail]` is the candidate): rule in the phase 0b plan.
- Stage-review nits carried to the next docs-indexer pass: hooks and `tests/` outside mypy scope; engine package working names in `docs/milestones.md` unruled; `INV-nnn` numbering assumed equal to invariant numbers until the spec exists.

- Chrono adapter status (ADR 0004 open item): decide with the phase 1 contracts.
- C++ test markers for requirement citations (ADR 0008 open item): phase 0c.
