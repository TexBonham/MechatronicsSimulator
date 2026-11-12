# ADR 0016: Rope model

Date: 2026-11-11
Status: Proposed (owner reviews at the milestone 3 gate; round 5 ruling 2)
Closes rulings: P4; ADR 0003 and ADR 0010 open item "Rope model"; round 5 ruling 2 (planned default)
Supersedes: none (refines ADR 0003, P3 and P4; the milestone 1 rigid link stays)

## Context
The requirements doc asks for ropes and cables carrying suspended loads that swing as pendulums, with rope length
changes during lifting and slack when unloaded (Physics and structural analysis requirements), and for "loads hung on
rope or cable" with mass, attach point and rope length (Component library requirements, Loads row). G12 measures swing
after a fast move and after an E-stop (peak angle, settle time); G15 includes an E-stop with a swinging load; acceptance
criterion 3 is rewritten as the rope pendulum's period and swing decay against closed forms (ACC-037, Decisions log
row 24). P3 makes ropes our lumped models in the neutral form; P4 deferred the detail to this ADR and kept the
milestone 1 pendulum a rigid link. Round 5 ruling 1 retires MuJoCo this milestone, so only the native adapter maps the
rope. The own engine (ADR 0001, ADR 0013) already has: point bodies on joints in a tree, compliant spring-damper
couplings folded into the implicit K and D (transmissions), lumped beam chains (ENG-022), bilateral loop constraints
solved by Gauss-Seidel with projection, and mid-run model changes planned for tier 2 (ENG-031 to ENG-034). It has no
unilateral constraint handling. The gantry team's real lift is a belt moving a carriage, with a hook and a short rope or
sling to the load, so the gantry needs swing, slack and snatch, not winch length change; a general product (R-01)
still needs the length change.

## Options

**A. Lumped chain.** N segments; N - 1 interior point masses joined to each other and to the two end bodies by
unilateral springs: tension only, zero force below the segment rest length (slack), axial viscous damping, axial
stiffness EA / l from the catalog part, rest length changed at the driven end for a winch.

**B. Massless unilateral length constraint.** One point-to-point inequality, distance at most the rest length; no
sag, no rope mass, no transverse dynamics of the rope itself.

**C. Rigid link on a spherical joint.** The milestone 1 stand-in: the load swings on a fixed-length bilateral link.

| Criterion | A. Lumped chain | B. Unilateral constraint | C. Rigid link, spherical joint |
|---|---|---|---|
| G12 swing (period, decay, settle) | Period with the rope's added mass and stretch; decay from what the model damps | Exact massless pendulum while taut | Exact rigid pendulum; joint damping available |
| Slack and snatch loads (E-stop, G15) | Slack per segment; snatch force finite, set by EA and the load mass | Slack yes; snatch is an impulse, so the peak force scales with 1 / h and is step-dependent | No slack; the link pushes in compression, non-physical at an E-stop |
| Rope snap, tier 2 load release (ENG-034) | Tension is a force each step; compare with breaking tension, release the attachment | Tension only as impulse / h; the snap threshold becomes step-dependent | Possible by disabling the joint, but the force is a constraint force with no rope physics |
| Determinism, cost per step | Fixed sequence; 3 (N - 1) DOF added, no iteration | Needs a complementarity (active-set) solve: an iteration count that varies with state | Cheapest; already exists |
| What the engine has | Springs in implicit K and D (transmissions), chains (lumped beam); needs a 3-DOF point node and a tension-only switch | Unilateral handling is absent; a new constraint class and its solver | All of it |
| ACC-037 reference | Chain period with the added mass derived (closed form below); converges in N | Point-mass period only; rope mass cannot be represented | Not a rope; ACC-033 and ACC-034 already cover it |
| Gantry lift (belt, hook, short rope or sling) | Fits: one rope hook to payload | Fits only while taut | The milestone 1 to 2 model; no slack at E-stop |

## Decision
Option A. Option C stays as the rigid pendulum (ACC-033, ACC-034) and as any design's explicit choice; option B is not
built.

| Element | Decision |
|---|---|
| Neutral model element | `Rope{name, from_body, from_point, to_body, to_point, segments, EA, damping, mass_per_length, rest_length, breaking_tension}`; points in the body frame; `from_body` may be `world`; SI (ADR 0012) |
| Units | `EA` N; `damping` N*s (axial force per unit strain rate); `mass_per_length` kg/m; `rest_length` m; `breaking_tension` N, absent means the rope never snaps; `segments` integer, at least 1 |
| Segments | Default 8 per rope, overridable per rope in the design file (BLD-032), as BLD-018 does for beams |
| Chain built by the engine | Segment rest length l = rest_length / N; N - 1 interior nodes, each a point mass mass_per_length * l on a new 3-DOF translation joint to world (no rotation); the two end half-masses mass_per_length * l / 2 are added to `from_body` and `to_body` at their attach points, so the rope mass is conserved |
| Segment force | With d the segment length and n its unit vector: T = (EA / l)(d - l) + (damping / l) d_dot when d > l, clamped at 0; T = 0 when d <= l (slack); force T n on the lower end, -T n on the upper end |
| Implicit treatment (ADR 0013) | A taut segment adds k n n^T + (T / d)(I - n n^T) to K (axial plus geometric stiffness) and c n n^T to D, with the same a b^T coupling as transmissions; a slack segment adds nothing. The geometric term is required: the highest transverse chain mode is about 2 N sqrt(T / mass_per_length) / L, about 2000 rad/s for the gantry placeholder (T = 29 N, 0.02 kg/m, L = 0.3 m, N = 8), so omega h is about 2 at 1 ms, at the explicit limit |
| Slack rule | Taut or slack is decided per segment from the state at the start of the step and held for the step; no event location, no iteration, deterministic (invariant 4) |
| Tension published per step (PHY-004) | Every segment's tension at the end of the step, plus per rope the maximum segment tension and the tension of the segment at `to_body` (the load end) |
| Length change | `set_rope_rest_length(i, L)` input applied at the next step: every segment's rest length becomes L / N; node masses stay at the built value (rope mass fixed at the build rest length); L at or below zero, or not finite, ends the run (invariant 7). A drum coupling (winch transmission) is an open item |
| Snap rule (tier 2, ENG-034) | When any segment's tension exceeds `breaking_tension` at the end of a step, the rope's attachment to `to_body` is removed from the next step on (load release); the chain stays hung from `from_body`; a timestamped tier 2 failure event names the rope, the segment, the tension and the limit; the published load-end tension is 0 afterwards. One step over the limit suffices; no filtering |
| Rigid pendulum | Stays: designs/pendulum's rigid link and ACC-033, ACC-034 unchanged |
| Rope pendulum reference (ACC-037) | A second pendulum case: rope `from_body` world, `to_point` at the bob's center of mass (so the bob's rotation decouples), bob mass M, rope mass m_r = mass_per_length * L, small amplitude (0.02 rad, amplitude correction theta0^2 / 16 = 2.5e-5) |
| ACC-037 period closed form | Straight-mode Rayleigh quotient of the chain as built: I_N = M L^2 + m_r L^2 (1/3 + 1 / (6 N^2)), S = M L + m_r L / 2, T_N = 2 pi sqrt(I_N / (g S)), with L the loaded length L0 + g L0 (M + m_r / 2) / EA. As m_r / M goes to 0, T_N goes to 2 pi sqrt(L / g). The straight-mode shape error is second order in m_r / M; the reference keeps m_r / M at 0.05 or less and cross-checks T_N against the chain's linearized eigenvalue (tridiagonal tension stiffness, lumped masses), exact for the discrete model |
| Expected convergence | T_N approaches the continuum straight-mode period with relative error m_r / (12 N^2 (M + m_r / 3)), second order in 1 / N; the case runs N = 4, 8, 16, 32 and checks the error falls by about 4 per doubling |
| ACC-037 decay closed form | The rope element damps only axially, so the undamped swing keeps its amplitude; the decay check bounds the chain's numerical decay against that constant amplitude |
| ACC-037 tolerance (T4) | Proposed: period within 0.1 % of T_N (as ACC-033 and ENG-036); amplitude within 2 % of the closed form over 20 periods (as ACC-034's proposal). Owner sets it |
| Gantry placement | One rope from the hook to the payload replaces the `hook_swing` spherical joint; the hook stays a body on the lift carriage, joined to it by a fixed joint; the payload is a free body hung on the rope at its center of mass; rope part, length and segment count are gantry data |

## Consequences
- Cost per step: each rope adds 3 (N - 1) DOF (21 for the default N = 8) and a banded block in K and D; the step stays
  one dense LLT today (the gantry grows from about 15 to about 36 DOF), with no iteration added. A band or sparse solve
  is a later optimisation behind the same step.
- Snatch peaks are resolved when the load-on-rope frequency omega_s = sqrt(EA / (L M)) times h is small: about 0.5 at
  1 ms for the gantry placeholder, where implicit Euler damps the first peak noticeably. Runs report omega_s h per rope;
  above 0.2, the snatch peak is a lower bound and a smaller physics step (run configuration) is the remedy.
- Swing damping is not modelled by the rope: a rope pendulum settles only through numerical dissipation, the payload's
  own damping or controller action. G12's settle time therefore depends on a swing damper the model does not yet have
  (open item); the gantry's `hook_friction`, `hook_damping` and `hook_backlash` parameters lose their joint when the
  spherical joint goes.
- Adapter interface: native only (the MuJoCo bridge is retired this round, round 5 ruling 1). The adapter maps `Rope` to
  the engine chain and publishes the tensions every step (PHY-004); an adapter that cannot map a rope ends the build
  (ADR 0003).
- Engine (ENG-047): a 3-DOF translation joint for nodes, a tension-only spring with the geometric stiffness term, the
  rest-length input, and attachment removal (ENG-034).
- Builder (BLD-032): emits one `Rope` element per rope with the segment count resolved; the chain is the engine's.
- Library (LIB-020): the rope part carries EA, axial damping, mass per length and breaking tension; diameter for display.
- Recorder (REC-026): records each rope's maximum and load-end tension every physics tick; the snap event joins the
  failure events (ADR 0010).
- Viewer (VIE-011): the swing overlay takes the angle of the line from the rope's `from` point to its `to` point
  against the vertical; the chain nodes are drawn from the recorded node positions.
- Tier 1 integrity (ADR 0010) may check rope tension against a safety factor on `breaking_tension`; that check is a
  plugin, not part of this ADR.

## Open items
- Multi-rope loads (slings, several ropes to one load): M4 or later; the element allows it, references and tests do not
  cover it yet.
- Rope on pulleys and wrapping, and a winch drum coupling a joint to the rest length: M4 or later.
- Swing damping source (payload air drag or a damper at the rope's top attachment, which would keep the gantry's hook
  parameters meaningful) and a damped ACC-037 decay variant: owner, before G12 settle time is scored.
- Rope mass change with length (paying out adds mass): not modelled; revisit with the winch.
- Gantry rope part, length and segment count: gantry team (placeholders in the design stage).

## Amendment 2026-11-11 (round 5 rulings 14 and 15)

- Swing damping: the rope's top end hangs from the hook through a damped spherical joint whose damping and friction are the design's `hook_damping` and `hook_friction` parameters, so G12's settle time is a design input; the rope itself damps only along its length.
- Snatch peaks are underestimated at 1 ms when a segment's natural frequency times the step exceeds 0.2; the adapter records an approximation line when a rope crosses it.
