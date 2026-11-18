#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace mechsim::engine {

struct BodyOutput;
struct JointOutput;
struct ConstraintOutput;

// A lumped member as built by add_lumped_beam (ENG-022); model.members holds one per beam (ENG-030).
struct LumpedBeam {
    Eigen::Index parent = -1;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();               // root frame origin in the parent frame, m
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  // root frame in the parent frame
    double length = 0.0;                                              // m
    double node_stiffness = 0.0;                                      // E I / l, N m/rad
    std::vector<Eigen::Index> bodies;                                 // root to tip
    std::vector<double> starts;                                       // x of each body's joint along the beam, m
    Eigen::Index roller = -1;                                         // SimplySupported: the prismatic roller body
    Eigen::Index roller_constraint = -1;  // SimplySupported: point constraint of the far end onto the roller;
                                          // Clamped: hinge constraint (point plus rotation lock about z and y)
    bool pinned_root = false;             // SimplySupported: bodies[0] is the spring-free pin segment
    // Carried joints (ENG-052, ruling 63): child bodies whose joint to the member's parent the member carries; each
    // is a child of `parent`, carried by at most one member, never a member's own body or roller.
    std::vector<Eigen::Index> carried;
};

// Spring nodes of a member, root to tip: the bodies whose joint carries E I / l (a SimplySupported pin carries none).
// Node j is the spring index of set_spring_offset and set_spring_stiffness (ENG-031, ENG-032).
[[nodiscard]] std::vector<Eigen::Index> member_nodes(const LumpedBeam& beam);

// End-loads of one member (ENG-030, PHY-007): the wrench its support exerts on the member at each end, moments about
// the end point (x = 0 and x = L), both in the member's root frame axes. The start wrench is the root joint's
// reaction (segment springs and joint constraint together) moved from the first joint to x = 0 across the clamped
// stub; the end wrench is the roller constraint's force and torque on the last segment (a pinned far end's point
// constraint carries no moment, a clamped far end's hinge rows carry the end moment), zero for a free end or a
// disabled roller. Cantilever under a tip load P (world -y, tip at x_t): start force +P y,
// start moment P x_t z. Statics, not a fit.
struct MemberOutput {
    Eigen::Vector3d start_moment = Eigen::Vector3d::Zero();  // N m
    Eigen::Vector3d start_force = Eigen::Vector3d::Zero();   // N
    Eigen::Vector3d end_moment = Eigen::Vector3d::Zero();    // N m
    Eigen::Vector3d end_force = Eigen::Vector3d::Zero();     // N
    // Largest |deflection| of a carried joint's foot point this step (ENG-052): the position error the carried
    // joint ignores by moving with the parent instead of riding the deflected member; 0 without carried joints. m
    double carried_deflection = 0.0;
    // The pair's work and the validity of the one-way result (ENG-052, brief M3-E7c; derivation under Work and
    // validity below), all 0 / false without carried joints.
    double carried_work = 0.0;               // W, the pair's net work on the member since load, J
    double carried_force = 0.0;              // largest |F_y| of the member's carried joints this step, N
    double peak_carried_force = 0.0;         // the largest carried_force since load, N
    double quasi_static_deflection = 0.0;    // largest |u_qs| of the member's carried joints this step, m
    double peak_quasi_static = 0.0;          // the largest quasi_static_deflection since load, m
    double relative_deflection = 0.0;        // the sample: max over the load nodes of abs(d_a - g_a) (Definition), m
    double static_floor = 0.0;               // the base's floor: max over joints of max over travel C(s) W, m
    double window_peak = 0.0;                // the window's peak of max over nodes abs(u_qs,a - g_a), m
    double window_quasi_static = 0.0;        // the ratio's base: max(window_peak, static_floor), m
    double window_periods = 0.0;             // N, the window in periods of the member's first loaded mode
    Eigen::Index window_steps = 0;           // the window in steps, 4 ceil(N) b (Definition)
    double dynamic_ratio = 0.0;              // relative_deflection / window_quasi_static (0 before the evaluation
                                             // start and while the base is 0)
    double peak_ratio = 0.0;                 // the largest dynamic_ratio since load
    // At the step of peak_ratio: the foot position s of the carried joint with the largest abs(d - g) at its foot, the
    // window peak and the sample (peak_ratio = peak_relative_deflection / max(peak_window_peak, static_floor)).
    double peak_position = 0.0;             // m
    double peak_window_peak = 0.0;          // m
    double peak_relative_deflection = 0.0;  // m
    bool unresolved = false;                 // dynamic_ratio has exceeded carried_ratio_limit at some step
    double unresolved_time = 0.0;            // start time of the first such step, s (0 while resolved)
};

// Work and validity (ENG-052, brief M3-E7c, ruling 69 A1).
//
// Work. In step n the pair puts w_a F y and w_a M z on the body of load node a (a = k, k + 1) at the node's point
// p_a and the exact opposite on the parent at p_a (add_carried_wrenches), at the step's start configuration q_n. The
// scheme's displacement over the step is h v_{n+1} (q_{n+1} - q_n for every joint but a quaternion's), so the pair's
// work in the step is its generalized force there times h v_{n+1}:
//   W_n = h sum_a w_a [F y . (J_a(p_a) - J_P(p_a)) + M z . (Jw_a - Jw_P)] v_{n+1},
// the node forces times the node displacements minus the same forces times the parent's displacement at the same
// points (J the point Jacobians, Jw the angular ones, at q_n); carried_work = sum over the run of W_n over the
// member's carried joints. It is Q_n . (q_{n+1} - q_n) of the energy inequality below, so it is exactly the energy
// the pair puts into the system. Its rows are the rate of the published deflection d(s) = (w_k p_k + w_{k+1} p_{k+1}
// - p) . y, so for a constant F (to first order in the member's rotations) the sum telescopes: W = F (d_end -
// d_start). A static load P that deflects the member by u from straight has done W = P u: twice the elastic energy P
// u / 2 the member stores at rest; the other half went into the transient and was dissipated (node damping, and
// backward Euler's own loss). On the source side the carried body loses none of it: it does not descend with the
// member (ruling 67's review), which is why W, not the elastic energy, is the figure to compare with the energy the
// carried system has to give.
//
// Quasi-static deflection. At load (with the stability check) the engine takes the M-normalised elastic modes phi_p,
// omega_p of K, M on the loop null space and, per member, the rows g_a = y . (J_a(p_a) - J_P(p_a)) and
// r_a = z . (Jw_a - Jw_P) of its load nodes. The static compliances on the nodes are
//   C_f(a, b) = sum_p (g_a phi_p)(g_b phi_p) / omega_p^2,   C_m(a, b) = sum_p (g_a phi_p)(r_b phi_p) / omega_p^2
// (a self-equilibrated pair has no rigid-mode component; the chain's own compliance, so the chain's spatial
// discretisation error is in both the deflection and u_qs and cancels in their ratio), and the deflection under the
// model's gravity from straight, d_g(a) = sum_p (g_a phi_p)(phi_p^T Q_g) / omega_p^2 (inertia relief included on a
// free parent; the member's own weight sags it without any carried load). Each step, with e_i the lever weights of
// joint i over the nodes (w_i on k_i, 1 - w_i on k_i + 1) and F_j, M_j the loads applied this step, joint i's
//   u_qs,i = e_i . d_g + sum_j (e_i^T C_f e_j F_j + e_i^T C_m e_j M_j),
// the deflection the member would settle to under this step's loads held constant; quasi_static_deflection is the
// largest |u_qs,i| of the member, peak_quasi_static the largest since load, window_quasi_static the base of
// Definition below. Linear at the load configuration: the
// compliance is in the member frame, so a parent's rigid motion does not change it. With the carried mass (Carried
// mass, brief M3-E7d) the modes are those of K and H + B and both sums are unchanged in value, not only in form: for
// a self-equilibrated load sum_p phi_p phi_p^T f / omega_p^2 solves K u = f up to a rigid motion, which no g_a
// sees, whatever the mass; and B phi_r = 0 on every rigid mode (G phi_r = 0), so the rigid modes, their
// normalisation and gravity's inertia relief are those of H. The validity bound below holds on the new modes as
// written: the load and the measurement are the same, only the frequencies move.
//
// Validity. A linear undamped member under a load applied no faster than suddenly deflects at most twice the
// quasi-static value: for one mode, u = u_s (1 - cos omega t). The scheme keeps the bound exactly: backward Euler on
// a step load gives e_n = u_n - u_s with (1 + x^2) e_{n+1} - 2 e_n + e_{n-1} = 0, e_0 = -u_s, e_1 = -u_s / (1 + x^2),
// whose solution is e_n = -u_s rho^n cos(n theta), rho = cos theta = (1 + x^2)^(-1/2): |e_n| < u_s, u_n < 2 u_s. A
// load at one joint, measured at the same joint, gives every mode a positive share (g phi)^2 / omega^2 of u_qs, so
// the sum of the modes' step responses is below 2 u_qs as well (a lagged load is the same load one step late). A
// deflection above twice the largest quasi-static deflection so far therefore means the carried system drives the
// member near one of its own frequencies (resonant growth, u ~ F_0 omega t / (2 k), passes 2 F_0 / k at
// omega t ~ 4): exactly where the pair's missing back-reaction would detune or absorb it, so the one-way result is
// not valid. dynamic_ratio = sample / base of Definition below; the member is unresolved from the first step
// whose ratio exceeds carried_ratio_limit = 2 (1 + carried_validity_margin). The margin, 2 percent, is the chain's
// discretisation tolerance (ENG-022): the linear chain under the scheme stays below 2 exactly, so the margin covers
// what the linear chain at the load configuration does not, the geometric nonlinearity (relative O(theta^2), below 2
// percent for member rotations below 0.14 rad) and a load moving along the member, and keeps the flag from firing on
// round-off at a ratio of exactly 2. Not covered by the bound (the rule applies, the theorem does not): a moment and
// a force at the same joint with modal shares of opposite sign, and several carried joints on one member (a joint's
// deflection under another joint's load). The flag is information: nothing is clamped and the dynamics do not change.
// Definition (briefs M3-E7f, M3-E7g, rulings 74, 76, 77; replaces the E7e base, whose window used the scheme's own
// decay and whose ratio was formed on absolute deflections, and E7f's sample at the foot, which let the member's peak
// away from a foot near a support reach many times the floor unflagged). Member-wide: with a running over the
// member's load nodes, d_a the node's deflection across the member (node_deflections), g_a its own-weight deflection
// (d_g), u_qs,a - g_a = sum_j (C_f(a, .) e_j F_j + C_m(a, .) e_j M_j) the carried loads' quasi-static deflection
// there, and W_i = m_i (gravity . y) the carried weight across the member (m_i the joint's subtree mass), each step
// from the evaluation start:
//   sample  = max_a abs(d_a - g_a)
//   base    = max(max over the window of max_a abs(u_qs,a - g_a), floor),   floor = max_i F_i, (c)
//   ratio   = sample / base,   flagged from the first step whose ratio exceeds carried_ratio_limit.
// Between nodes both deflections are the lever rule's interpolation of the two nodes (the published foot deflection
// is), so the maxima over the nodes are the maxima over the member and bound every foot point's values.
// Node by node. For a force F at joint j, mode p contributes at node a the share c_ap = (g_a phi_p)(e_j phi_p) F /
// omega_p^2 of u_qs,a, and under the scheme the step response of each mode keeps abs(e_n) < abs(c_ap) (above), so
// abs(u_a - u_qs,a) < sum_p abs(c_ap) at every node and every step. At the collocated node every share has the sign
// of F and the sum is abs(u_qs,j): u_j < 2 u_qs,j <= 2 max_b abs(u_qs,b), the factor 2 exactly. At a node whose
// shares all have one sign the same holds there: abs(u_a) < 2 abs(u_qs,a). At other nodes the shares can differ in sign and the sum of their
// magnitudes exceed abs(u_qs,a); for the maxima the bound is then max_a (abs(u_qs,a) + sum_p abs(c_ap)) over max_b
// abs(u_qs,b), which on the pinned continuum (shares sin(n pi a / L) sin(n pi s / L) / n^4) is at most 2.09 (a load
// near a support, the node near 0.43 L), reached only if every mode's phase aligns: with the continuum's own
// frequencies n^2 omega_1 the largest over time is 1.98 to 2.00 over s in [0.002, 0.5] L. So for the maxima the
// factor 2 holds within the 2 percent margin wherever the modes' phases do not align beyond the continuum's; a
// lumped chain's frequencies are not commensurate, and a step load near a support left ringing undamped for many
// periods can drift toward 2.09 and raise a conservative flag. Both maxima are of the same nodes' values, so a
// window of several loads and the floor bound the sample as they bound the collocated value.
// (a) The window is physical time. A load older than the window can legitimately leave free ringing of at most 2
// u_qs of its own (the step response above); the member's node damping takes e^(-2 pi zeta) of it per period, so
// after N = ln(1 / delta) / (2 pi zeta) periods of the first loaded mode it is below delta of its size and may be
// forgotten. The scheme's own loss pi x per period is left out: it only shortens what the window must hold (so
// leaving it out is conservative) and it would make the memory depend on the step (E7e's window ran twice as many
// periods at half the step). The window is 4 ceil(N) quarter-period buckets of b = max(1, floor(pi / (2 x))) steps
// each, b h within one step of a quarter period: the same physical time at any step to one step per bucket. An
// undamped member (zeta = 0: its ringing never decays physically) and any with N above the cap get
// carried_window_periods_max = 100 periods, a choice: past it an undamped member's old ringing counts against its
// current loads (a conservative false flag), and the cap binds for zeta < ln 50 / (200 pi) = 0.0062.
// (b) Relative to the member's static shape. The own weight is a load the member carries whether or not anything
// rides it; its static shape g is the member's equilibrium, not a carried load's effect. Subtracting g from both the
// sample and u_qs leaves u_qs,a - g_a, the carried loads' own quasi-static deflection, and the step-response bound
// above applies to it alone; it removes the old case of gravity sag opposite to the load (ratio 3). What the
// subtraction does not remove: own-weight ringing still in the member at the evaluation start (the release from
// straight on an undamped member decays only by the scheme) is counted against the carried loads.
// (c) The base has a floor (ruling 76, replacing the floor at the current foot point of ruling 74): the static
// deflection of the carried weight at the worst foot position over the carried joint's travel, F_i = max over s in
// the joint's travel of C(s) abs(W_i), computed once at load from the chain's static compliance; the member's floor is
// the largest over its joints. The travel: a prismatic joint's finite limits mapped along the member; the whole span
// for one without finite limits and for planar, free and translation joints; none (s at load only) for a joint whose
// foot cannot move along the member (revolute, fixed, spherical, or a prismatic axis across the member). W_i counts
// the carried subtree in the tree only: a load hanging on a rope or a contact reaches the member through the joint's
// reaction, which u_qs includes, but not through the floor (conservative: a lower floor only adds flags). Why: the
// floor at the current foot fell as s^2 toward a pinned support while a small ring there (a mode's shape, a moment's
// collocated term) falls as s, so the ratio grew as 1 / s with nothing material happening. Against F_i the ratio is
// the dynamic deflection's excess over the scale of the member's legitimate carried deflection, wherever the carriage
// stands. What it keeps: a member pumped anywhere on its span to more than 2.04 F is flagged. The flag time moves
// only while 2.04 F exceeds 2.04 times the windowed peak: for a force F_0 sin(omega_1 t) the resonant growth u ~ s1 c
// F_0 omega_1 t / 2 is flagged where the larger base is first passed, so the flag moves from omega t near the E7c
// bound th_1 (g(th) = 2.04) to th_F with g(th_F) = 2.04 F / (c F_0), later by (th_F - th_1) / omega_1 when F > c F_0
// and not at all otherwise. Gravity off (W = 0, F = 0), as in the E7c resonant test and two-member rig, it does not
// move. It only raises the base, so it can only remove flags.
// (d) Evaluation starts at MultibodyConfig::carried_evaluation_start (s, default 0): before it the work and the peaks
// accumulate but the window is not fed and the ratio is 0 and never flagged; the window starts empty at the first
// step whose start time is at or after it, counted in steps (not the summed time, whose round-off judged the step
// beginning exactly at the start as before): n = ceil((start - t_0) / h - 1e-9) steps after load, t_0 and h the first
// step's; a start between steps rounds up. A release from straight (the loads applied suddenly at t = 0, and the
// own weight with them) is then not judged when the start is after the settle time.
// What no flag bounds: from the evaluation start, at every step the member's largest deflection relative to its own
// static shape, at any point of the span, stayed within carried_ratio_limit = 2.04 times the larger of the windowed
// peak of the carried loads' largest nodal quasi-static deflection and the worst-case static carried deflection over
// the travel. Still flagged without resonance (conservative, not special-cased): a single load reversal at half a
// period (ratio up to 4); a release from straight judged from time 0 (evaluation start 0: the own-weight ringing
// counts against the carried loads); a step load near a support ringing long undamped (Node by node, up to 2.09).
// Not covered by the bound: a moment and a force at the same joint with modal shares of opposite sign, and several
// carried joints on one member. A resonance whose amplitude stays below 2.04 F anywhere on the span is not flagged.
inline constexpr double carried_validity_margin = 0.02;
inline constexpr double carried_ratio_limit = 2.0 * (1.0 + carried_validity_margin);
inline constexpr double carried_window_periods_max = 100.0;
// N for a mode with damping ratio zeta: ln(1 / delta) / (2 pi zeta), at most the cap (the cap for zeta <= 0).
[[nodiscard]] double carried_window_periods(double zeta);
// b, the steps per bucket: max(1, floor(pi / (2 x))).
[[nodiscard]] Eigen::Index carried_window_bucket(double x);

// poses at the configuration the reactions were computed at (the step's start state, as record_outputs uses).
[[nodiscard]] MemberOutput member_end_loads(const LumpedBeam& beam, const std::vector<BodyOutput>& poses,
                                            const std::vector<JointOutput>& joints,
                                            const std::vector<ConstraintOutput>& constraints);

// Load-path coupling (ENG-052, ruling 63; milestone 4 replaces it with the carried body riding the deflected beam).
// The joint of a carried child body C stays a joint to the member's parent P, so C moves as if the member were
// rigid; its load on the member is rerouted through the member's nodes.
//
// Load. The joint's reaction, parent on child, is [m about the child origin r_c; f], world-aligned
// (JointOutput::reaction_wrench, from the subtree's Newton-Euler at the step's accelerations, so the travel-limit
// and transmission lock rows' impulses are in it: they act along the joint's own axes, inside the subtree's motion).
// C's load on the member is its negative, F = -f, M = -m about r_c. With the member root frame (x along the member,
// ruling 24), s = (r_c - r_root) . x, foot point p = r_root + s x, d = r_c - p, the moment about p is M_p = M + d x F.
//
// Rerouted components. The lumped chain bends in one plane only, about the member frame z (rulings 16, 65): only
// the in-plane transverse force F_y = F . y and the bending moment M_z = M_p . z are rerouted. The axial force, the
// out-of-plane force, the out-of-plane bending moment and the torsion stay on P through C's own joint (the chain
// has no such freedom: out-of-plane bending is rigid, the twist and the axial slide are rigid or free at the ends,
// ruling 33). Each end condition keeps its own node: a clamped root's node at x = 0 is P itself (that share stays
// on P), a pinned root's is the pin segment's origin (straight into the pin), a roller end's node at x = L is the
// last segment's tip (straight into the roller), a free tip's is the tip itself.
//
// Lever rule. Nodes at x_0 = 0, the spring nodes (j + 1/2) l, and x_n = L. For x_k <= s <= x_{k+1}, l_k = x_{k+1} -
// x_k: w_k = (x_{k+1} - s) / l_k, w_{k+1} = (s - x_k) / l_k; w_k F_y y at node k's point (statically equivalent
// to F_y at p: w_k + w_{k+1} = 1 and w_k x_k + w_{k+1} x_{k+1} = s), w_k M_z z on node k's body. P gets the exact
// opposite at the same points, so the pair is internal: zero net force and moment, and P receives the load
// through the member's supports. A load at an end node of a supported end bends nothing.
//
// Timing. The load applied in step n is the reaction recorded by step n - 1 (computed at step n - 1's start state
// with its accelerations and row impulses). Before the first step there is none: the reaction is computed from the
// first start state by Newton-Euler at zero acceleration, so a carried body at rest puts its static weight on the
// member from the first step. The pair is explicit (lagged); it is not in K, D or the mass matrix. Its inertial
// part for the member's own deflection is: Carried mass, next.
//
// Carried mass (brief M3-E7d, ruling 70 b). A body of mass m that rode the deflected member at the foot point would
// move with the parent plus u(p) y, u(p) = w_k u_k + w_{k+1} u_{k+1} the lever-interpolated deflection the pair
// already publishes; its kinetic energy (m / 2)|v_P(p) + u'(p) y|^2 has three parts. The first, (m / 2)|v_P|^2, is
// already in the engine: C is a child of P, so C's inertia sits in P's rows of the mass matrix. The cross term m
// v_P . u' y is the parent-side half of the two-way coupling: through it the member's deflection would accelerate
// the parent, which is the position coupling ruling 63 keeps one-way (its other half, the force m a_P on the member,
// is already the pair's inertial part). The third, (m / 2) u'(p)^2, is the member-side inertia the pair lacks: the
// member is pushed by -m a_P but carries none of m's mass for its own motion, so it rang at its unloaded frequency
// (ruling 70). It is added: with G the row of the deflection rate at the foot point,
//   G v = u'(p) = sum_a w_a y . (J_a(p_a) - J_P(p_a)) v   (a = k, k + 1; the work rows g_a of Work and validity),
// the mass matrix gains B = m_c G^T G per carried joint, m_c the mass of C and its tree descendants (bodies hanging
// from C on ropes, welds, contacts or loop constraints are not in it: their loads reach C as forces, through their
// own compliance). On the two nodes' transverse coordinates B is m_c [w_k^2, w_k w_{k+1}; w_k w_{k+1}, w_{k+1}^2],
// the consistent mass of a point mass between two nodes; through the Jacobians it lands on the member's own joints.
// Three properties follow from G and are tested, not assumed: (i) no added weight: B is inertia only, the gravity
// and bias terms are unchanged, and the pair still carries m_c g once, so statics are unchanged; (ii) no added
// inertia for the parent's own motion: a rigid motion of P with the member moves every node with P, G v = 0, so B
// adds nothing to P's rows along it (on a rigid member the parent accelerates as m_p + m_c, not m_p + 2 m_c); (iii)
// symmetric and positive semi-definite, so H + B stays positive definite and (1/2) v^T B v is the added kinetic
// energy. Its force -B q'' is internal like the pair (on the nodes at their points and the opposite on the parent),
// added to the recorded reactions so the end loads include it. With the unlagged pair, a lumped node of mass mu on a
// parent fixed across the member then rings at sqrt(k / (mu + m_c)), the loaded member's frequency.
// Within a step. B is evaluated with H at the start-of-step configuration (weights w at the start-of-step s, the
// Jacobians at q_n) and held through the step like H.
// The momentum form (brief M3-E7e, ruling 73). A carriage travelling along the member changes B. Holding B and
// advancing B_n (v_{n+1} - v_n) carries the added mass's velocity across the change: for one modal coordinate u with
// modal mass m(t) = mu + m_c phi(s(t))^2 that is m u'' + k u = 0, whose adiabatic invariant makes the amplitude grow
// as m^(+1/4) as the carriage moves toward mid-span. The physics is Lagrange's equation of L = (1/2) m(t) u'^2 -
// (1/2) k u^2 with s(t) prescribed by the carriage: d/dt(m u') + k u = 0, whose action E / omega is the adiabatic
// invariant (dJ/dtheta = (m' / (2 m omega)) J cos 2 theta, J = E / omega: a relative oscillation of amplitude
// eps = abs(m') / (4 m omega) and no secular drift at first order), so the amplitude goes as m^(-1/4). The step
// therefore carries the momentum: with B'_{n-1} the block at the previous step's lever position s_{n-1} and today's
// Jacobians (B'_{n-1} = m_c G'^T G', G' the row of Carried mass at s_{n-1}),
//   (H_n + B_n) v_{n+1} = H_n v_n + B'_{n-1} v_n + h (forces at n, with D v_{n+1} and K q_{n+1} implicit),
// that is the unchanged solve with the force -(B_n - B'_{n-1}) v_n / h added: the momentum B'_{n-1} v_n the added
// mass carried out of the previous step is what it carries into this one. Only the travel is carried as momentum:
// the change of G with the member's own configuration (the nodes' Jacobians rotating with the segments) keeps the
// velocity form, like H's (its Lagrangian companion -(1/2) v^T dB/dq v is not in the bias either), so a standing
// carriage (s_n = s_{n-1} exactly: the term is skipped) steps bit-identically to the block of brief M3-E7d. The step
// before which no previous position exists (the first step after load) carries nothing. The energy: with E =
// (1/2) v^T (H + B) v + V the added mass's part changes at -(1/2) v^T (dB/dt) v, the work done on the member's
// motion by the force the prescribed travel exerts on the carriage, -(1/2) v^T (dB/ds) v along the member (it would
// load the carriage's drive; one-way, it is not applied to the carriage); per step W_n = -(1/2) v_n^T (B_n -
// B'_{n-1}) v_n, to O(h) relative. The force -(B_n - B'_{n-1}) v_n / h acts on the member through G_n^T and G'^T,
// internal like the pair, and the recorded reactions carry the momentum rate -(m_c G_n v_{n+1} - m_c G' v_n) / h
// at the two lever positions. Still dropped: the convective terms of the moving point. A body at the foot point
// moving at c along the member has transverse velocity u_t + c u_x; the momentum form keeps m_c u_t and drops m_c c
// u_x, which for a mode of wavenumber kappa is relative 2 c kappa / omega in the added mass's kinetic energy (the
// cross term) plus (c kappa / omega)^2: for the pinned first mode kappa = pi / L, so 2 pi c / (L omega): 1 m/s on a
// 4 m rail at 15 Hz is 1.7 percent, at 0.05 m/s on the 1 m test bar near 31 rad/s, 1 percent. The lever weights are
// piecewise linear in s, so a carriage crossing a node meets a kink in B (a jump in m'), not a jump in B.
// Rotary inertia. Not included: the moment channel stays without mass. For a rigid carriage at offset d from the
// axis, riding a mode sin(kappa x), the omitted part is the slope's share: relative (kappa d)^2 cot^2(kappa s) of the
// translational block at s (zero at mid-span of the pinned first mode, large near a support where the deflection
// vanishes and the slope does not). Bodies below C with their own compliance are counted rigid in m_c. The relative rotation at the foot point is
// the member's slope theta(p), and C's share of it would be the rotary inertia of the carried subtree about the
// foot point, I_zz + m_c |d|^2. That is not a rigid property of a carried system with its own joints (a hook on a
// rope does not follow the rail's slope; its m |d|^2 about the rail would be a large, wrong inertia), and for a
// compact carriage its ratio to the translational part in a bending mode is (k_m rho)^2, rho the radius of
// gyration, k_m the mode's wavenumber (pi / L at midspan of a pinned span): under 1 percent for rho below L / 30.
// The moment channel's lag (ruling 68) is therefore unchanged and still not checked.
//
// Stability (brief M3-E7b). C's inertia sits in P's row of the mass matrix (implicit); the pair takes m_c a_{n-1}
// (a the carried subtree's acceleration across the member, m_c its mass) off P and puts it on the nodes one step
// late. On a parent fixed in the bending plane a = 0: the pair is feed-forward (gravity, applied loads) and always
// stable. On a parent that can move there the lag feeds back through the member's bending modes. With the
// coordinates q, mass matrix M (C included), J_c the subtree's centre-of-mass Jacobian along y, J_P the parent's
// and J_n the lever-weighted nodes' point Jacobians along y at the load, the pair's generalized force is
// m_c (J_P - J_n)^T J_c q''_{n-1}. Project on an M-normalised bending mode phi (K phi = omega^2 M phi, on the null
// space of the loop constraints), dropping the lag's coupling between modes: with u the modal coordinate
//   u''_n - r u''_{n-1} + 2 zeta omega u'_{n+1} + omega^2 u_{n+1} = 0,   r = m_c (J_c phi) ((J_P - J_n) phi),
// zeta = phi^T D phi / (2 omega). For one lumped node of mass mu on a parent of mass m_p this is the review's
// alpha u''_n - beta u''_{n-1} + k u = 0 with r = beta / alpha = m_c / (m_p + m_c); a distributed mode adds the
// shape factor phi(s) sum m phi / sum m phi^2 (4 / pi for the pinned-pinned first mode at midspan).
// The engine's scheme (backward Euler in K and D: v_{n+1} = v_n + h u''_n, u_{n+1} = u_n + h v_{n+1}) with
// u_n = z^n gives, x = omega h,
//   (1 + 2 zeta x + x^2) z^3 - (2 + r + 2 zeta x) z^2 + (1 + 2 r) z - r = 0
// (r = 0: z = 0 and backward Euler's 1 / (1 -+ i x)). The growth factor is the largest |z| (lag_growth_factor).
// Undamped, z = e^(i theta) on the unit circle needs cos theta = 1 / (1 + x^2) and then r = (1 + x^2) / 2: the
// mode grows exactly when r > (1 + x^2) / 2 (lag_stability_boundary), to leading order
//   |z| = 1 + x^2 (2 r - 1) / (2 (1 - r)^2),
// a negative damping above r = 1/2 that backward Euler's x^2 / 2 offsets only at a coarse step (the h -> 0 limit
// of the boundary is r = 1/2: m_c > m_p for the lumped node). At load (the first step's start state) the engine
// computes, per carried joint, the inverse inertia J_P M^-1 J_P^T on the constrained space (zero: P fixed in the
// plane, no check) and r, x of every coupled mode; the decision does not drop the coupling between modes (measured:
// the diagonal r of the pinned-pinned free-parent rig is about 0.7 % low near the boundary, enough to turn a decay
// of 1e-4 per step into one of 4e-4): it takes the spectral radius of the whole linear step on every elastic mode
// with the full rank-one-per-joint lag (lag_spectral_radius; one mode: exactly the cubic above). It refuses the
// model (InvalidModelError naming the member, the joint closest to its boundary and its r) when the undamped radius
// exceeds 1 (ruling 67: refuse, never clamp), and otherwise publishes per joint the mode closest to its boundary
// (CarriedLoadOutput::lag_ratio, lag_omega_h) and the damped radius (lag_factor). The check is linear at the load
// configuration (a carriage moving along the member changes r with phi(s)); the moment channel (C's rotary inertia
// about z on a rotating parent) is not checked.
// With the carried mass (brief M3-E7d). The equation gains B (Carried mass): with G = J_n - J_P along y,
//   (H + B) q''_n + D v_{n+1} + K q_{n+1} = -m_c G^T J_c q''_{n-1}   (+ the feed-forward loads),
// the same recursion on the modes of K and H + B (phi normalised in H + B), r = -m_c (J_c phi)(G phi) on them: the
// cubic and the boundary r > (1 + x^2) / 2 hold unchanged in form. The mass keeps r below it. The carried subtree's
// kinetic energy in phi is at least m_c (J_c phi)^2 / 2 (its mass at its centre of mass), so with
// a = sqrt(m_c) |J_c phi| and b = sqrt(m_c) |G phi|, 1 = phi^T (H + B) phi >= a^2 + b^2 and
//   |r| <= a b <= (a^2 + b^2) / 2 <= 1/2 < (1 + x^2) / 2   for every x > 0:
// a single mode of the force channel is stable for any carried mass on any parent. The lumped node: r = m_c mu /
// ((m_p + m_c)(mu + m_c) + mu m_c), below 1/2 since m_p + m_c > m_c (E7b's r = m_c / (m_p + m_c) crossed 1/2 at
// m_c = m_p). Between modes the lag matrix B_pq = m_c (G phi_p)(J_c phi_q) is rank one per joint with norm below 1
// by the same two bounds over all modes, which does not by itself bound the coupled step: the engine keeps the full
// check (lag_spectral_radius on the modes of K and H + B) and the refusal as a guard, published per joint as before.
// A parent fixed in the plane is unchanged (J_P = 0, feed-forward). The cost on a free parent: the lumped mode is
// omega^2 = k (m_p + m_c + mu) / ((mu + m_c)(m_p + m_c)) against full coupling's k (m_p + m_c + mu) / ((mu + m_c)
// m_p): without the cross term m_c is counted on both sides, omega^2 low by the factor 1 - m_c / (m_p + m_c).
// Full (implicit) coupling is milestone 4. For a constant acceleration the reaction is constant after the first
// step and the lag error is zero; a step change da in acceleration leaves the rerouted load off by m_c da for one
// step only (m_c j h under a constant jerk j).
// Energy. The pair does work only through the members' deflection (its two halves act at the same points), so on
// a rigid configuration it does none; with a constant load it is the potential -F_y u(p) - M_z theta(p) of the
// deflection under the load, and the scheme's energy E (kinetic, gravitational, elastic) satisfies
//   E(n + 1) - E(n) <= Q_n . (q_{n+1} - q_n),   Q_n the pair's generalized force (carried_generalized_forces),
// the backward Euler inequality of the energy oracle with the pair as an explicit applied force.
// The approximation: C ignores the member's deflection u(p) at its foot point, published per carried joint as
// CarriedLoadOutput::deflection and per member as MemberOutput::carried_deflection.
struct CarriedLoadOutput {
    Eigen::Index member = -1;
    Eigen::Index body = -1;      // the carried child body
    double position = 0.0;       // s along the member, m
    double force = 0.0;          // F_y on the member, member frame y, N
    double moment = 0.0;         // M_z on the member about the foot point, member frame z, N m
    Eigen::Index node = 0;       // k: the load is on nodes k and k + 1 (0 = the root node, x = 0)
    double weight = 1.0;         // w_k; node k + 1 takes 1 - w_k
    double deflection = 0.0;     // (w_k p_k + w_{k+1} p_{k+1} - p) . y, m
    // The lag's stability at load (Stability above); constant over the run, 0 on a parent fixed in the plane.
    double inverse_inertia = 0.0;  // J_P M^-1 J_P^T at the load, 1/kg
    double lag_ratio = 0.0;        // r of the coupled mode closest to its boundary
    double lag_omega_h = 0.0;      // x = omega h of that mode
    double lag_factor = 0.0;       // per-step |z| of the coupled modes together, damped (< 1 decays)
};

// Largest |z| of (1 + 2 zeta x + x^2) z^3 - (2 + r + 2 zeta x) z^2 + (1 + 2 r) z - r = 0 (Stability above).
[[nodiscard]] double lag_growth_factor(double r, double x, double zeta);
// The undamped boundary: a mode grows exactly when r > (1 + x^2) / 2.
[[nodiscard]] double lag_stability_boundary(double x);
// The coupled modes together, without dropping the lag's coupling between them: the largest |z| of the linear step
// on (u_n, v_n, a_{n-1}) for M-normalised modes, lag the modal B (B_pq = sum over carried joints of
// m_c ((J_P - J_n) phi_p) (J_c phi_q)), omega_squared the modes' K, damping the modal D. One mode: lag_growth_factor.
[[nodiscard]] double lag_spectral_radius(const Eigen::MatrixXd& lag, const Eigen::VectorXd& omega_squared,
                                         const Eigen::MatrixXd& damping, double h);

// The member's load nodes, root to tip: x along the member, the body holding the point (-1 the world) and the point
// in that body's frame.
struct LoadNode {
    double x = 0.0;
    Eigen::Index body = -1;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
};
[[nodiscard]] std::vector<LoadNode> load_nodes(const LumpedBeam& beam);

// The deflection of each load node across the member, root to tip: (p_a - (r_root + x_a x)) . y, m (the published
// foot deflection at s = x_a; the member-wide sample of Definition).
[[nodiscard]] std::vector<double> node_deflections(const LumpedBeam& beam, const std::vector<Eigen::Matrix3d>& rotation,
                                                   const std::vector<Eigen::Vector3d>& origin);

// The pair for one carried body: world points and axes from body_from_world (per body, body_X_world: rotation world
// to body, translation the body origin in the world), load the world-aligned [M about r_c; F] on the member.
struct CarriedPair {
    CarriedLoadOutput output;
    Eigen::Index body_lower = -1;
    Eigen::Index body_upper = -1;
    Eigen::Vector3d point_lower = Eigen::Vector3d::Zero();  // world, m
    Eigen::Vector3d point_upper = Eigen::Vector3d::Zero();
    Eigen::Vector3d axis_y = Eigen::Vector3d::UnitY();       // member frame y, world
    Eigen::Vector3d axis_z = Eigen::Vector3d::UnitZ();       // member frame z, world
    double excursion = 0.0;  // distance of s outside [0, L], m (0 on the span)
};
// The lever rule and member frame at s (clamped to [0, L]) without a load: the nodes and weight of a carried joint
// at a given position (the momentum form's previous position).
[[nodiscard]] CarriedPair carried_pair_at(const LumpedBeam& beam, double s, const std::vector<Eigen::Matrix3d>& rotation,
                                          const std::vector<Eigen::Vector3d>& origin);
[[nodiscard]] CarriedPair carried_pair(const LumpedBeam& beam, Eigen::Index carried,
                                       const std::vector<Eigen::Matrix3d>& rotation,
                                       const std::vector<Eigen::Vector3d>& origin,
                                       const Eigen::Matrix<double, 6, 1>& load);

}  // namespace mechsim::engine
