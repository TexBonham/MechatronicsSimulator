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
    double peak_quasi_static = 0.0;          // the largest quasi_static_deflection since load (the ratio's base), m
    double dynamic_ratio = 0.0;              // carried_deflection / peak_quasi_static (0 while that is 0)
    double peak_ratio = 0.0;                 // the largest dynamic_ratio since load
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
// largest |u_qs,i| of the member, peak_quasi_static the largest since load. Linear at the load configuration: the
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
// not valid. dynamic_ratio = carried_deflection / peak_quasi_static; the member is unresolved from the first step
// whose ratio exceeds carried_ratio_limit = 2 (1 + carried_validity_margin). The margin, 2 percent, is the chain's
// discretisation tolerance (ENG-022): the linear chain under the scheme stays below 2 exactly, so the margin covers
// what the linear chain at the load configuration does not, the geometric nonlinearity (relative O(theta^2), below 2
// percent for member rotations below 0.14 rad) and a load moving along the member, and keeps the flag from firing on
// round-off at a ratio of exactly 2. Not covered by the bound (the rule applies, the theorem does not): a moment and
// a force at the same joint with modal shares of opposite sign, and several carried joints on one member (a joint's
// deflection under another joint's load). The flag is information: nothing is clamped and the dynamics do not change.
inline constexpr double carried_validity_margin = 0.02;
inline constexpr double carried_ratio_limit = 2.0 * (1.0 + carried_validity_margin);

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
// Jacobians at q_n) and held through the step like H. What freezing costs: B's dependence on q has no Coriolis or
// centripetal companion in the bias (those are the moving-mass terms 2 c u'_x and c^2 u_xx of a mass crossing a beam
// at speed c, and the slope terms of G's rotation with the member), so the energy (1/2) v^T B v is not conserved
// to O(dB/dt) per unit time: from the moving weights about m_c u'(p)^2 c / l (zero for a standing carriage), and from
// the member's own rotation a relative O(theta theta' h) per step. The energy oracle (standing carriages, no node
// damping) measures the second against the scheme's own loss. The lever weights are piecewise linear in s, so a
// carriage crossing a node meets a kink in B, not a jump.
// Rotary inertia. Not included: the moment channel stays without mass. The relative rotation at the foot point is
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
[[nodiscard]] CarriedPair carried_pair(const LumpedBeam& beam, Eigen::Index carried,
                                       const std::vector<Eigen::Matrix3d>& rotation,
                                       const std::vector<Eigen::Vector3d>& origin,
                                       const Eigen::Matrix<double, 6, 1>& load);

}  // namespace mechsim::engine
