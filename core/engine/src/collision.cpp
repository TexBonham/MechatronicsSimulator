#include "mechsim/engine/collision.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "mechsim/engine/engine_error.hpp"

namespace mechsim::engine {

// ===== Shapes ============================================================

namespace {

constexpr double degenerate_scale = 1e-10;  // relative to the Minkowski difference's extent

struct SupportPoint {
    Eigen::Vector3d w = Eigen::Vector3d::Zero();  // a - b
    Eigen::Vector3d a = Eigen::Vector3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
};

double sign_of(double x) {
    if (x < 0.0) {
        return -1.0;
    }
    return 1.0;
}

Eigen::Vector3d local_support(const Shape& shape, const Eigen::Vector3d& d) {
    switch (shape.kind) {
        case ShapeKind::Sphere: {
            const double n = d.norm();
            if (n == 0.0) {
                return Eigen::Vector3d::Zero();
            }
            return shape.radius * d / n;
        }
        case ShapeKind::Capsule: {
            Eigen::Vector3d out(0.0, 0.0, sign_of(d.z()) * shape.half_length);
            const double n = d.norm();
            if (n > 0.0) {
                out += shape.radius * d / n;
            }
            return out;
        }
        case ShapeKind::Box: {
            return {sign_of(d.x()) * shape.half_extents.x(), sign_of(d.y()) * shape.half_extents.y(),
                    sign_of(d.z()) * shape.half_extents.z()};
        }
        case ShapeKind::Cylinder: {
            Eigen::Vector3d out(0.0, 0.0, sign_of(d.z()) * shape.half_length);
            const double radial = std::hypot(d.x(), d.y());
            if (radial > 0.0) {
                out.x() = shape.radius * d.x() / radial;
                out.y() = shape.radius * d.y() / radial;
            }
            return out;
        }
        case ShapeKind::ConvexHull: {
            std::size_t best = 0;
            double best_dot = shape.vertices[0].dot(d);
            for (std::size_t i = 1; i < shape.vertices.size(); ++i) {
                const double dot = shape.vertices[i].dot(d);
                if (dot > best_dot) {
                    best_dot = dot;
                    best = i;
                }
            }
            return shape.vertices[best];
        }
        case ShapeKind::Plane: {
            break;
        }
    }
    throw InvalidModelError(0.0, "a plane has no support point");
}

bool all_positive(const Eigen::Vector3d& a) { return a.allFinite() && a.minCoeff() > 0.0; }

bool positive(double x) { return std::isfinite(x) && x > 0.0; }

}  // namespace

ShapeKind shape_kind(const std::string& name) {
    const std::array<std::pair<const char*, ShapeKind>, 6> names{{{"sphere", ShapeKind::Sphere},
                                                                  {"capsule", ShapeKind::Capsule},
                                                                  {"box", ShapeKind::Box},
                                                                  {"cylinder", ShapeKind::Cylinder},
                                                                  {"plane", ShapeKind::Plane},
                                                                  {"convex_hull", ShapeKind::ConvexHull}}};
    for (const auto& [text, kind] : names) {
        if (name == text) {
            return kind;
        }
    }
    throw InvalidModelError(0.0, "contact geometry kind '" + name + "' is neither a primitive nor a convex hull");
}

const char* shape_name(ShapeKind kind) {
    switch (kind) {
        case ShapeKind::Sphere:
            return "sphere";
        case ShapeKind::Capsule:
            return "capsule";
        case ShapeKind::Box:
            return "box";
        case ShapeKind::Cylinder:
            return "cylinder";
        case ShapeKind::Plane:
            return "plane";
        case ShapeKind::ConvexHull:
            return "convex_hull";
    }
    return "unknown";
}

void validate_shape(const Shape& shape, const std::string& label) {
    switch (shape.kind) {
        case ShapeKind::Sphere: {
            if (!positive(shape.radius)) {
                throw InvalidModelError(0.0, label + "sphere radius must be finite and > 0");
            }
            return;
        }
        case ShapeKind::Capsule:
        case ShapeKind::Cylinder: {
            if (!positive(shape.radius) || !positive(shape.half_length)) {
                throw InvalidModelError(0.0, label + shape_name(shape.kind) +
                                                 " radius and half length must be finite and > 0");
            }
            return;
        }
        case ShapeKind::Box: {
            if (!all_positive(shape.half_extents)) {
                throw InvalidModelError(0.0, label + "box half extents must be finite and > 0");
            }
            return;
        }
        case ShapeKind::ConvexHull: {
            if (shape.vertices.size() < 4) {
                throw InvalidModelError(0.0, label + "convex hull needs at least four vertices");
            }
            double extent = 0.0;
            for (const Eigen::Vector3d& v : shape.vertices) {
                if (!v.allFinite()) {
                    throw InvalidModelError(0.0, label + "convex hull vertex is not finite");
                }
                extent = std::max(extent, (v - shape.vertices[0]).norm());
            }
            const Eigen::Vector3d& o = shape.vertices[0];
            const double volume_floor = degenerate_scale * extent * extent * extent;
            for (std::size_t i = 1; i < shape.vertices.size(); ++i) {
                for (std::size_t j = i + 1; j < shape.vertices.size(); ++j) {
                    const Eigen::Vector3d n = (shape.vertices[i] - o).cross(shape.vertices[j] - o);
                    for (std::size_t k = j + 1; k < shape.vertices.size(); ++k) {
                        if (std::abs(n.dot(shape.vertices[k] - o)) > volume_floor) {
                            return;
                        }
                    }
                }
            }
            throw InvalidModelError(0.0, label + "convex hull vertices are coplanar");
        }
        case ShapeKind::Plane: {
            return;
        }
    }
}

Eigen::Vector3d support(const Shape& shape, const Pose& pose, const Eigen::Vector3d& direction) {
    return pose.position + pose.rotation * local_support(shape, pose.rotation.transpose() * direction);
}

// ===== Broad phase =======================================================

Aabb bounds(const Shape& shape, const Pose& pose) {
    Aabb box;
    Eigen::Vector3d extent = Eigen::Vector3d::Zero();
    const Eigen::Vector3d axis = pose.rotation.col(2);
    switch (shape.kind) {
        case ShapeKind::Sphere: {
            extent.setConstant(shape.radius);
            break;
        }
        case ShapeKind::Capsule: {
            extent = shape.half_length * axis.cwiseAbs() + Eigen::Vector3d::Constant(shape.radius);
            break;
        }
        case ShapeKind::Box: {
            extent = pose.rotation.cwiseAbs() * shape.half_extents;
            break;
        }
        case ShapeKind::Cylinder: {
            for (Eigen::Index i = 0; i < 3; ++i) {
                extent[i] = shape.half_length * std::abs(axis[i]) +
                            shape.radius * std::sqrt(std::max(0.0, 1.0 - axis[i] * axis[i]));
            }
            break;
        }
        case ShapeKind::ConvexHull: {
            box.min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
            box.max = -box.min;
            for (const Eigen::Vector3d& v : shape.vertices) {
                const Eigen::Vector3d w = pose.position + pose.rotation * v;
                box.min = box.min.cwiseMin(w);
                box.max = box.max.cwiseMax(w);
            }
            return box;
        }
        case ShapeKind::Plane: {
            box.max = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
            box.min = -box.max;
            return box;
        }
    }
    box.min = pose.position - extent;
    box.max = pose.position + extent;
    return box;
}

bool overlaps(const Aabb& a, const Aabb& b) {
    return (a.min.array() <= b.max.array()).all() && (b.min.array() <= a.max.array()).all();
}

std::vector<std::pair<std::size_t, std::size_t>> broad_phase(const std::vector<Aabb>& boxes) {
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        for (std::size_t j = i + 1; j < boxes.size(); ++j) {
            if (overlaps(boxes[i], boxes[j])) {
                pairs.emplace_back(i, j);
            }
        }
    }
    return pairs;
}

// ===== Closed forms ======================================================

namespace {

std::vector<ContactPoint> sphere_sphere(const Eigen::Vector3d& ca, double ra, const Eigen::Vector3d& cb, double rb) {
    const Eigen::Vector3d d = cb - ca;
    const double dist = d.norm();
    const double depth = ra + rb - dist;
    if (!(depth > 0.0)) {
        return {};
    }
    Eigen::Vector3d n = Eigen::Vector3d::UnitZ();
    if (dist > 0.0) {
        n = d / dist;
    }
    ContactPoint c;
    c.normal = n;
    c.depth = depth;
    c.point = 0.5 * ((ca + ra * n) + (cb - rb * n));
    return {c};
}

Eigen::Vector3d closest_on_segment(const Eigen::Vector3d& p, const Eigen::Vector3d& s0, const Eigen::Vector3d& s1) {
    const Eigen::Vector3d d = s1 - s0;
    const double t = std::clamp((p - s0).dot(d) / d.squaredNorm(), 0.0, 1.0);
    return s0 + t * d;
}

// Closest points of segments p1-q1 and p2-q2 (Ericson, Real-Time Collision Detection 5.1.9); both have length > 0.
std::pair<Eigen::Vector3d, Eigen::Vector3d> closest_segments(const Eigen::Vector3d& p1, const Eigen::Vector3d& q1,
                                                             const Eigen::Vector3d& p2, const Eigen::Vector3d& q2) {
    const Eigen::Vector3d d1 = q1 - p1;
    const Eigen::Vector3d d2 = q2 - p2;
    const Eigen::Vector3d r = p1 - p2;
    const double a = d1.squaredNorm();
    const double e = d2.squaredNorm();
    const double f = d2.dot(r);
    const double c = d1.dot(r);
    const double b = d1.dot(d2);
    const double denom = a * e - b * b;
    double s = 0.0;
    if (denom > std::numeric_limits<double>::epsilon() * a * e) {
        s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
    }
    double t = (b * s + f) / e;
    if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
    } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
    }
    return {p1 + s * d1, p2 + t * d2};
}

std::array<Eigen::Vector3d, 2> segment(const Shape& capsule, const Pose& pose) {
    const Eigen::Vector3d half = capsule.half_length * pose.rotation.col(2);
    return {pose.position - half, pose.position + half};
}

std::vector<ContactPoint> sphere_box(const Shape& sphere, const Pose& ps, const Shape& box, const Pose& pb) {
    const Eigen::Vector3d c = pb.rotation.transpose() * (ps.position - pb.position);
    const Eigen::Vector3d& e = box.half_extents;
    const Eigen::Vector3d q = c.cwiseMax(-e).cwiseMin(e);
    ContactPoint out;
    if (q != c) {
        const Eigen::Vector3d d = q - c;
        const double dist = d.norm();
        out.depth = sphere.radius - dist;
        if (!(out.depth > 0.0)) {
            return {};
        }
        out.normal = pb.rotation * d / dist;
        const Eigen::Vector3d witness_b = pb.position + pb.rotation * q;
        out.point = 0.5 * ((ps.position + sphere.radius * out.normal) + witness_b);
        return {out};
    }
    Eigen::Index axis = 0;
    double gap = e[0] - std::abs(c[0]);
    for (Eigen::Index i = 1; i < 3; ++i) {
        const double g = e[i] - std::abs(c[i]);
        if (g < gap) {
            gap = g;
            axis = i;
        }
    }
    const double s = sign_of(c[axis]);
    out.depth = sphere.radius + gap;
    out.normal = -s * pb.rotation.col(axis);
    Eigen::Vector3d face = c;
    face[axis] = s * e[axis];
    const Eigen::Vector3d witness_b = pb.position + pb.rotation * face;
    out.point = 0.5 * ((ps.position + sphere.radius * out.normal) + witness_b);
    return {out};
}

// Points of the shape that can reach deepest into a half-space with outward normal n (world).
std::vector<Eigen::Vector3d> plane_features(const Shape& shape, const Pose& pose, const Eigen::Vector3d& n) {
    switch (shape.kind) {
        case ShapeKind::Sphere: {
            return {pose.position - shape.radius * n};
        }
        case ShapeKind::Capsule: {
            const std::array<Eigen::Vector3d, 2> s = segment(shape, pose);
            return {s[0] - shape.radius * n, s[1] - shape.radius * n};
        }
        case ShapeKind::Box: {
            std::vector<Eigen::Vector3d> corners;
            for (int i = 0; i < 8; ++i) {
                Eigen::Vector3d local = shape.half_extents;
                if ((i & 1) != 0) {
                    local.x() = -local.x();
                }
                if ((i & 2) != 0) {
                    local.y() = -local.y();
                }
                if ((i & 4) != 0) {
                    local.z() = -local.z();
                }
                corners.emplace_back(pose.position + pose.rotation * local);
            }
            return corners;
        }
        case ShapeKind::ConvexHull: {
            std::vector<Eigen::Vector3d> points;
            points.reserve(shape.vertices.size());
            for (const Eigen::Vector3d& v : shape.vertices) {
                points.emplace_back(pose.position + pose.rotation * v);
            }
            return points;
        }
        case ShapeKind::Cylinder: {
            const Eigen::Vector3d axis = pose.rotation.col(2);
            Eigen::Vector3d u = -(n - n.dot(axis) * axis);
            if (u.norm() <= degenerate_scale) {
                u = pose.rotation.col(0);
            }
            u.normalize();
            const Eigen::Vector3d w = axis.cross(u);
            std::vector<Eigen::Vector3d> points;
            for (const double side : {-1.0, 1.0}) {
                const Eigen::Vector3d cap = pose.position + side * shape.half_length * axis;
                points.emplace_back(cap + shape.radius * u);
                points.emplace_back(cap + shape.radius * w);
                points.emplace_back(cap - shape.radius * u);
                points.emplace_back(cap - shape.radius * w);
            }
            return points;
        }
        case ShapeKind::Plane: {
            break;
        }
    }
    return {};
}

std::vector<ContactPoint> shape_plane(const Shape& shape, const Pose& pose, const Pose& plane) {
    const Eigen::Vector3d n = plane.rotation.col(2);
    std::vector<ContactPoint> out;
    for (const Eigen::Vector3d& x : plane_features(shape, pose, n)) {
        const double depth = -n.dot(x - plane.position);
        if (depth > 0.0) {
            ContactPoint c;
            c.depth = depth;
            c.normal = -n;
            c.point = x + 0.5 * depth * n;
            out.push_back(c);
        }
    }
    return out;
}

// ===== GJK ===============================================================

struct Pair {
    const Shape& a;
    const Pose& pa;
    const Shape& b;
    const Pose& pb;
    const CollisionLimits& limits;

    [[nodiscard]] SupportPoint operator()(const Eigen::Vector3d& d) const {
        SupportPoint p;
        p.a = support(a, pa, d);
        p.b = support(b, pb, -d);
        p.w = p.a - p.b;
        return p;
    }
};

Eigen::Vector3d triple(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c) {
    return a.cross(b).cross(c);
}

// Simplex vectors keep the newest point last. Each handler reduces to the feature nearest the origin and sets the
// next search direction; a zero direction means the origin lies on the simplex.
void line_case(std::vector<SupportPoint>& s, Eigen::Vector3d& d) {
    const SupportPoint a = s[1];
    const SupportPoint b = s[0];
    const Eigen::Vector3d ab = b.w - a.w;
    const Eigen::Vector3d ao = -a.w;
    if (ab.dot(ao) > 0.0) {
        s = {b, a};
        d = triple(ab, ao, ab);
        return;
    }
    s = {a};
    d = ao;
}

void triangle_case(std::vector<SupportPoint>& s, Eigen::Vector3d& d) {
    const SupportPoint a = s[2];
    const SupportPoint b = s[1];
    const SupportPoint c = s[0];
    const Eigen::Vector3d ab = b.w - a.w;
    const Eigen::Vector3d ac = c.w - a.w;
    const Eigen::Vector3d ao = -a.w;
    const Eigen::Vector3d abc = ab.cross(ac);
    if (abc.cross(ac).dot(ao) > 0.0) {
        if (ac.dot(ao) > 0.0) {
            s = {c, a};
            d = triple(ac, ao, ac);
            return;
        }
        s = {b, a};
        line_case(s, d);
        return;
    }
    if (ab.cross(abc).dot(ao) > 0.0) {
        s = {b, a};
        line_case(s, d);
        return;
    }
    const double side = abc.dot(ao);
    if (side > 0.0) {
        s = {c, b, a};
        d = abc;
        return;
    }
    if (side < 0.0) {
        s = {b, c, a};
        d = -abc;
        return;
    }
    d = Eigen::Vector3d::Zero();
}

// Returns true when the origin is inside the tetrahedron (or on its boundary).
bool tetrahedron_case(std::vector<SupportPoint>& s, Eigen::Vector3d& d) {
    const SupportPoint a = s[3];
    const std::array<std::array<SupportPoint, 3>, 3> faces{{{s[2], s[1], s[0]}, {s[1], s[0], s[2]}, {s[0], s[2], s[1]}}};
    const Eigen::Vector3d ao = -a.w;
    for (const auto& [x, y, z] : faces) {
        Eigen::Vector3d n = (x.w - a.w).cross(y.w - a.w);
        if (n.dot(z.w - a.w) > 0.0) {
            n = -n;
        }
        if (n.dot(ao) > 0.0) {
            s = {x, y, a};
            triangle_case(s, d);
            return false;
        }
    }
    return true;
}

bool gjk(const Pair& pair, std::vector<SupportPoint>& s) {
    Eigen::Vector3d d = pair.pb.position - pair.pa.position;
    if (d.squaredNorm() == 0.0) {
        d = Eigen::Vector3d::UnitX();
    }
    s = {pair(d)};
    d = -s[0].w;
    for (int iteration = 0; iteration < pair.limits.gjk_iterations; ++iteration) {
        if (d.squaredNorm() == 0.0) {
            return true;
        }
        const SupportPoint p = pair(d);
        if (p.w.dot(d) <= pair.limits.tolerance * d.norm()) {
            return false;
        }
        s.push_back(p);
        if (s.size() == 2) {
            line_case(s, d);
        } else if (s.size() == 3) {
            triangle_case(s, d);
        } else if (tetrahedron_case(s, d)) {
            return true;
        }
    }
    throw SolverNonConvergenceError(0.0, std::string("GJK did not converge within ") +
                                             std::to_string(pair.limits.gjk_iterations) + " iterations on " +
                                             shape_name(pair.a.kind) + "-" + shape_name(pair.b.kind));
}

// Grows a simplex that contains the origin on its boundary to a tetrahedron with volume; false if the Minkowski
// difference is itself flat at this scale.
bool complete_simplex(const Pair& pair, std::vector<SupportPoint>& s, double eps) {
    const std::array<Eigen::Vector3d, 6> axes{Eigen::Vector3d::UnitX(),  Eigen::Vector3d::UnitY(),
                                              Eigen::Vector3d::UnitZ(),  -Eigen::Vector3d::UnitX(),
                                              -Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitZ()};
    if (s.size() == 1) {
        for (const Eigen::Vector3d& axis : axes) {
            const SupportPoint p = pair(axis);
            if ((p.w - s[0].w).norm() > eps) {
                s.push_back(p);
                break;
            }
        }
    }
    if (s.size() == 2) {
        const Eigen::Vector3d u = s[1].w - s[0].w;
        Eigen::Index smallest = 0;
        u.cwiseAbs().minCoeff(&smallest);
        const Eigen::Vector3d p1 = u.cross(Eigen::Vector3d::Unit(smallest)).normalized();
        const Eigen::Vector3d p2 = u.cross(p1).normalized();
        for (const Eigen::Vector3d& dir : {p1, Eigen::Vector3d(-p1), p2, Eigen::Vector3d(-p2)}) {
            const SupportPoint p = pair(dir);
            if (u.cross(p.w - s[0].w).norm() > eps * u.norm()) {
                s.push_back(p);
                break;
            }
        }
    }
    if (s.size() == 3) {
        const Eigen::Vector3d n = (s[1].w - s[0].w).cross(s[2].w - s[0].w);
        const double area = n.norm();
        if (area > 0.0) {
            for (const Eigen::Vector3d& dir : {n, Eigen::Vector3d(-n)}) {
                const SupportPoint p = pair(dir);
                if (std::abs(n.dot(p.w - s[0].w)) > eps * area) {
                    s.push_back(p);
                    break;
                }
            }
        }
    }
    return s.size() == 4;
}

// ===== EPA ===============================================================

struct Face {
    std::array<std::size_t, 3> v{};
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    double dist = 0.0;
};

bool make_face(const std::vector<SupportPoint>& pts, std::size_t i, std::size_t j, std::size_t k, Face& face) {
    const Eigen::Vector3d n = (pts[j].w - pts[i].w).cross(pts[k].w - pts[i].w);
    const double length = n.norm();
    if (!(length > 0.0)) {
        return false;
    }
    face.v = {i, j, k};
    face.n = n / length;
    face.dist = face.n.dot(pts[i].w);
    return true;
}

std::vector<ContactPoint> epa(const Pair& pair, std::vector<SupportPoint> pts) {
    const Eigen::Vector3d centroid = 0.25 * (pts[0].w + pts[1].w + pts[2].w + pts[3].w);
    std::vector<Face> faces;
    const std::array<std::array<std::size_t, 3>, 4> initial{{{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}}};
    for (const auto& [i, j, k] : initial) {
        Face face;
        if (!make_face(pts, i, j, k, face)) {
            throw SolverNonConvergenceError(0.0, std::string("EPA met a degenerate initial tetrahedron on ") +
                                                     shape_name(pair.a.kind) + "-" + shape_name(pair.b.kind));
        }
        if (face.n.dot(pts[i].w - centroid) < 0.0) {
            static_cast<void>(make_face(pts, i, k, j, face));
        }
        faces.push_back(face);
    }
    std::size_t best = 0;
    bool converged = false;
    for (int iteration = 0; iteration < pair.limits.epa_iterations; ++iteration) {
        best = 0;
        for (std::size_t f = 1; f < faces.size(); ++f) {
            if (faces[f].dist < faces[best].dist) {
                best = f;
            }
        }
        const Face closest = faces[best];
        const SupportPoint p = pair(closest.n);
        if (p.w.dot(closest.n) - closest.dist <= pair.limits.tolerance) {
            converged = true;
            break;
        }
        pts.push_back(p);
        const std::size_t index = pts.size() - 1;
        std::vector<std::pair<std::size_t, std::size_t>> horizon;
        std::vector<Face> kept;
        for (const Face& face : faces) {
            if (face.n.dot(p.w - pts[face.v[0]].w) > pair.limits.tolerance) {
                for (int e = 0; e < 3; ++e) {
                    const std::pair<std::size_t, std::size_t> edge{face.v[static_cast<std::size_t>(e)],
                                                                   face.v[static_cast<std::size_t>((e + 1) % 3)]};
                    const auto twin = std::find(horizon.begin(), horizon.end(),
                                                std::pair<std::size_t, std::size_t>{edge.second, edge.first});
                    if (twin != horizon.end()) {
                        horizon.erase(twin);
                    } else {
                        horizon.push_back(edge);
                    }
                }
            } else {
                kept.push_back(face);
            }
        }
        if (kept.size() == faces.size()) {
            break;
        }
        bool degenerate = false;
        for (const auto& [i, j] : horizon) {
            Face face;
            if (!make_face(pts, i, j, index, face)) {
                degenerate = true;
                break;
            }
            kept.push_back(face);
        }
        if (degenerate) {
            break;
        }
        faces = kept;
        best = 0;
    }
    if (!converged) {
        throw SolverNonConvergenceError(0.0, std::string("EPA did not reach its tolerance within ") +
                                                 std::to_string(pair.limits.epa_iterations) + " iterations on " +
                                                 shape_name(pair.a.kind) + "-" + shape_name(pair.b.kind));
    }
    best = 0;
    for (std::size_t f = 1; f < faces.size(); ++f) {
        if (faces[f].dist < faces[best].dist) {
            best = f;
        }
    }
    const Face& face = faces[best];
    if (!(face.dist > 0.0)) {
        return {};
    }
    const Eigen::Vector3d& a = pts[face.v[0]].w;
    const Eigen::Vector3d& b = pts[face.v[1]].w;
    const Eigen::Vector3d& c = pts[face.v[2]].w;
    const Eigen::Vector3d x = face.dist * face.n;
    const Eigen::Vector3d v0 = b - a;
    const Eigen::Vector3d v1 = c - a;
    const Eigen::Vector3d v2 = x - a;
    const double d00 = v0.dot(v0);
    const double d01 = v0.dot(v1);
    const double d11 = v1.dot(v1);
    const double d20 = v2.dot(v0);
    const double d21 = v2.dot(v1);
    const double denom = d00 * d11 - d01 * d01;
    double lb = 0.0;
    double lc = 0.0;
    if (denom > 0.0) {
        lb = (d11 * d20 - d01 * d21) / denom;
        lc = (d00 * d21 - d01 * d20) / denom;
    }
    const double la = 1.0 - lb - lc;
    const Eigen::Vector3d witness_a = la * pts[face.v[0]].a + lb * pts[face.v[1]].a + lc * pts[face.v[2]].a;
    const Eigen::Vector3d witness_b = la * pts[face.v[0]].b + lb * pts[face.v[1]].b + lc * pts[face.v[2]].b;
    ContactPoint out;
    out.normal = face.n;
    out.depth = face.dist;
    out.point = 0.5 * (witness_a + witness_b);
    return {out};
}

std::vector<ContactPoint> gjk_epa(const Shape& a, const Pose& pa, const Shape& b, const Pose& pb,
                                  const CollisionLimits& limits) {
    const Pair pair{a, pa, b, pb, limits};
    std::vector<SupportPoint> simplex;
    if (!gjk(pair, simplex)) {
        return {};
    }
    double scale = 0.0;
    for (const Eigen::Vector3d& dir : {Eigen::Vector3d(Eigen::Vector3d::UnitX()), Eigen::Vector3d(Eigen::Vector3d::UnitY()),
                                       Eigen::Vector3d(Eigen::Vector3d::UnitZ())}) {
        scale = std::max(scale, (pair(dir).w - pair(-dir).w).norm());
    }
    const double eps = degenerate_scale * scale;
    if (!complete_simplex(pair, simplex, eps)) {
        return {};
    }
    return epa(pair, simplex);
}

std::vector<ContactPoint> flipped(std::vector<ContactPoint> contacts) {
    for (ContactPoint& c : contacts) {
        c.normal = -c.normal;
    }
    return contacts;
}

}  // namespace

// ===== Narrow phase ======================================================

std::vector<ContactPoint> collide(const Shape& a, const Pose& pa, const Shape& b, const Pose& pb,
                                  const CollisionLimits& limits) {
    if (limits.gjk_iterations < 1 || limits.epa_iterations < 1 || !std::isfinite(limits.tolerance) ||
        !(limits.tolerance > 0.0)) {
        throw InvalidConfigError(0.0, "collision limits need iteration caps >= 1 and a finite tolerance > 0");
    }
    if (a.kind == ShapeKind::Plane && b.kind == ShapeKind::Plane) {
        return {};
    }
    if (b.kind == ShapeKind::Plane) {
        return shape_plane(a, pa, pb);
    }
    if (a.kind == ShapeKind::Plane) {
        return flipped(shape_plane(b, pb, pa));
    }
    const bool a_round = a.kind == ShapeKind::Sphere || a.kind == ShapeKind::Capsule;
    const bool b_round = b.kind == ShapeKind::Sphere || b.kind == ShapeKind::Capsule;
    if (a_round && b_round) {
        if (a.kind == ShapeKind::Sphere && b.kind == ShapeKind::Sphere) {
            return sphere_sphere(pa.position, a.radius, pb.position, b.radius);
        }
        if (a.kind == ShapeKind::Sphere) {
            const std::array<Eigen::Vector3d, 2> s = segment(b, pb);
            return sphere_sphere(pa.position, a.radius, closest_on_segment(pa.position, s[0], s[1]), b.radius);
        }
        if (b.kind == ShapeKind::Sphere) {
            const std::array<Eigen::Vector3d, 2> s = segment(a, pa);
            return sphere_sphere(closest_on_segment(pb.position, s[0], s[1]), a.radius, pb.position, b.radius);
        }
        const std::array<Eigen::Vector3d, 2> sa = segment(a, pa);
        const std::array<Eigen::Vector3d, 2> sb = segment(b, pb);
        const auto [xa, xb] = closest_segments(sa[0], sa[1], sb[0], sb[1]);
        return sphere_sphere(xa, a.radius, xb, b.radius);
    }
    if (a.kind == ShapeKind::Sphere && b.kind == ShapeKind::Box) {
        return sphere_box(a, pa, b, pb);
    }
    if (a.kind == ShapeKind::Box && b.kind == ShapeKind::Sphere) {
        return flipped(sphere_box(b, pb, a, pa));
    }
    return gjk_epa(a, pa, b, pb, limits);
}

}  // namespace mechsim::engine
