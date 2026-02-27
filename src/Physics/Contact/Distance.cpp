#include "NexDynIPC/Physics/Contact/Distance.h"
#include "NexDynIPC/Physics/Geometry/MeshShape.h"
#include <algorithm>
#include <limits>

namespace NexDynIPC::Physics::Contact {

namespace {

double clamp01(double x)
{
    return std::max(0.0, std::min(1.0, x));
}

Eigen::MatrixXd worldVertices(const Dynamics::RigidBody& body, const Physics::MeshShape& mesh)
{
    Eigen::MatrixXd vw(mesh.vertices().rows(), 3);
    for (int i = 0; i < mesh.vertices().rows(); ++i) {
        const Eigen::Vector3d local = mesh.vertices().row(i).transpose();
        vw.row(i) = body.toWorld(local);
    }
    return vw;
}

} // namespace

double pointPointDistance(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1)
{
    return (p0 - p1).norm();
}

double pointEdgeDistance(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& e0,
    const Eigen::Vector3d& e1,
    Eigen::Vector3d& closest_edge_point)
{
    const Eigen::Vector3d edge = e1 - e0;
    const double edge_len2 = edge.squaredNorm();
    if (edge_len2 < 1e-20) {
        closest_edge_point = e0;
        return (p - e0).norm();
    }

    const double t = clamp01((p - e0).dot(edge) / edge_len2);
    closest_edge_point = e0 + t * edge;
    return (p - closest_edge_point).norm();
}

double pointTriangleDistance(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& t0,
    const Eigen::Vector3d& t1,
    const Eigen::Vector3d& t2,
    Eigen::Vector3d& closest_triangle_point)
{
    const Eigen::Vector3d ab = t1 - t0;
    const Eigen::Vector3d ac = t2 - t0;
    const Eigen::Vector3d ap = p - t0;

    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        closest_triangle_point = t0;
        return (p - t0).norm();
    }

    const Eigen::Vector3d bp = p - t1;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3) {
        closest_triangle_point = t1;
        return (p - t1).norm();
    }

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        closest_triangle_point = t0 + v * ab;
        return (p - closest_triangle_point).norm();
    }

    const Eigen::Vector3d cp = p - t2;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6) {
        closest_triangle_point = t2;
        return (p - t2).norm();
    }

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = d2 / (d2 - d6);
        closest_triangle_point = t0 + w * ac;
        return (p - closest_triangle_point).norm();
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const Eigen::Vector3d bc = t2 - t1;
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        closest_triangle_point = t1 + w * bc;
        return (p - closest_triangle_point).norm();
    }

    const double denom = 1.0 / (va + vb + vc);
    const double v = vb * denom;
    const double w = vc * denom;
    closest_triangle_point = t0 + ab * v + ac * w;
    return (p - closest_triangle_point).norm();
}

double edgeEdgeDistance(
    const Eigen::Vector3d& a0,
    const Eigen::Vector3d& a1,
    const Eigen::Vector3d& b0,
    const Eigen::Vector3d& b1,
    Eigen::Vector3d& closest_a,
    Eigen::Vector3d& closest_b)
{
    const Eigen::Vector3d u = a1 - a0;
    const Eigen::Vector3d v = b1 - b0;
    const Eigen::Vector3d w = a0 - b0;

    const double a = u.dot(u);
    const double b = u.dot(v);
    const double c = v.dot(v);
    const double d = u.dot(w);
    const double e = v.dot(w);
    const double D = a * c - b * b;

    double sN;
    double sD = D;
    double tN;
    double tD = D;

    if (D < 1e-20) {
        sN = 0.0;
        sD = 1.0;
        tN = e;
        tD = c;
    } else {
        sN = (b * e - c * d);
        tN = (a * e - b * d);
        if (sN < 0.0) {
            sN = 0.0;
            tN = e;
            tD = c;
        } else if (sN > sD) {
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {
        tN = 0.0;
        if (-d < 0.0) {
            sN = 0.0;
        } else if (-d > a) {
            sN = sD;
        } else {
            sN = -d;
            sD = a;
        }
    } else if (tN > tD) {
        tN = tD;
        if ((-d + b) < 0.0) {
            sN = 0.0;
        } else if ((-d + b) > a) {
            sN = sD;
        } else {
            sN = (-d + b);
            sD = a;
        }
    }

    const double sc = (std::abs(sN) < 1e-20 ? 0.0 : sN / sD);
    const double tc = (std::abs(tN) < 1e-20 ? 0.0 : tN / tD);

    closest_a = a0 + sc * u;
    closest_b = b0 + tc * v;
    return (closest_a - closest_b).norm();
}

std::vector<ContactDistanceResult> computeNarrowPhaseDistances(
    const std::vector<CollisionCandidate>& candidates,
    const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies,
    double max_distance)
{
    std::vector<ContactDistanceResult> results;
    results.reserve(candidates.size());

    for (const auto& candidate : candidates) {
        if (candidate.bodyA_idx < 0 || candidate.bodyB_idx < 0
            || candidate.bodyA_idx >= static_cast<int>(bodies.size())
            || candidate.bodyB_idx >= static_cast<int>(bodies.size())) {
            continue;
        }

        const auto& bodyA = bodies[static_cast<size_t>(candidate.bodyA_idx)];
        const auto& bodyB = bodies[static_cast<size_t>(candidate.bodyB_idx)];
        const auto meshA = std::dynamic_pointer_cast<Physics::MeshShape>(bodyA->shape);
        const auto meshB = std::dynamic_pointer_cast<Physics::MeshShape>(bodyB->shape);

        if (!meshA || !meshB) {
            continue;
        }

        ContactDistanceResult result;
        result.bodyA_idx = candidate.bodyA_idx;
        result.bodyB_idx = candidate.bodyB_idx;
        result.type = candidate.type;
        result.primitiveA_idx = candidate.primitiveA_idx;
        result.primitiveB_idx = candidate.primitiveB_idx;

        const Eigen::MatrixXd VA = worldVertices(*bodyA, *meshA);
        const Eigen::MatrixXd VB = worldVertices(*bodyB, *meshB);

        if (candidate.type == ContactPrimitiveType::VertexFace) {
            if (candidate.primitiveA_idx < 0 || candidate.primitiveA_idx >= VA.rows()
                || candidate.primitiveB_idx < 0 || candidate.primitiveB_idx >= meshB->faces().rows()) {
                continue;
            }

            const Eigen::Vector3d p = VA.row(candidate.primitiveA_idx).transpose();
            const int f0 = meshB->faces()(candidate.primitiveB_idx, 0);
            const int f1 = meshB->faces()(candidate.primitiveB_idx, 1);
            const int f2 = meshB->faces()(candidate.primitiveB_idx, 2);
            const Eigen::Vector3d t0 = VB.row(f0).transpose();
            const Eigen::Vector3d t1 = VB.row(f1).transpose();
            const Eigen::Vector3d t2 = VB.row(f2).transpose();

            Eigen::Vector3d q = Eigen::Vector3d::Zero();
            const double d = pointTriangleDistance(p, t0, t1, t2, q);
            if (d > max_distance) {
                continue;
            }

            result.distance = d;
            result.pointA = p;
            result.pointB = q;
            const Eigen::Vector3d delta = q - p;
            result.normal = delta.norm() > 1e-12 ? delta.normalized() : Eigen::Vector3d::UnitY();
            results.push_back(result);
        }
    }

    return results;
}

} // namespace NexDynIPC::Physics::Contact
