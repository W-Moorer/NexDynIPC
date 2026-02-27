#include "NexDynIPC/Dynamics/IPCSolver.h"
#include "NexDynIPC/Dynamics/Forms/InertiaForm.h"
#include "NexDynIPC/Dynamics/Forms/GravityForm.h"
#include "NexDynIPC/Dynamics/Forms/ConstraintForm.h"
#include "NexDynIPC/Dynamics/Joints/HingeJoint.h"
#include "NexDynIPC/Dynamics/Joints/FixedJoint.h"
#include "NexDynIPC/Dynamics/Joints/SphericalJoint.h"
#include "NexDynIPC/Dynamics/Joints/PrismaticJoint.h"
#include "NexDynIPC/Dynamics/Joints/CylindricalJoint.h"
#include "NexDynIPC/Dynamics/Joints/AngleLimitJoint.h"
#include "NexDynIPC/Dynamics/Joints/DistanceLimitJoint.h"
#include "NexDynIPC/Control/VelocityDriveForm.h"
#include "NexDynIPC/Control/LinearVelocityDriveForm.h"
#include "NexDynIPC/Control/PositionDriveForm.h"
#include "NexDynIPC/Control/ForceDriveForm.h"
#include "NexDynIPC/Control/DampedSpringForm.h"
#include "NexDynIPC/TimeIntegration/ImplicitEulerIntegrator.h"
#include "NexDynIPC/Physics/Geometry/MeshShape.h"
#include <iostream>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <limits>
#include <filesystem>
#include <fstream>

namespace NexDynIPC::Dynamics {

namespace {
constexpr double kMaxALMStiffness = 1e12;

bool closestPointsOnSegments(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& q1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& q2,
    Eigen::Vector3d& c1,
    Eigen::Vector3d& c2,
    double& s,
    double& t)
{
    constexpr double eps = 1e-12;
    const Eigen::Vector3d d1 = q1 - p1;
    const Eigen::Vector3d d2 = q2 - p2;
    const Eigen::Vector3d r = p1 - p2;
    const double a = d1.dot(d1);
    const double e = d2.dot(d2);
    const double f = d2.dot(r);

    if (a <= eps && e <= eps) {
        s = 0.0;
        t = 0.0;
        c1 = p1;
        c2 = p2;
        return true;
    }

    if (a <= eps) {
        s = 0.0;
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        const double c = d1.dot(r);
        if (e <= eps) {
            t = 0.0;
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            const double b = d1.dot(d2);
            const double denom = a * e - b * b;

            if (std::abs(denom) > eps) {
                s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
            } else {
                s = 0.0;
            }

            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }

    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
    return true;
}

Eigen::Vector3d safeNormalized(
    const Eigen::Vector3d& v,
    const Eigen::Vector3d& fallback)
{
    const double n = v.norm();
    if (n > 1e-12) {
        return v / n;
    }
    const double fn = fallback.norm();
    if (fn > 1e-12) {
        return fallback / fn;
    }
    return Eigen::Vector3d::UnitY();
}

bool pointInTriangleBarycentric(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    double eps = 1e-8)
{
    const Eigen::Vector3d v0 = b - a;
    const Eigen::Vector3d v1 = c - a;
    const Eigen::Vector3d v2 = p - a;

    const double d00 = v0.dot(v0);
    const double d01 = v0.dot(v1);
    const double d11 = v1.dot(v1);
    const double d20 = v2.dot(v0);
    const double d21 = v2.dot(v1);

    const double denom = d00 * d11 - d01 * d01;
    if (std::abs(denom) < 1e-16) {
        return false;
    }

    const double v = (d11 * d20 - d01 * d21) / denom;
    const double w = (d00 * d21 - d01 * d20) / denom;
    const double u = 1.0 - v - w;

    return u >= -eps && v >= -eps && w >= -eps;
}
}

class SimulationProblem : public Math::OptimizationProblem {
public:
    SimulationProblem(World& world, std::shared_ptr<TimeIntegration::ImplicitTimeIntegrator> integrator) 
        : world_(world), integrator_(integrator) {
        inertia_form_ = std::make_unique<InertiaForm>(world, integrator_->dt());
        gravity_form_ = std::make_unique<GravityForm>(world, Eigen::Vector3d(0, -9.81, 0));
        
        constraint_form_ = std::make_unique<ConstraintForm>();
        for(auto& joint : world.joints) {
            constraint_form_->addJoint(joint);
        }
    }

    void setPredictiveState(const Eigen::VectorXd& x_hat) {
        inertia_form_->setPredictiveState(x_hat);
    }

    double computeValue(const Eigen::VectorXd& x) override {
        double val = inertia_form_->value(x);
        double scaling = integrator_->acceleration_scaling();
        
        val += scaling * gravity_form_->value(x);
        val += constraint_form_->value(x);

        for (const auto& form : world_.forms) {
            val += scaling * form->value(x); 
        }
        return val;
    }

    void computeGradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) override {
        grad.setZero(x.size());
        
        Eigen::VectorXd tmp_grad(x.size());
        double scaling = integrator_->acceleration_scaling();
        
        tmp_grad.setZero();
        inertia_form_->gradient(x, tmp_grad);
        grad += tmp_grad;

        tmp_grad.setZero();
        gravity_form_->gradient(x, tmp_grad);
        grad += scaling * tmp_grad;

        tmp_grad.setZero();
        constraint_form_->gradient(x, tmp_grad);
        grad += tmp_grad;

        for (const auto& form : world_.forms) {
            tmp_grad.setZero();
            form->gradient(x, tmp_grad);
            grad += scaling * tmp_grad;
        }
    }

    void computeHessian(const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& hess) override {
        std::vector<Eigen::Triplet<double>> triplets;
        double scaling = integrator_->acceleration_scaling();
        
        inertia_form_->hessian(x, triplets);
        
        std::vector<Eigen::Triplet<double>> tmp_triplets;
        
        gravity_form_->hessian(x, tmp_triplets);
        for(auto& t : tmp_triplets) {
            triplets.emplace_back(t.row(), t.col(), t.value() * scaling);
        }

        tmp_triplets.clear();
        constraint_form_->hessian(x, tmp_triplets);
        for(auto& t : tmp_triplets) {
            triplets.emplace_back(t.row(), t.col(), t.value());
        }

        for (const auto& form : world_.forms) {
            tmp_triplets.clear();
            form->hessian(x, tmp_triplets);
            for(auto& t : tmp_triplets) {
                triplets.emplace_back(t.row(), t.col(), t.value() * scaling);
            }
        }

        hess.resize(x.size(), x.size());
        hess.setFromTriplets(triplets.begin(), triplets.end());
    }

    double computeContactValue(const Eigen::VectorXd& x) override {
        double val = 0.0;
        const double scaling = integrator_->acceleration_scaling();
        for (const auto& form : world_.forms) {
            const bool is_contact_form =
                static_cast<bool>(std::dynamic_pointer_cast<DistanceBarrierForm>(form)) ||
                static_cast<bool>(std::dynamic_pointer_cast<FrictionForm>(form));
            if (is_contact_form) {
                val += scaling * form->value(x);
            }
        }
        return val;
    }

    void computeContactGradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) override {
        grad.setZero(x.size());
        const double scaling = integrator_->acceleration_scaling();
        Eigen::VectorXd tmp_grad(x.size());
        for (const auto& form : world_.forms) {
            const bool is_contact_form =
                static_cast<bool>(std::dynamic_pointer_cast<DistanceBarrierForm>(form)) ||
                static_cast<bool>(std::dynamic_pointer_cast<FrictionForm>(form));
            if (!is_contact_form) {
                continue;
            }
            tmp_grad.setZero();
            form->gradient(x, tmp_grad);
            grad += scaling * tmp_grad;
        }
    }

    ConstraintForm* getConstraintForm() { return constraint_form_.get(); }

private:
    World& world_;
    std::shared_ptr<TimeIntegration::ImplicitTimeIntegrator> integrator_;
    std::unique_ptr<InertiaForm> inertia_form_;
    std::unique_ptr<GravityForm> gravity_form_;
    std::unique_ptr<ConstraintForm> constraint_form_;
};

IPCSolver::IPCSolver() {
    integrator_ = std::make_shared<TimeIntegration::ImplicitEuler>();
}

std::uint64_t IPCSolver::makePairKey(int body_id_a, int body_id_b) {
    const int lo = std::min(body_id_a, body_id_b);
    const int hi = std::max(body_id_a, body_id_b);
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(lo)) << 32) |
           static_cast<std::uint32_t>(hi);
}

void IPCSolver::addAllowedContactPair(int body_id_a, int body_id_b, double friction) {
    const auto key = makePairKey(body_id_a, body_id_b);
    allowed_contact_pair_keys_.insert(key);
    if (friction >= 0.0) {
        pair_friction_overrides_[key] = friction;
    }
}

bool IPCSolver::isContactPairAllowed(int body_id_a, int body_id_b) const {
    if (allowed_contact_pair_keys_.empty()) {
        return true;
    }
    const auto key = makePairKey(body_id_a, body_id_b);
    return allowed_contact_pair_keys_.find(key) != allowed_contact_pair_keys_.end();
}

double IPCSolver::estimateBroadPhaseMinDistance(double inflation_radius) {
    double min_dist = contact_assembler_.estimateMinBodyPairDistance(inflation_radius);
    if (!std::isfinite(min_dist)) {
        return std::numeric_limits<double>::infinity();
    }
    return min_dist;
}

void IPCSolver::setIntegrator(std::shared_ptr<TimeIntegration::ImplicitTimeIntegrator> integrator) {
    integrator_ = integrator;
}

bool IPCSolver::writeLastContactResidualSeriesCSV(const std::string& file_path) const {
    std::filesystem::path out_path(file_path);
    if (out_path.has_parent_path()) {
        std::filesystem::create_directories(out_path.parent_path());
    }

    std::ofstream out(out_path, std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }

    out << "alm_iteration,newton_iteration,contact_residual,line_search_alpha,contact_value_before,contact_value_after\n";
    for (const auto& sample : last_contact_residual_series_) {
        out << sample.alm_iteration << ","
            << sample.newton_iteration << ","
            << sample.contact_residual << ","
            << sample.line_search_alpha << ","
            << sample.contact_value_before << ","
            << sample.contact_value_after << "\n";
    }

    return static_cast<bool>(out);
}

void IPCSolver::initializeAdaptiveBarrier(World& world) {
    if (world.bodies.empty()) return;
    
    double total_mass = 0.0;
    Eigen::Vector3d bbox_min(std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity());
    Eigen::Vector3d bbox_max(-std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity());
    
    for (const auto& body : world.bodies) {
        total_mass += body->mass;
        
        bbox_min = bbox_min.cwiseMin(body->position);
        bbox_max = bbox_max.cwiseMax(body->position);
    }
    
    double avg_mass = total_mass / world.bodies.size();
    double bbox_diagonal = (bbox_max - bbox_min).norm();
    
    if (bbox_diagonal < 1e-6) {
        bbox_diagonal = 1.0;
    }
    
    adaptive_barrier_.initialize(avg_mass, bbox_diagonal);
}

double IPCSolver::computeMaxStep(World& world, double dt) {
    if (!enable_ccd_ || world.bodies.empty()) {
        return dt;
    }

    double max_speed = 0.0;
    for (const auto& body : world.bodies) {
        max_speed = std::max(max_speed, body->velocity.norm());
    }

    const double close_bodies_inflation = std::max(1e-6, adaptive_barrier_.currentDhat()) + max_speed * dt;
    const auto close_pairs = contact_assembler_.detectBodyPairs(close_bodies_inflation);

    bool has_allowed_close_pair = false;
    for (const auto& wp : close_pairs) {
        const int id_i = world.bodies[wp.first]->id;
        const int id_j = world.bodies[wp.second]->id;
        if (isContactPairAllowed(id_i, id_j)) {
            has_allowed_close_pair = true;
            break;
        }
    }
    if (!has_allowed_close_pair) {
        return dt;
    }

    if (contact_candidates_.fv.empty()) {
        return dt;
    }
    
    double toi = ccd_system_.computeEarliestTOI(world.bodies, dt, contact_candidates_);
    
    if (toi < 1.0) {
        return toi * ccd_safety_factor_ * dt;
    }
    
    return dt;
}

std::vector<ContactPair> IPCSolver::detectContacts(World& world) {
    std::vector<ContactPair> contacts;

    if (world.bodies.empty()) {
        return contacts;
    }

    std::unordered_map<int, int> body_id_to_world_index;
    body_id_to_world_index.reserve(world.bodies.size());
    for (int i = 0; i < static_cast<int>(world.bodies.size()); ++i) {
        body_id_to_world_index[world.bodies[i]->id] = i;
    }

    std::unordered_map<int, const Physics::Contact::BodyContactMesh*> body_id_to_mesh;
    for (const auto& mesh : contact_assembler_.bodyMeshes()) {
        body_id_to_mesh[mesh.body_id] = &mesh;
    }

    const double dhat = adaptive_barrier_.currentDhat();
    constexpr size_t kMaxRawContacts = 40000;
    constexpr size_t kMaxContacts = 2000;

    auto world_vertex = [&](int body_id, int local_vertex_id, Eigen::Vector3d& out) -> bool {
        const auto mit = body_id_to_mesh.find(body_id);
        const auto bit = body_id_to_world_index.find(body_id);
        if (mit == body_id_to_mesh.end() || bit == body_id_to_world_index.end()) {
            return false;
        }
        const auto* mesh = mit->second;
        if (local_vertex_id < 0 || local_vertex_id >= mesh->vertices.rows()) {
            return false;
        }
        const auto& body = world.bodies[bit->second];
        out = body->position + body->orientation * mesh->vertices.row(local_vertex_id).transpose();
        return true;
    };

    auto append_contact = [&](int bodyA_id,
                              int bodyB_id,
                              int primitive_type,
                              int primitiveA_idx,
                              int primitiveB_idx,
                              const Eigen::Vector3d& point,
                              const Eigen::Vector3d& normal,
                              double distance) {
        if (contacts.size() >= kMaxRawContacts) {
            return;
        }

        const auto ait = body_id_to_world_index.find(bodyA_id);
        const auto bit = body_id_to_world_index.find(bodyB_id);
        if (ait == body_id_to_world_index.end() || bit == body_id_to_world_index.end()) {
            return;
        }
        if (!isContactPairAllowed(bodyA_id, bodyB_id)) {
            return;
        }

        const Eigen::Vector3d pa = world.bodies[ait->second]->position;
        const Eigen::Vector3d pb = world.bodies[bit->second]->position;
        Eigen::Vector3d n = safeNormalized(normal, pb - pa);
        if (n.dot(pb - pa) < 0.0) {
            n = -n;
        }

        ContactPair c;
        c.bodyA_idx = ait->second;
        c.bodyB_idx = bit->second;
        c.primitive_type = primitive_type;
        c.primitiveA_idx = primitiveA_idx;
        c.primitiveB_idx = primitiveB_idx;
        c.contact_point = point;
        c.normal = n;
        c.distance = std::max(0.0, distance);
        contacts.push_back(c);
    };

    for (const auto& cand : contact_candidates_.fv) {
        const auto fit = body_id_to_mesh.find(cand.face_body_id);
        if (fit == body_id_to_mesh.end()) {
            continue;
        }
        const auto* face_mesh = fit->second;
        if (cand.face_local_id < 0 || cand.face_local_id >= face_mesh->faces.rows()) {
            continue;
        }

        const int i0 = face_mesh->faces(cand.face_local_id, 0);
        const int i1 = face_mesh->faces(cand.face_local_id, 1);
        const int i2 = face_mesh->faces(cand.face_local_id, 2);

        Eigen::Vector3d v0, v1, v2, p;
        if (!world_vertex(cand.face_body_id, i0, v0) ||
            !world_vertex(cand.face_body_id, i1, v1) ||
            !world_vertex(cand.face_body_id, i2, v2) ||
            !world_vertex(cand.vertex_body_id, cand.vertex_local_id, p)) {
            continue;
        }

        const Eigen::Vector3d raw_n = (v1 - v0).cross(v2 - v0);
        const Eigen::Vector3d n = safeNormalized(raw_n, Eigen::Vector3d::UnitY());
        const double signed_dist = (p - v0).dot(n);
        const double dist = std::abs(signed_dist);
        if (dist >= dhat) {
            continue;
        }
        const Eigen::Vector3d proj = p - signed_dist * n;
        if (!pointInTriangleBarycentric(proj, v0, v1, v2)) {
            continue;
        }
        append_contact(cand.face_body_id,
                       cand.vertex_body_id,
                       0,
                       cand.face_local_id,
                       cand.vertex_local_id,
                       0.5 * (proj + p),
                       (signed_dist >= 0.0) ? n : -n,
                       dist);
    }

    for (const auto& cand : contact_candidates_.ee) {
        Eigen::Vector3d a0, a1, b0, b1;
        if (!world_vertex(cand.body0_id, cand.edge0_vertex0_local_id, a0) ||
            !world_vertex(cand.body0_id, cand.edge0_vertex1_local_id, a1) ||
            !world_vertex(cand.body1_id, cand.edge1_vertex0_local_id, b0) ||
            !world_vertex(cand.body1_id, cand.edge1_vertex1_local_id, b1)) {
            continue;
        }

        Eigen::Vector3d c0, c1;
        double s = 0.0;
        double t = 0.0;
        if (!closestPointsOnSegments(a0, a1, b0, b1, c0, c1, s, t)) {
            continue;
        }

        const Eigen::Vector3d delta = c1 - c0;
        const double dist = delta.norm();
        if (dist >= dhat) {
            continue;
        }

        append_contact(cand.body0_id,
                       cand.body1_id,
                       1,
                       cand.edge0_vertex0_local_id,
                       cand.edge1_vertex0_local_id,
                       0.5 * (c0 + c1),
                       delta,
                       dist);
    }

    if (contacts.empty()) {
        for (const auto& cand : contact_candidates_.vv) {
            Eigen::Vector3d a, b;
            if (!world_vertex(cand.body0_id, cand.vertex0_local_id, a) ||
                !world_vertex(cand.body1_id, cand.vertex1_local_id, b)) {
                continue;
            }

            const Eigen::Vector3d d = b - a;
            const double dist = d.norm();
            if (dist >= dhat) {
                continue;
            }

            append_contact(cand.body0_id,
                           cand.body1_id,
                           0,
                           cand.vertex0_local_id,
                           cand.vertex1_local_id,
                           0.5 * (a + b),
                           d,
                           dist);
        }
    }

    if (contacts.size() > kMaxContacts) {
        std::sort(contacts.begin(), contacts.end(),
            [](const ContactPair& a, const ContactPair& b) {
                return a.distance < b.distance;
            });
        contacts.resize(kMaxContacts);
    }

    return contacts;
}

std::vector<ContactPair> IPCSolver::detectContactsAtState(World& world, const Eigen::VectorXd& x_state) const {
    std::vector<ContactPair> contacts;

    if (world.bodies.empty()) {
        return contacts;
    }

    std::unordered_map<int, int> body_id_to_world_index;
    body_id_to_world_index.reserve(world.bodies.size());
    for (int i = 0; i < static_cast<int>(world.bodies.size()); ++i) {
        body_id_to_world_index[world.bodies[i]->id] = i;
    }

    std::unordered_map<int, const Physics::Contact::BodyContactMesh*> body_id_to_mesh;
    for (const auto& mesh : contact_assembler_.bodyMeshes()) {
        body_id_to_mesh[mesh.body_id] = &mesh;
    }

    const double dhat = adaptive_barrier_.currentDhat();
    constexpr size_t kMaxRawContacts = 40000;
    constexpr size_t kMaxContacts = 2000;

    auto body_world_transform = [&](int body_id, Eigen::Vector3d& position, Eigen::Quaterniond& orientation) -> bool {
        const auto bit = body_id_to_world_index.find(body_id);
        if (bit == body_id_to_world_index.end()) {
            return false;
        }

        const int body_idx = bit->second;
        const int dof_idx = body_idx * 6;
        if (dof_idx + 5 >= x_state.size()) {
            return false;
        }

        position = x_state.segment<3>(dof_idx);

        const Eigen::Vector3d theta = x_state.segment<3>(dof_idx + 3);
        const double angle = theta.norm();
        if (angle > 1e-10) {
            const Eigen::Vector3d axis = theta / angle;
            const Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
            orientation = (dq * world.bodies[body_idx]->orientation).normalized();
        } else {
            orientation = world.bodies[body_idx]->orientation;
        }

        return true;
    };

    auto world_vertex = [&](int body_id, int local_vertex_id, Eigen::Vector3d& out) -> bool {
        const auto mit = body_id_to_mesh.find(body_id);
        if (mit == body_id_to_mesh.end()) {
            return false;
        }
        const auto* mesh = mit->second;
        if (local_vertex_id < 0 || local_vertex_id >= mesh->vertices.rows()) {
            return false;
        }

        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        if (!body_world_transform(body_id, position, orientation)) {
            return false;
        }

        out = position + orientation * mesh->vertices.row(local_vertex_id).transpose();
        return true;
    };

    auto append_contact = [&](int bodyA_id,
                              int bodyB_id,
                              int primitive_type,
                              int primitiveA_idx,
                              int primitiveB_idx,
                              const Eigen::Vector3d& point,
                              const Eigen::Vector3d& normal,
                              double distance) {
        if (contacts.size() >= kMaxRawContacts) {
            return;
        }

        const auto ait = body_id_to_world_index.find(bodyA_id);
        const auto bit = body_id_to_world_index.find(bodyB_id);
        if (ait == body_id_to_world_index.end() || bit == body_id_to_world_index.end()) {
            return;
        }
        if (!isContactPairAllowed(bodyA_id, bodyB_id)) {
            return;
        }

        Eigen::Vector3d posA;
        Eigen::Quaterniond oriA;
        Eigen::Vector3d posB;
        Eigen::Quaterniond oriB;
        if (!body_world_transform(bodyA_id, posA, oriA) || !body_world_transform(bodyB_id, posB, oriB)) {
            return;
        }

        Eigen::Vector3d n = safeNormalized(normal, posB - posA);
        if (n.dot(posB - posA) < 0.0) {
            n = -n;
        }

        ContactPair c;
        c.bodyA_idx = ait->second;
        c.bodyB_idx = bit->second;
        c.primitive_type = primitive_type;
        c.primitiveA_idx = primitiveA_idx;
        c.primitiveB_idx = primitiveB_idx;
        c.contact_point = point;
        c.normal = n;
        c.distance = std::max(0.0, distance);
        contacts.push_back(c);
    };

    for (const auto& cand : contact_candidates_.fv) {
        const auto fit = body_id_to_mesh.find(cand.face_body_id);
        if (fit == body_id_to_mesh.end()) {
            continue;
        }
        const auto* face_mesh = fit->second;
        if (cand.face_local_id < 0 || cand.face_local_id >= face_mesh->faces.rows()) {
            continue;
        }

        const int i0 = face_mesh->faces(cand.face_local_id, 0);
        const int i1 = face_mesh->faces(cand.face_local_id, 1);
        const int i2 = face_mesh->faces(cand.face_local_id, 2);

        Eigen::Vector3d v0, v1, v2, p;
        if (!world_vertex(cand.face_body_id, i0, v0) ||
            !world_vertex(cand.face_body_id, i1, v1) ||
            !world_vertex(cand.face_body_id, i2, v2) ||
            !world_vertex(cand.vertex_body_id, cand.vertex_local_id, p)) {
            continue;
        }

        const Eigen::Vector3d raw_n = (v1 - v0).cross(v2 - v0);
        const Eigen::Vector3d n = safeNormalized(raw_n, Eigen::Vector3d::UnitY());
        const double signed_dist = (p - v0).dot(n);
        const double dist = std::abs(signed_dist);
        if (dist >= dhat) {
            continue;
        }
        const Eigen::Vector3d proj = p - signed_dist * n;
        if (!pointInTriangleBarycentric(proj, v0, v1, v2)) {
            continue;
        }
        append_contact(cand.face_body_id,
                       cand.vertex_body_id,
                       0,
                       cand.face_local_id,
                       cand.vertex_local_id,
                       0.5 * (proj + p),
                       (signed_dist >= 0.0) ? n : -n,
                       dist);
    }

    for (const auto& cand : contact_candidates_.ee) {
        Eigen::Vector3d a0, a1, b0, b1;
        if (!world_vertex(cand.body0_id, cand.edge0_vertex0_local_id, a0) ||
            !world_vertex(cand.body0_id, cand.edge0_vertex1_local_id, a1) ||
            !world_vertex(cand.body1_id, cand.edge1_vertex0_local_id, b0) ||
            !world_vertex(cand.body1_id, cand.edge1_vertex1_local_id, b1)) {
            continue;
        }

        Eigen::Vector3d c0, c1;
        double s = 0.0;
        double t = 0.0;
        if (!closestPointsOnSegments(a0, a1, b0, b1, c0, c1, s, t)) {
            continue;
        }

        const Eigen::Vector3d delta = c1 - c0;
        const double dist = delta.norm();
        if (dist >= dhat) {
            continue;
        }

        append_contact(cand.body0_id,
                       cand.body1_id,
                       1,
                       cand.edge0_vertex0_local_id,
                       cand.edge1_vertex0_local_id,
                       0.5 * (c0 + c1),
                       delta,
                       dist);
    }

    if (contacts.empty()) {
        for (const auto& cand : contact_candidates_.vv) {
            Eigen::Vector3d a, b;
            if (!world_vertex(cand.body0_id, cand.vertex0_local_id, a) ||
                !world_vertex(cand.body1_id, cand.vertex1_local_id, b)) {
                continue;
            }

            const Eigen::Vector3d d = b - a;
            const double dist = d.norm();
            if (dist >= dhat) {
                continue;
            }

            append_contact(cand.body0_id,
                           cand.body1_id,
                           0,
                           cand.vertex0_local_id,
                           cand.vertex1_local_id,
                           0.5 * (a + b),
                           d,
                           dist);
        }
    }

    if (contacts.size() > kMaxContacts) {
        std::sort(contacts.begin(), contacts.end(),
            [](const ContactPair& a, const ContactPair& b) {
                return a.distance < b.distance;
            });
        contacts.resize(kMaxContacts);
    }

    return contacts;
}

void IPCSolver::step(World& world, double dt) {
    if (!integrator_) {
        std::cerr << "Reference Error: Integrator not set in IPCSolver!" << std::endl;
        return;
    }

    contact_assembler_.build(world);
    contact_assembler_.buildCandidateSet(adaptive_barrier_.currentDhat(), contact_candidates_);

    if (enable_adaptive_barrier_) {
        static bool barrier_initialized = false;
        if (!barrier_initialized) {
            initializeAdaptiveBarrier(world);
            barrier_initialized = true;
        }

        const double dhat = adaptive_barrier_.currentDhat();
        const double broad_min_dist = estimateBroadPhaseMinDistance(dhat * 2.0);
        if (broad_min_dist < dhat * 2.0) {
            double min_dist = Physics::Contact::computeSceneMinDistance(world.bodies);
            if (min_dist < std::numeric_limits<double>::infinity()) {
                adaptive_barrier_.updateStiffness(min_dist);
            }
        }
    }

    double actual_dt = dt;
    if (enable_ccd_) {
        actual_dt = computeMaxStep(world, dt);
    }

    int n = static_cast<int>(world.bodies.size()) * 6;
    
    Eigen::VectorXd x_curr = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd v_curr = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd a_curr = Eigen::VectorXd::Zero(n);

    int idx = 0;
    for (const auto& body : world.bodies) {
        x_curr.segment<3>(idx) = body->position;
        x_curr.segment<3>(idx+3) = Eigen::Vector3d::Zero();

        v_curr.segment<3>(idx) = body->velocity;
        v_curr.segment<3>(idx+3) = body->angular_velocity;
        
        a_curr.segment<3>(idx) = body->linear_acceleration;
        a_curr.segment<3>(idx+3) = body->angular_acceleration;
        
        idx += 6;
    }
    integrator_->init(x_curr, v_curr, a_curr, actual_dt);

    Eigen::VectorXd x_hat = integrator_->x_tilde();
    Eigen::VectorXd x_new = x_hat; 

    std::unordered_map<int, int> body_id_to_idx;
    std::unordered_map<int, std::shared_ptr<RigidBody>> body_id_to_ptr;
    idx = 0;
    for (const auto& body : world.bodies) {
        body_id_to_idx[body->id] = idx;
        body_id_to_ptr[body->id] = body;
        idx += 6;
    }

    std::vector<ContactPair> active_contacts;
    const bool needs_contact_forms = enable_adaptive_barrier_ || (enable_friction_ && friction_coeff_ > 0.0);
    if (needs_contact_forms) {
        active_contacts = detectContacts(world);
    }

    if (needs_contact_forms) {
        barrier_form_ = std::make_shared<DistanceBarrierForm>(
            world,
            adaptive_barrier_.currentDhat(),
            adaptive_barrier_.currentStiffness());
        barrier_form_->updateContacts(active_contacts);
    }

    if (enable_adaptive_barrier_ && barrier_form_) {
        world.forms.push_back(barrier_form_);
    }

    // Setup friction form if enabled
    if (enable_friction_ && friction_coeff_ > 0) {
        double effective_mu = friction_coeff_;
        if (!active_contacts.empty() && !pair_friction_overrides_.empty()) {
            double mu_sum = 0.0;
            int mu_count = 0;
            for (const auto& c : active_contacts) {
                const int idA = world.bodies[c.bodyA_idx]->id;
                const int idB = world.bodies[c.bodyB_idx]->id;
                const auto key = makePairKey(idA, idB);
                auto it = pair_friction_overrides_.find(key);
                if (it != pair_friction_overrides_.end()) {
                    mu_sum += it->second;
                } else {
                    mu_sum += friction_coeff_;
                }
                ++mu_count;
            }
            if (mu_count > 0) {
                effective_mu = mu_sum / static_cast<double>(mu_count);
            }
        }

        friction_form_ = std::make_shared<FrictionForm>(world, effective_mu);
        friction_form_->setTimeStep(actual_dt);
        friction_form_->updateContactPoints(active_contacts);
        world.forms.push_back(friction_form_);
    }

    if (barrier_form_) {
        barrier_form_->setParameters(adaptive_barrier_.currentDhat(), adaptive_barrier_.currentStiffness());
    }

    for (const auto& form : world.forms) {
        auto drive = std::dynamic_pointer_cast<NexDynIPC::Control::VelocityDriveForm>(form);
        if (drive) {
            drive->setTimeStep(actual_dt);
            drive->advanceControlState();
            drive->updateGlobalIndices(world.bodies);
            continue;
        }

        auto linear_drive = std::dynamic_pointer_cast<NexDynIPC::Control::LinearVelocityDriveForm>(form);
        if (linear_drive) {
            linear_drive->setTimeStep(actual_dt);
            linear_drive->advanceControlState();
            linear_drive->updateGlobalIndices(world.bodies);
            continue;
        }

        auto position_drive = std::dynamic_pointer_cast<NexDynIPC::Control::PositionDriveForm>(form);
        if (position_drive) {
            position_drive->setTimeStep(actual_dt);
            position_drive->updateGlobalIndices(world.bodies);
            continue;
        }

        auto force_drive = std::dynamic_pointer_cast<NexDynIPC::Control::ForceDriveForm>(form);
        if (force_drive) {
            force_drive->setTimeStep(actual_dt);
            force_drive->updateGlobalIndices(world.bodies);
            continue;
        }

        auto damped_spring = std::dynamic_pointer_cast<NexDynIPC::Control::DampedSpringForm>(form);
        if (damped_spring) {
            damped_spring->updateGlobalIndices(world.bodies);
        }
    }

    last_max_constraint_violation_ = 0.0;
    last_dual_residual_ = 0.0;
    last_contact_residual_ = 0.0;
    last_contact_residual_series_.clear();

    for(int alm_iter = 0; alm_iter < alm_max_iters_; ++alm_iter) {
        for (auto& joint : world.joints) {
            auto hinge = std::dynamic_pointer_cast<NexDynIPC::Dynamics::HingeJoint>(joint);
            if (hinge) {
                int idA = hinge->getBodyAId();
                int idB = hinge->getBodyBId();
                
                int idxA = body_id_to_idx[idA];
                int idxB = body_id_to_idx[idB];
                Eigen::Vector3d pA = body_id_to_ptr[idA]->position;
                Eigen::Quaterniond qA = body_id_to_ptr[idA]->orientation;
                Eigen::Vector3d pB = body_id_to_ptr[idB]->position;
                Eigen::Quaterniond qB = body_id_to_ptr[idB]->orientation;
                hinge->updateState(idxA, idxB, pA, qA, pB, qB);
                continue;
            }

            auto fixed = std::dynamic_pointer_cast<NexDynIPC::Dynamics::FixedJoint>(joint);
            if (fixed) {
                int bodyId = fixed->getBodyId();
                int bodyIdx = body_id_to_idx[bodyId];
                Eigen::Vector3d p = body_id_to_ptr[bodyId]->position;
                Eigen::Quaterniond q = body_id_to_ptr[bodyId]->orientation;
                fixed->updateState(bodyIdx, p, q);
                continue;
            }

            auto spherical = std::dynamic_pointer_cast<NexDynIPC::Dynamics::SphericalJoint>(joint);
            if (spherical) {
                int idA = spherical->getBodyAId();
                int idB = spherical->getBodyBId();

                int idxA = body_id_to_idx[idA];
                int idxB = body_id_to_idx[idB];
                Eigen::Vector3d pA = body_id_to_ptr[idA]->position;
                Eigen::Quaterniond qA = body_id_to_ptr[idA]->orientation;
                Eigen::Vector3d pB = body_id_to_ptr[idB]->position;
                Eigen::Quaterniond qB = body_id_to_ptr[idB]->orientation;
                spherical->updateState(idxA, idxB, pA, qA, pB, qB);
                continue;
            }

            auto prismatic = std::dynamic_pointer_cast<NexDynIPC::Dynamics::PrismaticJoint>(joint);
            if (prismatic) {
                int idA = prismatic->getBodyAId();
                int idB = prismatic->getBodyBId();

                int idxA = body_id_to_idx[idA];
                int idxB = body_id_to_idx[idB];
                Eigen::Vector3d pA = body_id_to_ptr[idA]->position;
                Eigen::Quaterniond qA = body_id_to_ptr[idA]->orientation;
                Eigen::Vector3d pB = body_id_to_ptr[idB]->position;
                Eigen::Quaterniond qB = body_id_to_ptr[idB]->orientation;
                prismatic->updateState(idxA, idxB, pA, qA, pB, qB);
                continue;
            }

            auto cylindrical = std::dynamic_pointer_cast<NexDynIPC::Dynamics::CylindricalJoint>(joint);
            if (cylindrical) {
                int idA = cylindrical->getBodyAId();
                int idB = cylindrical->getBodyBId();

                int idxA = body_id_to_idx[idA];
                int idxB = body_id_to_idx[idB];
                Eigen::Vector3d pA = body_id_to_ptr[idA]->position;
                Eigen::Quaterniond qA = body_id_to_ptr[idA]->orientation;
                Eigen::Vector3d pB = body_id_to_ptr[idB]->position;
                Eigen::Quaterniond qB = body_id_to_ptr[idB]->orientation;
                cylindrical->updateState(idxA, idxB, pA, qA, pB, qB);
                continue;
            }

            auto angle_limit = std::dynamic_pointer_cast<NexDynIPC::Dynamics::AngleLimitJoint>(joint);
            if (angle_limit) {
                int idA = angle_limit->getBodyAId();
                int idB = angle_limit->getBodyBId();

                int idxA = body_id_to_idx[idA];
                int idxB = body_id_to_idx[idB];
                Eigen::Vector3d pA = body_id_to_ptr[idA]->position;
                Eigen::Quaterniond qA = body_id_to_ptr[idA]->orientation;
                Eigen::Vector3d pB = body_id_to_ptr[idB]->position;
                Eigen::Quaterniond qB = body_id_to_ptr[idB]->orientation;
                angle_limit->updateState(idxA, idxB, pA, qA, pB, qB);
                continue;
            }

            auto distance_limit = std::dynamic_pointer_cast<NexDynIPC::Dynamics::DistanceLimitJoint>(joint);
            if (distance_limit) {
                int idA = distance_limit->getBodyAId();
                int idB = distance_limit->getBodyBId();

                int idxA = body_id_to_idx[idA];
                int idxB = body_id_to_idx[idB];
                Eigen::Vector3d pA = body_id_to_ptr[idA]->position;
                Eigen::Quaterniond qA = body_id_to_ptr[idA]->orientation;
                Eigen::Vector3d pB = body_id_to_ptr[idB]->position;
                Eigen::Quaterniond qB = body_id_to_ptr[idB]->orientation;
                distance_limit->updateState(idxA, idxB, pA, qA, pB, qB);
                continue;
            }
        }

        SimulationProblem problem(world, integrator_);
        problem.setPredictiveState(x_hat);

        bool converged = solver_.minimize(problem, x_new);

        for (const auto& newton_diag : solver_.lastIterationDiagnostics()) {
            ContactResidualSample sample;
            sample.alm_iteration = alm_iter;
            sample.newton_iteration = newton_diag.iteration;
            sample.contact_residual = newton_diag.contact_residual;
            sample.line_search_alpha = newton_diag.line_search_alpha;
            sample.contact_value_before = newton_diag.contact_value_before;
            sample.contact_value_after = newton_diag.contact_value_after;
            last_contact_residual_series_.push_back(sample);
        }
        
        double max_constraint_violation = 0.0;
        double dual_residual = 0.0;
        for (const auto& joint : problem.getConstraintForm()->getJoints()) {
            Eigen::VectorXd C;
            joint->computeC(x_new, C);
            if (C.size() > 0) {
                const double inf_norm = C.cwiseAbs().maxCoeff();
                max_constraint_violation = std::max(max_constraint_violation, inf_norm);
                const double dual_candidate = joint->getStiffness() * inf_norm;
                if (std::isfinite(dual_candidate)) {
                    dual_residual = std::max(dual_residual, std::min(dual_candidate, kMaxALMStiffness));
                } else {
                    dual_residual = std::max(dual_residual, kMaxALMStiffness);
                }

                if (inf_norm > alm_hardening_trigger_ * alm_constraint_tolerance_) {
                    const double next_stiffness = joint->getStiffness() * alm_hardening_ratio_;
                    if (std::isfinite(next_stiffness)) {
                        joint->setStiffness(std::min(next_stiffness, kMaxALMStiffness));
                    } else {
                        joint->setStiffness(kMaxALMStiffness);
                    }
                }
            }
        }

        problem.getConstraintForm()->updateLambdas(x_new);
        
        if (barrier_form_ || friction_form_) {
            const auto iter_contacts = detectContactsAtState(world, x_new);

            if (barrier_form_) {
                barrier_form_->updateContacts(iter_contacts);
            }

            if (enable_friction_ && friction_form_) {
                friction_form_->updateContactPoints(iter_contacts);

                std::vector<double> normal_forces;
                if (barrier_form_) {
                    normal_forces = barrier_form_->computeNormalForces(x_new);
                } else {
                    normal_forces.assign(iter_contacts.size(), 0.0);
                }
                friction_form_->updateNormalForces(normal_forces);
            }
        }

        double contact_residual = 0.0;
        if (barrier_form_ || friction_form_) {
            Eigen::VectorXd contact_gradient = Eigen::VectorXd::Zero(x_new.size());

            if (barrier_form_) {
                Eigen::VectorXd barrier_grad = Eigen::VectorXd::Zero(x_new.size());
                barrier_form_->gradient(x_new, barrier_grad);
                contact_gradient += barrier_grad;
            }

            if (enable_friction_ && friction_form_) {
                Eigen::VectorXd friction_grad = Eigen::VectorXd::Zero(x_new.size());
                friction_form_->gradient(x_new, friction_grad);
                contact_gradient += friction_grad;
            }

            if (contact_gradient.size() > 0) {
                contact_residual = contact_gradient.cwiseAbs().maxCoeff();
                if (!std::isfinite(contact_residual)) {
                    contact_residual = kMaxALMStiffness;
                }
            }
        }

        last_max_constraint_violation_ = max_constraint_violation;
        last_dual_residual_ = dual_residual;
        last_contact_residual_ = contact_residual;

        if (converged
            && max_constraint_violation <= alm_constraint_tolerance_
            && dual_residual <= alm_dual_tolerance_
            && contact_residual <= alm_contact_tolerance_) {
            break;
        }
    }

    integrator_->update_quantities(x_new);

    const Eigen::VectorXd& x_final = integrator_->x_prev();
    const Eigen::VectorXd& v_final = integrator_->v_prev();
    const Eigen::VectorXd& a_final = integrator_->a_prev();

    idx = 0;
    for (auto& body : world.bodies) {
        Eigen::Vector3d p_new = x_final.segment<3>(idx);
        Eigen::Vector3d theta_new = x_final.segment<3>(idx+3);

        body->linear_acceleration = a_final.segment<3>(idx);
        body->angular_acceleration = a_final.segment<3>(idx+3);

        body->velocity = v_final.segment<3>(idx);
        body->angular_velocity = v_final.segment<3>(idx+3);

        body->position = p_new;

        double angle = theta_new.norm();
        if (angle > 1e-10) {
            Eigen::Vector3d axis = theta_new / angle;
            Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
            body->orientation = (dq * body->orientation).normalized();
        }

        idx += 6;
    }

    double max_vw = 0.0;
    int drive_count = 0;
    int saturated_count = 0;
    for (const auto& form : world.forms) {
        auto drive = std::dynamic_pointer_cast<NexDynIPC::Control::VelocityDriveForm>(form);
        if (!drive) {
            continue;
        }
        ++drive_count;
        max_vw = std::max(max_vw, std::abs(drive->getVelocityErrorFromBodies()));
        if (drive->isTorqueSaturatedFromBodies()) {
            ++saturated_count;
        }
    }

    last_angular_velocity_error_ = max_vw;
    last_torque_saturation_ratio_ = (drive_count > 0)
        ? static_cast<double>(saturated_count) / static_cast<double>(drive_count)
        : 0.0;
    
    // Remove friction form from world forms after step
    if (enable_friction_ && friction_form_) {
        world.forms.erase(
            std::remove(world.forms.begin(), world.forms.end(), friction_form_),
            world.forms.end());
        friction_form_.reset();
    }

    if (barrier_form_) {
        world.forms.erase(
            std::remove(world.forms.begin(), world.forms.end(), barrier_form_),
            world.forms.end());
        barrier_form_.reset();
    }
}

} // namespace NexDynIPC::Dynamics
