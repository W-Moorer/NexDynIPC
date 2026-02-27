#pragma once

#include "Impact.h"
#include "TimeOfImpact.h"
#include "NexDynIPC/Physics/Contact/ContactAssembler.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include <vector>
#include <memory>
#include <limits>
#include <unordered_map>

namespace NexDynIPC::Physics::CCD {

using namespace NexDynIPC::Dynamics;

class CCDSystem {
public:
    CCDSystem() : toi_calculator_() {}

    void setTOITolerance(double tol) { toi_calculator_.setTOITolerance(tol); }

    double computeEarliestTOI(
        const std::vector<std::shared_ptr<RigidBody>>& bodies,
        double dt,
        const Contact::ContactCandidateSet& candidates) const {

        if (bodies.empty() || candidates.fv.empty()) {
            return 1.0;
        }

        std::unordered_map<int, int> body_id_to_index;
        body_id_to_index.reserve(bodies.size());
        for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
            body_id_to_index[bodies[i]->id] = i;
        }

        auto pose_t0 = [&](const std::shared_ptr<RigidBody>& body, Eigen::Vector3d& p, Eigen::Quaterniond& q) {
            p = body->position;
            q = body->orientation;
        };

        auto pose_t1 = [&](const std::shared_ptr<RigidBody>& body, const Eigen::Vector3d& p0, const Eigen::Quaterniond& q0,
                           Eigen::Vector3d& p1, Eigen::Quaterniond& q1) {
            p1 = p0 + body->velocity * dt;
            q1 = q0;
        };

        double earliest_toi = 1.0;

        for (const auto& fv : candidates.fv) {
            auto it_face = body_id_to_index.find(fv.face_body_id);
            auto it_vert = body_id_to_index.find(fv.vertex_body_id);
            if (it_face == body_id_to_index.end() || it_vert == body_id_to_index.end()) {
                continue;
            }

            const auto& face_body = bodies[it_face->second];
            const auto& vert_body = bodies[it_vert->second];

            double toi = std::numeric_limits<double>::infinity();
            bool collision = false;

            Eigen::Vector3d face_p0, vert_p0, face_p1, vert_p1;
            Eigen::Quaterniond face_q0, vert_q0, face_q1, vert_q1;

            pose_t0(face_body, face_p0, face_q0);
            pose_t0(vert_body, vert_p0, vert_q0);
            pose_t1(face_body, face_p0, face_q0, face_p1, face_q1);
            pose_t1(vert_body, vert_p0, vert_q0, vert_p1, vert_q1);

            collision = toi_calculator_.computeFaceVertex(
                *vert_body, vert_p0, vert_q0, vert_p1, vert_q1, fv.vertex_local_id,
                *face_body, face_p0, face_q0, face_p1, face_q1, fv.face_local_id,
                toi, earliest_toi);

            if (collision && toi < earliest_toi) {
                earliest_toi = toi;
            }
        }

        return earliest_toi;
    }

private:
    TimeOfImpactCalculator toi_calculator_;
};

} // namespace NexDynIPC::Physics::CCD
