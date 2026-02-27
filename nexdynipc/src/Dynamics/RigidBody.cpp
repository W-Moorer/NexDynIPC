#include "NexDynIPC/Dynamics/RigidBody.h"

namespace NexDynIPC::Dynamics {

Eigen::Matrix3d RigidBody::getInertiaWorld() const {
    Eigen::Matrix3d R = orientation.toRotationMatrix();
    return R * inertia_body * R.transpose();
}

Eigen::Vector3d RigidBody::toWorld(const Eigen::Vector3d& p_local) const {
    return orientation * p_local + position;
}

Eigen::Vector3d RigidBody::toLocal(const Eigen::Vector3d& p_world) const {
    return orientation.inverse() * (p_world - position);
}

} // namespace NexDynIPC::Dynamics
