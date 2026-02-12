#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include "NexDynIPC/Physics/Geometry/Shape.hpp"

namespace NexDynIPC::Dynamics {

class RigidBody {
public:
    int id = -1;
    std::string name;
    
    // State
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero();

    // Properties
    double mass = 1.0;
    Eigen::Matrix3d inertia_body = Eigen::Matrix3d::Identity(); // Inertia in local frame
    double friction_coeff = 0.0;
    double restitution_coeff = 0.0;

    // Geometry
    std::shared_ptr<Physics::Shape> shape;

    // Helpers
    Eigen::Matrix3d getInertiaWorld() const;
    Eigen::Vector3d toWorld(const Eigen::Vector3d& p_local) const;
    Eigen::Vector3d toLocal(const Eigen::Vector3d& p_world) const;

    // Pose as 12-vector? Or we use State vector in World.
};

} // namespace NexDynIPC::Dynamics
