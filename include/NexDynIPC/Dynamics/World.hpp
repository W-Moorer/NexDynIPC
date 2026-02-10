#pragma once

#include "NexDynIPC/Dynamics/RigidBody.hpp"
#include "NexDynIPC/Dynamics/Forms/Form.hpp"
#include <vector>
#include <memory>

namespace NexDynIPC::Dynamics {

class World {
public:
    std::vector<std::shared_ptr<RigidBody>> bodies;
    std::vector<std::shared_ptr<Form>> forms;
    
    void addBody(std::shared_ptr<RigidBody> body);
    void addForm(std::shared_ptr<Form> form);

    // Flatten state to vector: [x1, q1, x2, q2, ...] 
    // Rigid body state size: 3 (pos) + 4 (quat) check normalization? Or 3 (pos) + 3 (orientation in incremental step?)
    // Usually for optimization we use 3 (pos) + 3 (scaled axis angle or similar) or 12/13 DOFs depends on formulation.
    // For simplicity, let's assume we optimize for [x, theta] where theta is 3D rotation update.
    // But for State vector we might store full state.
    // Let's implement getting/setting state for *optimization variables* typically.
    
    // Total degrees of freedom for optimization
    int numDofs() const;
    
    // Get/Set current position/orientation from state vector x
    // x layout: [body0_pos(3), body0_rot(3), body1_pos(3), ...]
    // Rotations are parametrized as rotation vectors limits close to 0 ideal for Newton steps? 
    // Or we use the state directly.
    // Let's assume x contains [pos, theta] for implicit Euler update.
};

} // namespace NexDynIPC::Dynamics
