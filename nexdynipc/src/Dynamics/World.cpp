#include "NexDynIPC/Dynamics/World.h"

namespace NexDynIPC::Dynamics {

void World::addBody(std::shared_ptr<RigidBody> body) {
    if (body->id == -1) {
        body->id = static_cast<int>(bodies.size());
    }
    bodies.push_back(body);
}

void World::addJoint(std::shared_ptr<Joint> joint) {
    joints.push_back(joint);
}

void World::addForm(std::shared_ptr<Form> form) {
    forms.push_back(form);
}

int World::numDofs() const {
    return static_cast<int>(bodies.size()) * 6; // 3 translation + 3 rotation
}

} // namespace NexDynIPC::Dynamics
