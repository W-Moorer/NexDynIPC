#include "NexDynIPC/Control/ExternalForceForm.h"
#include <algorithm>

namespace NexDynIPC::Control {

void ExternalForceForm::addBody(std::shared_ptr<Dynamics::RigidBody> body) {
    if (!body) return;
    
    // 检查是否已存在
    if (findEntry(body) != nullptr) return;
    
    ForceEntry entry;
    entry.body = body;
    entries_.push_back(entry);
}

void ExternalForceForm::removeBody(std::shared_ptr<Dynamics::RigidBody> body) {
    if (!body) return;
    
    auto it = std::remove_if(entries_.begin(), entries_.end(),
        [&body](const ForceEntry& entry) {
            return entry.body == body;
        });
    entries_.erase(it, entries_.end());
}

void ExternalForceForm::setForce(std::shared_ptr<Dynamics::RigidBody> body, 
                                 const Eigen::Vector3d& force,
                                 const Eigen::Vector3d& local_point) {
    auto entry = findEntry(body);
    if (!entry) {
        addBody(body);
        entry = findEntry(body);
    }
    
    if (entry) {
        entry->force = force;
        entry->local_point = local_point;
    }
}

void ExternalForceForm::setTorque(std::shared_ptr<Dynamics::RigidBody> body, 
                                  const Eigen::Vector3d& torque) {
    auto entry = findEntry(body);
    if (!entry) {
        addBody(body);
        entry = findEntry(body);
    }
    
    if (entry) {
        entry->torque = torque;
    }
}

void ExternalForceForm::setForceAndTorque(std::shared_ptr<Dynamics::RigidBody> body,
                                          const Eigen::Vector3d& force,
                                          const Eigen::Vector3d& torque,
                                          const Eigen::Vector3d& local_point) {
    auto entry = findEntry(body);
    if (!entry) {
        addBody(body);
        entry = findEntry(body);
    }
    
    if (entry) {
        entry->force = force;
        entry->torque = torque;
        entry->local_point = local_point;
    }
}

void ExternalForceForm::clearForces(std::shared_ptr<Dynamics::RigidBody> body) {
    auto entry = findEntry(body);
    if (entry) {
        entry->force.setZero();
        entry->torque.setZero();
    }
}

void ExternalForceForm::clearAllForces() {
    for (auto& entry : entries_) {
        entry.force.setZero();
        entry.torque.setZero();
    }
}

void ExternalForceForm::updateGlobalIndices(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies) {
    for (auto& entry : entries_) {
        entry.global_idx = -1;
        
        // 在bodies列表中查找索引
        for (size_t i = 0; i < bodies.size(); ++i) {
            if (bodies[i] == entry.body) {
                entry.global_idx = static_cast<int>(i * 6);  // 每个刚体6个自由度
                break;
            }
        }
    }
}

double ExternalForceForm::value(const Eigen::VectorXd& x) const {
    (void)x;  // 外力势能 E = -F·x，但在优化中通常不需要显式计算
    return 0.0;
}

void ExternalForceForm::gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    (void)x;  // x 在当前实现中不需要，但保留参数以符合接口
    
    for (const auto& entry : entries_) {
        if (entry.global_idx < 0 || !entry.body) continue;
        
        int idx = entry.global_idx;
        
        // 力的梯度：∇E = -F
        // 注意：grad 是势能的梯度，负梯度方向为力的方向
        // 所以我们直接将 -force 加到 grad 上
        grad.segment<3>(idx) -= entry.force;
        
        // 力矩的梯度（对于旋转向量参数化）
        grad.segment<3>(idx + 3) -= entry.torque;
    }
}

void ExternalForceForm::hessian(const Eigen::VectorXd& x, 
                                std::vector<Eigen::Triplet<double>>& triplets) const {
    (void)x;
    (void)triplets;
    // 外力形式的 Hessian 为零（线性项）
}

Eigen::Vector3d ExternalForceForm::getForce(std::shared_ptr<Dynamics::RigidBody> body) const {
    auto entry = findEntry(body);
    if (entry) {
        return entry->force;
    }
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d ExternalForceForm::getTorque(std::shared_ptr<Dynamics::RigidBody> body) const {
    auto entry = findEntry(body);
    if (entry) {
        return entry->torque;
    }
    return Eigen::Vector3d::Zero();
}

ExternalForceForm::ForceEntry* ExternalForceForm::findEntry(std::shared_ptr<Dynamics::RigidBody> body) {
    for (auto& entry : entries_) {
        if (entry.body == body) {
            return &entry;
        }
    }
    return nullptr;
}

const ExternalForceForm::ForceEntry* ExternalForceForm::findEntry(std::shared_ptr<Dynamics::RigidBody> body) const {
    for (const auto& entry : entries_) {
        if (entry.body == body) {
            return &entry;
        }
    }
    return nullptr;
}

} // namespace NexDynIPC::Control
