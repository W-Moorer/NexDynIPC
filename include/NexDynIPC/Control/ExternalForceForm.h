#pragma once

#include "NexDynIPC/Dynamics/Forms/Form.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include <memory>
#include <vector>

namespace NexDynIPC::Control {

/**
 * @brief 外力形式：在刚体上施加外部力/力矩
 * 
 * 物理原理：
 * - 力 F 产生势能 E = -F·x
 * - 梯度 ∇E = -F （负梯度方向为力的方向）
 * 
 * 使用方式：
 * - 施加恒定力：setForce()
 * - 施加时变力：通过回调函数
 * - 施加力矩：setTorque()
 */
class ExternalForceForm : public Dynamics::Form {
public:
    struct ForceEntry {
        std::shared_ptr<Dynamics::RigidBody> body;
        Eigen::Vector3d force = Eigen::Vector3d::Zero();      // 世界坐标系下的力
        Eigen::Vector3d torque = Eigen::Vector3d::Zero();     // 世界坐标系下的力矩
        Eigen::Vector3d local_point = Eigen::Vector3d::Zero(); // 作用点（本地坐标系，质心为原点）
        int global_idx = -1;  // 在全局状态向量中的索引
    };

    ExternalForceForm() = default;
    ~ExternalForceForm() override = default;

    /**
     * @brief 添加刚体到力控制系统
     */
    void addBody(std::shared_ptr<Dynamics::RigidBody> body);

    /**
     * @brief 移除刚体
     */
    void removeBody(std::shared_ptr<Dynamics::RigidBody> body);

    /**
     * @brief 在刚体上施加力（世界坐标系）
     * @param body 目标刚体
     * @param force 力向量（世界坐标系）
     * @param local_point 作用点（本地坐标系，默认为质心）
     */
    void setForce(std::shared_ptr<Dynamics::RigidBody> body, 
                  const Eigen::Vector3d& force,
                  const Eigen::Vector3d& local_point = Eigen::Vector3d::Zero());

    /**
     * @brief 在刚体上施加力矩（世界坐标系）
     */
    void setTorque(std::shared_ptr<Dynamics::RigidBody> body, 
                   const Eigen::Vector3d& torque);

    /**
     * @brief 同时施加力和力矩
     */
    void setForceAndTorque(std::shared_ptr<Dynamics::RigidBody> body,
                           const Eigen::Vector3d& force,
                           const Eigen::Vector3d& torque,
                           const Eigen::Vector3d& local_point = Eigen::Vector3d::Zero());

    /**
     * @brief 清除指定刚体的所有外力
     */
    void clearForces(std::shared_ptr<Dynamics::RigidBody> body);

    /**
     * @brief 清除所有外力
     */
    void clearAllForces();

    /**
     * @brief 更新全局索引（World构建后调用）
     */
    void updateGlobalIndices(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies);

    // Form 接口实现
    double value(const Eigen::VectorXd& x) const override;
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override;
    void hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) const override;

    /**
     * @brief 获取指定刚体的当前力
     */
    Eigen::Vector3d getForce(std::shared_ptr<Dynamics::RigidBody> body) const;

    /**
     * @brief 获取指定刚体的当前力矩
     */
    Eigen::Vector3d getTorque(std::shared_ptr<Dynamics::RigidBody> body) const;

private:
    std::vector<ForceEntry> entries_;

    // 查找刚体对应的条目
    ForceEntry* findEntry(std::shared_ptr<Dynamics::RigidBody> body);
    const ForceEntry* findEntry(std::shared_ptr<Dynamics::RigidBody> body) const;
};

} // namespace NexDynIPC::Control
