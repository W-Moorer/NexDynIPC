#pragma once

#include "NexDynIPC/Dynamics/Forms/Form.h"
#include "NexDynIPC/Dynamics/Joints/Joint.h"
#include "NexDynIPC/Dynamics/RigidBody.h"
#include <memory>
#include <limits>
#include <vector>

namespace NexDynIPC::Control {

/**
 * @brief 速度驱动能量形式
 *
 * 将速度控制目标转化为能量最小化问题，基于NexDynIPC的能量框架实现。
 *
 * 物理原理：
 * 目标速度 v_target，当前速度 v，速度增益 kv
 *
 * 能量函数（二次势）：
 *   E(v) = 0.5 * kv * (v - v_target)^2
 *
 * 能量梯度（对应阻尼力）：
 *   dE/dv = kv * (v - v_target)
 *
 * 驱动力（负梯度方向）：
 *   F_drive = -dE/dv = kv * (v_target - v)
 *
 * 这与MuJoCo的PD速度控制完全等价：
 *   MuJoCo: tau = kv * (v_target - v)
 *   NexDynIPC: F_drive = kv * (v_target - v)
 *
 * 时间离散化（隐式欧拉）：
 *   v^{n+1} = (q^{n+1} - q^n) / dt
 *   E(q^{n+1}) = 0.5 * kv * ((q^{n+1} - q^n)/dt - v_target)^2
 *
 * 对于旋转关节（HingeJoint）：
 *   控制的是绕关节轴的相对角速度 omega_axis
 */
class VelocityDriveForm : public Dynamics::Form {
public:
    /**
     * @brief 构造函数
     * @param bodyA 父刚体（nullptr表示连接到世界/静态）
     * @param bodyB 子刚体（必须非空）
     * @param axis 旋转轴（世界坐标系或本地坐标系，取决于实现）
     * @param kv 速度增益（阻尼系数）
     * @param v_target 目标速度
     */
    VelocityDriveForm(std::shared_ptr<Dynamics::RigidBody> bodyA,
                      std::shared_ptr<Dynamics::RigidBody> bodyB,
                      const Eigen::Vector3d& axis,
                      double kv,
                      double v_target);

    /**
     * @brief 析构函数
     */
    ~VelocityDriveForm() override = default;

    /**
     * @brief 设置目标速度
     * @param v_target 目标速度（角速度，rad/s）
     */
    void setTargetVelocity(double v_target);

    /**
     * @brief 获取目标速度
     * @return 目标速度（rad/s）
     */
    double getTargetVelocity() const { return v_target_; }

    /**
     * @brief 设置速度增益
     * @param kv 速度增益（阻尼系数）
     */
    void setVelocityGain(double kv);

    /**
     * @brief 获取速度增益
     * @return 速度增益
     */
    double getVelocityGain() const { return kv_; }

    /**
     * @brief 设置最大驱动力矩（饱和上限）
     * @param max_torque 最大力矩（N·m），<=0 表示不限制
     */
    void setMaxTorque(double max_torque);

    /**
     * @brief 获取最大驱动力矩
     * @return 最大力矩（N·m）
     */
    double getMaxTorque() const { return max_torque_; }

    /**
     * @brief 设置速度误差死区（delay）
     * @param delay_radps 死区宽度（rad/s），<=0 表示无死区
     */
    void setDelay(double delay_radps);

    /**
     * @brief 获取速度误差死区
     * @return 死区宽度（rad/s）
     */
    double getDelay() const { return delay_radps_; }

    /**
     * @brief 设置目标速度一阶滤波时间常数
     * @param delay_tau_seconds 时间常数（秒），<=0 表示关闭滤波延迟
     */
    void setDelayTau(double delay_tau_seconds);

    /**
     * @brief 获取目标速度一阶滤波时间常数（秒）
     */
    double getDelayTau() const { return delay_tau_seconds_; }

    /**
     * @brief 推进一次目标速度滤波状态（建议每个时间步调用一次）
     */
    void advanceControlState();

    /**
     * @brief 获取当前生效的目标速度（滤波后）
     */
    double getEffectiveTargetVelocity() const;

    /**
     * @brief 设置时间步长
     * @param dt 时间步长（用于速度计算）
     */
    void setTimeStep(double dt) { dt_ = dt; }

    /**
     * @brief 获取时间步长
     * @return 时间步长
     */
    double getTimeStep() const { return dt_; }

    /**
     * @brief 更新全局索引
     *
     * 在World构建后调用，用于确定刚体在状态向量中的位置
     * @param bodies 刚体列表
     */
    void updateGlobalIndices(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies);

    /**
     * @brief 获取当前速度误差
     * @param x 当前状态向量
     * @return 速度误差（v_target - v_current）
     */
    double getVelocityError(const Eigen::VectorXd& x) const;

    /**
     * @brief 获取当前关节速度
     * @param x 当前状态向量
     * @return 当前角速度（rad/s）
     */
    double getCurrentVelocity(const Eigen::VectorXd& x) const;

    /**
     * @brief 获取当前施加的驱动力矩
     * @param x 当前状态向量
     * @return 驱动力矩（N·m）
     */
    double getDriveTorque(const Eigen::VectorXd& x) const;

    /**
     * @brief 基于刚体角速度计算当前关节速度（不依赖状态向量历史）
     */
    double getCurrentVelocityFromBodies() const;

    /**
     * @brief 基于刚体角速度计算当前速度误差
     */
    double getVelocityErrorFromBodies() const;

    /**
     * @brief 基于刚体角速度计算当前驱动力矩
     */
    double getDriveTorqueFromBodies() const;

    /**
     * @brief 当前是否触发扭矩饱和（基于刚体角速度）
     */
    bool isTorqueSaturatedFromBodies() const;

    // Form 接口实现

    /**
     * @brief 计算速度驱动能量值
     *
     * E = 0.5 * kv * (v - v_target)^2
     *
     * @param x 状态向量
     * @return 能量值
     */
    double value(const Eigen::VectorXd& x) const override;

    /**
     * @brief 计算能量梯度
     *
     * dE/dq = dE/dv * dv/dq
     *       = kv * (v - v_target) * (1/dt)
     *       = (kv/dt) * (v - v_target)
     *
     * 注意：这是势能梯度，负梯度方向才是驱动力方向
     * F_drive = -dE/dq = (kv/dt) * (v_target - v)
     *
     * @param x 状态向量
     * @param grad 梯度向量（输出）
     */
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override;

    /**
     * @brief 计算Hessian矩阵（用于牛顿法优化）
     *
     * H = d^2E/dq^2 = kv * (dv/dq)^2 = kv / dt^2
     *
     * @param x 状态向量
     * @param triplets Hessian矩阵的非零元素（输出）
     */
    void hessian(const Eigen::VectorXd& x,
                 std::vector<Eigen::Triplet<double>>& triplets) const override;

private:
    std::shared_ptr<Dynamics::RigidBody> bodyA_;  // 父刚体（可能为nullptr）
    std::shared_ptr<Dynamics::RigidBody> bodyB_;  // 子刚体
    Eigen::Vector3d axis_;                        // 旋转轴（世界坐标系）
    double kv_;                                   // 速度增益
    double v_target_;                             // 目标速度
    double dt_;                                   // 时间步长
    double max_torque_;                           // 最大驱动力矩（N·m）
    double delay_radps_;                          // 速度误差死区（rad/s）
    double delay_tau_seconds_;                    // 目标速度一阶滤波时间常数（s）

    int global_idx_A_ = -1;  // 父刚体在状态向量中的索引（-1表示静态）
    int global_idx_B_ = -1;  // 子刚体在状态向量中的索引

    mutable Eigen::Vector3d prev_theta_A_;  // 上一时刻父刚体旋转向量（用于速度计算）
    mutable Eigen::Vector3d prev_theta_B_;  // 上一时刻子刚体旋转向量
    mutable bool has_prev_state_ = false;   // 是否有上一时刻状态
    mutable bool filter_initialized_ = false;
    mutable double filtered_target_velocity_ = 0.0;

    /**
     * @brief 计算刚体的角速度
     *
     * 基于隐式欧拉：omega = (theta - theta_prev) / dt
     *
     * @param theta 当前旋转向量
     * @param theta_prev 上一时刻旋转向量
     * @return 角速度向量
     */
    Eigen::Vector3d computeAngularVelocity(const Eigen::Vector3d& theta,
                                            const Eigen::Vector3d& theta_prev) const;

    /**
     * @brief 从状态向量中提取旋转向量
     *
     * @param x 状态向量
     * @param global_idx 刚体在状态向量中的起始索引
     * @return 旋转向量（3D）
     */
    Eigen::Vector3d extractRotationVector(const Eigen::VectorXd& x, int global_idx) const;

    /**
     * @brief 计算关节相对角速度（沿旋转轴的分量）
     *
     * @param x 状态向量
     * @return 相对角速度（rad/s）
     */
    double computeJointVelocity(const Eigen::VectorXd& x) const;

    /**
     * @brief 应用速度误差死区（delay）
     */
    double applyDelayDeadzone(double velocity_error) const;

    /**
     * @brief 计算饱和后的驱动力矩
     */
    double computeSaturatedTorque(double effective_velocity_error) const;

    /**
     * @brief 获取当前生效目标速度（含一阶滤波）
     */
    double effectiveTargetVelocity() const;

    /**
     * @brief 查找刚体在列表中的索引
     *
     * @param bodies 刚体列表
     * @param body 目标刚体
     * @return 索引（-1表示未找到）
     */
    int findBodyIndex(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies,
                      const std::shared_ptr<Dynamics::RigidBody>& body) const;
};

} // namespace NexDynIPC::Control
