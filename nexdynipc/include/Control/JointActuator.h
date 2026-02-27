#pragma once

#include "NexDynIPC/Dynamics/Joints/Joint.h"
#include <memory>
#include <functional>

namespace NexDynIPC::Control {

/**
 * @brief 关节驱动器：在关节上施加驱动力矩/力
 * 
 * 支持多种控制模式：
 * - 力/力矩控制：直接设置目标力矩
 * - 位置控制（PD）：通过力矩实现位置跟踪
 * - 速度控制：通过力矩实现速度跟踪
 * 
 * 物理原理：
 * - 对于旋转关节：施加力矩 τ
 * - 对于移动关节：施加力 F
 * - 通过关节约束的拉格朗日乘子实现
 */
class JointActuator {
public:
    enum class ControlMode {
        TORQUE_CONTROL,    // 直接力矩控制
        POSITION_CONTROL,  // PD位置控制
        VELOCITY_CONTROL   // PD速度控制
    };

    JointActuator(std::shared_ptr<Dynamics::Joint> joint);
    ~JointActuator() = default;

    /**
     * @brief 设置控制模式
     */
    void setControlMode(ControlMode mode);
    ControlMode getControlMode() const { return mode_; }

    /**
     * @brief 力矩控制模式：设置目标力矩/力
     * @param torque 对于旋转关节是力矩，对于移动关节是力
     */
    void setTargetTorque(double torque);
    double getTargetTorque() const { return target_torque_; }

    /**
     * @brief 位置控制模式：设置目标位置（关节坐标）
     */
    void setTargetPosition(double position);
    double getTargetPosition() const { return target_position_; }

    /**
     * @brief 速度控制模式：设置目标速度
     */
    void setTargetVelocity(double velocity);
    double getTargetVelocity() const { return target_velocity_; }

    /**
     * @brief 设置PD控制参数
     */
    void setPDGains(double kp, double kd);
    double getKp() const { return kp_; }
    double getKd() const { return kd_; }

    /**
     * @brief 设置力/力矩限制（饱和）
     */
    void setTorqueLimits(double min_torque, double max_torque);
    double getMinTorque() const { return min_torque_; }
    double getMaxTorque() const { return max_torque_; }

    /**
     * @brief 获取当前关节位置
     */
    double getCurrentPosition() const;

    /**
     * @brief 获取当前关节速度
     */
    double getCurrentVelocity() const;

    /**
     * @brief 计算并应用控制输出
     * 根据控制模式计算力矩，并应用到关节
     * @return 实际应用的力矩/力
     */
    double computeControlOutput();

    /**
     * @brief 获取关联的关节
     */
    std::shared_ptr<Dynamics::Joint> getJoint() const { return joint_; }

    /**
     * @brief 启用/禁用驱动器
     */
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }

private:
    std::shared_ptr<Dynamics::Joint> joint_;
    ControlMode mode_ = ControlMode::TORQUE_CONTROL;
    
    // 目标值
    double target_torque_ = 0.0;
    double target_position_ = 0.0;
    double target_velocity_ = 0.0;
    
    // PD参数
    double kp_ = 100.0;  // 位置增益
    double kd_ = 10.0;   // 速度增益
    
    // 力矩限制
    double min_torque_ = -std::numeric_limits<double>::max();
    double max_torque_ = std::numeric_limits<double>::max();
    
    // 状态
    bool enabled_ = true;
    double last_position_ = 0.0;
    double last_velocity_ = 0.0;
    
    // 辅助函数
    double saturateTorque(double torque) const;
};

/**
 * @brief 关节驱动器管理器：管理多个关节驱动器
 */
class JointActuatorManager {
public:
    JointActuatorManager() = default;
    ~JointActuatorManager() = default;

    /**
     * @brief 为关节创建驱动器
     */
    std::shared_ptr<JointActuator> createActuator(std::shared_ptr<Dynamics::Joint> joint);

    /**
     * @brief 移除关节驱动器
     */
    void removeActuator(std::shared_ptr<Dynamics::Joint> joint);

    /**
     * @brief 获取关节驱动器
     */
    std::shared_ptr<JointActuator> getActuator(std::shared_ptr<Dynamics::Joint> joint) const;

    /**
     * @brief 更新所有驱动器（计算并应用控制）
     */
    void updateAllActuators();

    /**
     * @brief 清除所有驱动器
     */
    void clear();

    /**
     * @brief 获取驱动器数量
     */
    size_t getActuatorCount() const { return actuators_.size(); }

private:
    std::vector<std::shared_ptr<JointActuator>> actuators_;
};

} // namespace NexDynIPC::Control
