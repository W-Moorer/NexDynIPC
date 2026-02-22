#include "NexDynIPC/Control/VelocityDriveForm.h"
#include <algorithm>
#include <cmath>

namespace NexDynIPC::Control {

VelocityDriveForm::VelocityDriveForm(std::shared_ptr<Dynamics::RigidBody> bodyA,
                                      std::shared_ptr<Dynamics::RigidBody> bodyB,
                                      const Eigen::Vector3d& axis,
                                      double kv,
                                      double v_target)
    : bodyA_(bodyA),
      bodyB_(bodyB),
      axis_(axis.normalized()),  // 确保轴是单位向量
      kv_(kv),
      v_target_(v_target),
      dt_(0.01),  // 默认时间步长
      global_idx_A_(-1),
      global_idx_B_(-1),
      has_prev_state_(false) {
    // 子刚体必须非空
    if (!bodyB_) {
        throw std::invalid_argument("VelocityDriveForm: bodyB cannot be null");
    }
}

void VelocityDriveForm::setTargetVelocity(double v_target) {
    v_target_ = v_target;
}

void VelocityDriveForm::setVelocityGain(double kv) {
    kv_ = kv;
}

void VelocityDriveForm::updateGlobalIndices(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies) {
    global_idx_A_ = -1;
    global_idx_B_ = -1;

    // 查找父刚体索引（如果存在）
    if (bodyA_) {
        global_idx_A_ = findBodyIndex(bodies, bodyA_);
    }

    // 查找子刚体索引（必须存在）
    global_idx_B_ = findBodyIndex(bodies, bodyB_);

    // 重置上一时刻状态标记
    has_prev_state_ = false;
}

int VelocityDriveForm::findBodyIndex(const std::vector<std::shared_ptr<Dynamics::RigidBody>>& bodies,
                                      const std::shared_ptr<Dynamics::RigidBody>& body) const {
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i] == body) {
            return static_cast<int>(i * 6);  // 每个刚体6个自由度（3平移+3旋转）
        }
    }
    return -1;  // 未找到
}

double VelocityDriveForm::getVelocityError(const Eigen::VectorXd& x) const {
    double v_current = computeJointVelocity(x);
    return v_target_ - v_current;
}

double VelocityDriveForm::getCurrentVelocity(const Eigen::VectorXd& x) const {
    return computeJointVelocity(x);
}

double VelocityDriveForm::getDriveTorque(const Eigen::VectorXd& x) const {
    double v_error = getVelocityError(x);
    return kv_ * v_error;  // tau = kv * (v_target - v)
}

Eigen::Vector3d VelocityDriveForm::extractRotationVector(const Eigen::VectorXd& x, int global_idx) const {
    if (global_idx < 0 || global_idx + 5 >= x.size()) {
        return Eigen::Vector3d::Zero();
    }
    // 旋转向量位于位置向量之后（偏移3）
    return x.segment<3>(global_idx + 3);
}

Eigen::Vector3d VelocityDriveForm::computeAngularVelocity(const Eigen::Vector3d& theta,
                                                           const Eigen::Vector3d& theta_prev) const {
    if (dt_ <= 0) {
        return Eigen::Vector3d::Zero();
    }
    // 隐式欧拉：omega = (theta - theta_prev) / dt
    return (theta - theta_prev) / dt_;
}

double VelocityDriveForm::computeJointVelocity(const Eigen::VectorXd& x) const {
    // 提取当前旋转向量
    Eigen::Vector3d theta_A = extractRotationVector(x, global_idx_A_);
    Eigen::Vector3d theta_B = extractRotationVector(x, global_idx_B_);

    // 计算角速度
    Eigen::Vector3d omega_A = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_B = Eigen::Vector3d::Zero();

    if (has_prev_state_) {
        if (global_idx_A_ >= 0) {
            omega_A = computeAngularVelocity(theta_A, prev_theta_A_);
        }
        if (global_idx_B_ >= 0) {
            omega_B = computeAngularVelocity(theta_B, prev_theta_B_);
        }
    }

    // 计算相对角速度
    Eigen::Vector3d omega_relative = omega_B - omega_A;

    // 投影到旋转轴上
    double v_joint = omega_relative.dot(axis_);

    // 更新上一时刻状态（用于下一帧速度计算）
    prev_theta_A_ = theta_A;
    prev_theta_B_ = theta_B;
    has_prev_state_ = true;

    return v_joint;
}

/**
 * @brief 计算速度驱动能量值
 *
 * 能量函数：E = 0.5 * kv * (v - v_target)^2
 *
 * 这是二次势能，当 v = v_target 时能量最小（为零）
 */
double VelocityDriveForm::value(const Eigen::VectorXd& x) const {
    double v = computeJointVelocity(x);
    double dv = v - v_target_;
    return 0.5 * kv_ * dv * dv;
}

/**
 * @brief 计算能量梯度
 *
 * 数学推导：
 *   E = 0.5 * kv * (v - v_target)^2
 *   dE/dv = kv * (v - v_target)
 *
 * 对于隐式欧拉离散化：
 *   v = (q - q_prev) / dt
 *   dv/dq = 1/dt
 *
 * 链式法则：
 *   dE/dq = dE/dv * dv/dq = kv * (v - v_target) / dt
 *
 * 注意：
 *   - 这是势能的梯度
 *   - 负梯度方向 -dE/dq 才是驱动力方向
 *   - F_drive = -dE/dq = kv * (v_target - v) / dt
 *
 * 实现说明：
 *   - 对bodyA和bodyB分别计算梯度
 *   - bodyA的梯度为负（反作用力）
 *   - bodyB的梯度为正（主动力）
 */
void VelocityDriveForm::gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const {
    // 计算当前关节速度
    double v = computeJointVelocity(x);
    double dv = v - v_target_;

    // dE/dv = kv * (v - v_target)
    double dE_dv = kv_ * dv;

    // 检查时间步长有效性
    if (dt_ <= 0) {
        return;
    }

    // dv/dq = 1/dt (隐式欧拉离散化)
    double dv_dq = 1.0 / dt_;

    // dE/dq = dE/dv * dv/dq
    double dE_dq = dE_dv * dv_dq;

    // 将梯度应用到状态向量
    // 注意：这里只应用到旋转自由度（索引+3）

    // 对bodyB施加正向梯度（子刚体）
    if (global_idx_B_ >= 0 && global_idx_B_ + 5 < grad.size()) {
        int rot_idx_B = global_idx_B_ + 3;
        // 梯度方向沿旋转轴
        // dE/dtheta_B = dE/dq * axis
        grad.segment<3>(rot_idx_B) += dE_dq * axis_;
    }

    // 对bodyA施加负向梯度（父刚体，反作用力）
    if (global_idx_A_ >= 0 && global_idx_A_ + 5 < grad.size()) {
        int rot_idx_A = global_idx_A_ + 3;
        // 反作用力方向相反
        // dE/dtheta_A = -dE/dq * axis
        grad.segment<3>(rot_idx_A) -= dE_dq * axis_;
    }
}

/**
 * @brief 计算Hessian矩阵
 *
 * 数学推导：
 *   H = d^2E/dq^2 = d/dq (dE/dq) = d/dq (kv * (v - v_target) / dt)
 *     = kv/dt * dv/dq = kv/dt * (1/dt) = kv / dt^2
 *
 * Hessian矩阵元素：
 *   H_ii = kv / dt^2 （对角元素）
 *
 * 注意：
 *   - Hessian用于牛顿法优化
 *   - 这里使用高斯-牛顿近似（忽略二阶导数项）
 *   - 实际Hessian可能更复杂，但此近似在优化中通常足够
 */
void VelocityDriveForm::hessian(const Eigen::VectorXd& x,
                                 std::vector<Eigen::Triplet<double>>& triplets) const {
    (void)x;  // x在当前实现中不需要，但保留参数以符合接口

    // 检查时间步长有效性
    if (dt_ <= 0) {
        return;
    }

    // H = kv / dt^2
    double H = kv_ / (dt_ * dt_);

    // 对bodyB的Hessian贡献
    if (global_idx_B_ >= 0) {
        int rot_idx_B = global_idx_B_ + 3;
        // 对角元素
        for (int i = 0; i < 3; ++i) {
            triplets.emplace_back(rot_idx_B + i, rot_idx_B + i, H * axis_[i] * axis_[i]);
        }
        // 非对角元素（如果需要更精确的Hessian）
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                double off_diag = H * axis_[i] * axis_[j];
                triplets.emplace_back(rot_idx_B + i, rot_idx_B + j, off_diag);
                triplets.emplace_back(rot_idx_B + j, rot_idx_B + i, off_diag);
            }
        }
    }

    // 对bodyA的Hessian贡献（与bodyB相同）
    if (global_idx_A_ >= 0) {
        int rot_idx_A = global_idx_A_ + 3;
        // 对角元素
        for (int i = 0; i < 3; ++i) {
            triplets.emplace_back(rot_idx_A + i, rot_idx_A + i, H * axis_[i] * axis_[i]);
        }
        // 非对角元素
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                double off_diag = H * axis_[i] * axis_[j];
                triplets.emplace_back(rot_idx_A + i, rot_idx_A + j, off_diag);
                triplets.emplace_back(rot_idx_A + j, rot_idx_A + i, off_diag);
            }
        }
    }

    // 注意：这里忽略了bodyA和bodyB之间的耦合项
    // 在实际应用中，如果需要更精确的Hessian，应该考虑耦合
}

} // namespace NexDynIPC::Control
