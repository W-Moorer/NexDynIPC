// ============================================================================
// NexDynIPC AutoDiff 约束形式
// ============================================================================
// 使用 autodiff 库自动计算约束雅可比矩阵的约束形式
// ============================================================================

#pragma once

#include "NexDynIPC/Dynamics/Forms/Form.h"
#include "NexDynIPC/Math/AutoDiffExtended.h"

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <functional>
#include <vector>

namespace NexDynIPC::Dynamics {

/**
 * @brief 使用 autodiff 自动计算雅可比矩阵的约束形式
 * 
 * 支持等式约束和不等式约束，自动计算约束雅可比矩阵。
 * 
 * @example
 * // 定义距离约束：两点间距离等于目标值
 * auto distance_constraint = [](const autodiff::VectorXvar& x) -> autodiff::VectorXvar {
 *     autodiff::VectorXvar constraints(1);
 *     
 *     // 提取两个点的位置
 *     autodiff::var dx = x[0] - x[3];
 *     autodiff::var dy = x[1] - x[4];
 *     autodiff::var dz = x[2] - x[5];
 *     
 *     // 距离约束：||p1 - p2|| = 1.0
 *     autodiff::var dist = sqrt(dx*dx + dy*dy + dz*dz);
 *     constraints[0] = dist - 1.0;
 *     
 *     return constraints;
 * };
 * 
 * auto constraint_form = std::make_shared<AutoDiffConstraintForm>(
 *     distance_constraint, 1, AutoDiffConstraintForm::Equality);
 * world.addForm(constraint_form);
 */
class AutoDiffConstraintForm : public Form {
public:
    /**
     * @brief 约束类型
     */
    enum ConstraintType {
        Equality,       // 等式约束: c(x) = 0
        InequalityLEQ,  // 不等式约束: c(x) <= 0
        InequalityGEQ   // 不等式约束: c(x) >= 0
    };
    
    /**
     * @brief 约束函数类型定义
     * 
     * 接受 VectorXvar 类型的状态向量，返回 VectorXvar 类型的约束值
     */
    using ConstraintFunction = std::function<autodiff::VectorXvar<>(const autodiff::VectorXvar<>&)>;
    
    /**
     * @brief 构造函数
     * 
     * @param constraint_func 约束函数
     * @param num_constraints 约束数量
     * @param type 约束类型
     * @param stiffness 约束刚度（用于惩罚项）
     */
    AutoDiffConstraintForm(ConstraintFunction constraint_func, 
                           int num_constraints,
                           ConstraintType type = Equality,
                           double stiffness = 1e6)
        : constraint_func_(std::move(constraint_func))
        , num_constraints_(num_constraints)
        , type_(type)
        , stiffness_(stiffness) {}
    
    /**
     * @brief 计算约束违反能量
     * 
     * 使用二次惩罚项：E = 0.5 * κ * ||c(x)||²
     */
    double value(const Eigen::VectorXd& x) const override {
        autodiff::VectorXvar<> xv = toVectorVar(x);
        autodiff::VectorXvar<> c = constraint_func_(xv);
        
        double energy = 0.0;
        for (int i = 0; i < c.size(); ++i) {
            double ci = c[i].val;
            
            switch (type_) {
                case Equality:
                    // E = 0.5 * κ * c²
                    energy += 0.5 * stiffness_ * ci * ci;
                    break;
                case InequalityLEQ:
                    // E = 0.5 * κ * max(0, c)²  (如果 c > 0，即违反约束)
                    if (ci > 0) {
                        energy += 0.5 * stiffness_ * ci * ci;
                    }
                    break;
                case InequalityGEQ:
                    // E = 0.5 * κ * max(0, -c)²  (如果 c < 0，即违反约束)
                    if (ci < 0) {
                        energy += 0.5 * stiffness_ * ci * ci;
                    }
                    break;
            }
        }
        
        return energy;
    }
    
    /**
     * @brief 计算约束能量梯度
     */
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override {
        using namespace autodiff;
        
        VectorXvar<> xv = toVectorVar(x);
        VectorXvar<> c = constraint_func_(xv);
        
        grad.setZero(x.size());
        
        // 对每个约束计算梯度并累加
        for (int i = 0; i < c.size(); ++i) {
            double ci = c[i].val;
            
            // 检查约束是否激活
            bool active = false;
            switch (type_) {
                case Equality:
                    active = true;
                    break;
                case InequalityLEQ:
                    active = (ci > 0);
                    break;
                case InequalityGEQ:
                    active = (ci < 0);
                    break;
            }
            
            if (active) {
                // 计算第 i 个约束的梯度
                // ∇E = κ * c * ∇c
                double coeff = stiffness_ * ci;
                
                // 反向传播计算 ∇c
                c[i].seed(1.0);
                autodiff::derivatives(c[i], wrt(xv));
                
                // 累加梯度
                for (int j = 0; j < x.size(); ++j) {
                    grad[j] += coeff * xv[j].grad;
                }
            }
        }
    }
    
    /**
     * @brief 计算约束能量 Hessian（近似）
     */
    void hessian(const Eigen::VectorXd& x, 
                 std::vector<Eigen::Triplet<double>>& triplets) const override {
        // 使用高斯-牛顿近似：H ≈ κ * JᵀJ
        Eigen::MatrixXd J = constraintJacobian(x);
        
        Eigen::MatrixXd H = stiffness_ * J.transpose() * J;
        
        // 转换为稀疏格式
        for (int i = 0; i < H.rows(); ++i) {
            for (int j = 0; j < H.cols(); ++j) {
                if (std::abs(H(i, j)) > 1e-12) {
                    triplets.emplace_back(i, j, H(i, j));
                }
            }
        }
    }
    
    /**
     * @brief 计算约束雅可比矩阵
     * 
     * @param x 状态向量
     * @return Eigen::MatrixXd 雅可比矩阵 (num_constraints × x.size())
     */
    Eigen::MatrixXd constraintJacobian(const Eigen::VectorXd& x) const {
        using namespace autodiff;
        
        VectorXvar<> xv = toVectorVar(x);
        VectorXvar<> c = constraint_func_(xv);
        
        return jacobian(c, xv);
    }
    
    /**
     * @brief 计算约束值
     * 
     * @param x 状态向量
     * @return Eigen::VectorXd 约束值
     */
    Eigen::VectorXd constraintValues(const Eigen::VectorXd& x) const {
        autodiff::VectorXvar<> xv = toVectorVar(x);
        autodiff::VectorXvar<> c = constraint_func_(xv);
        
        Eigen::VectorXd values(c.size());
        for (int i = 0; i < c.size(); ++i) {
            values[i] = c[i].val;
        }
        return values;
    }
    
    /**
     * @brief 检查约束是否满足
     * 
     * @param x 状态向量
     * @param tolerance 容差
     * @return bool 是否满足
     */
    bool isSatisfied(const Eigen::VectorXd& x, double tolerance = 1e-6) const {
        Eigen::VectorXd c = constraintValues(x);
        
        for (int i = 0; i < c.size(); ++i) {
            switch (type_) {
                case Equality:
                    if (std::abs(c[i]) > tolerance) return false;
                    break;
                case InequalityLEQ:
                    if (c[i] > tolerance) return false;
                    break;
                case InequalityGEQ:
                    if (c[i] < -tolerance) return false;
                    break;
            }
        }
        return true;
    }
    
    /**
     * @brief 获取约束违反度
     */
    double constraintViolation(const Eigen::VectorXd& x) const {
        Eigen::VectorXd c = constraintValues(x);
        double violation = 0.0;
        
        for (int i = 0; i < c.size(); ++i) {
            switch (type_) {
                case Equality:
                    violation += std::abs(c[i]);
                    break;
                case InequalityLEQ:
                    violation += std::max(0.0, c[i]);
                    break;
                case InequalityGEQ:
                    violation += std::max(0.0, -c[i]);
                    break;
            }
        }
        
        return violation;
    }
    
    /**
     * @brief 设置约束刚度
     */
    void setStiffness(double stiffness) { stiffness_ = stiffness; }
    
    /**
     * @brief 获取约束刚度
     */
    double getStiffness() const { return stiffness_; }
    
    /**
     * @brief 获取约束数量
     */
    int getNumConstraints() const { return num_constraints_; }
    
    /**
     * @brief 获取约束类型
     */
    ConstraintType getType() const { return type_; }

private:
    ConstraintFunction constraint_func_;
    int num_constraints_;
    ConstraintType type_;
    double stiffness_;
    
    /**
     * @brief 将 Eigen::VectorXd 转换为 VectorXvar
     */
    static autodiff::VectorXvar<> toVectorVar(const Eigen::VectorXd& x) {
        autodiff::VectorXvar<> result(x.size());
        for (int i = 0; i < x.size(); ++i) {
            result[i] = autodiff::var(x[i]);
        }
        return result;
    }
};

/**
 * @brief 距离约束形式
 * 
 * 约束两点间距离等于目标值
 */
class DistanceConstraintForm : public AutoDiffConstraintForm {
public:
    /**
     * @brief 构造函数
     * 
     * @param idx1 第一个点的索引（在状态向量中的起始位置）
     * @param idx2 第二个点的索引
     * @param target_distance 目标距离
     * @param stiffness 约束刚度
     */
    DistanceConstraintForm(int idx1, int idx2, double target_distance, double stiffness = 1e6)
        : AutoDiffConstraintForm(
            createDistanceConstraint(idx1, idx2, target_distance),
            1,
            Equality,
            stiffness
        ), idx1_(idx1), idx2_(idx2), target_distance_(target_distance) {}
    
    /**
     * @brief 获取目标距离
     */
    double getTargetDistance() const { return target_distance_; }
    
    /**
     * @brief 设置目标距离
     */
    void setTargetDistance(double dist) { target_distance_ = dist; }

private:
    int idx1_, idx2_;
    double target_distance_;
    
    static ConstraintFunction createDistanceConstraint(int idx1, int idx2, double target) {
        return [idx1, idx2, target](const autodiff::VectorXvar<>& x) -> autodiff::VectorXvar<> {
            autodiff::VectorXvar<> c(1);
            
            autodiff::var dx = x[idx1] - x[idx2];
            autodiff::var dy = x[idx1 + 1] - x[idx2 + 1];
            autodiff::var dz = x[idx1 + 2] - x[idx2 + 2];
            
            autodiff::var dist = sqrt(dx*dx + dy*dy + dz*dz);
            c[0] = dist - target;
            
            return c;
        };
    }
};

/**
 * @brief 固定点约束形式
 * 
 * 约束某点固定在目标位置
 */
class FixedPointConstraintForm : public AutoDiffConstraintForm {
public:
    /**
     * @brief 构造函数
     * 
     * @param idx 点的索引（在状态向量中的起始位置）
     * @param target_position 目标位置
     * @param stiffness 约束刚度
     */
    FixedPointConstraintForm(int idx, const Eigen::Vector3d& target_position, double stiffness = 1e8)
        : AutoDiffConstraintForm(
            createFixedPointConstraint(idx, target_position),
            3,
            Equality,
            stiffness
        ), idx_(idx), target_position_(target_position) {}

private:
    int idx_;
    Eigen::Vector3d target_position_;
    
    static ConstraintFunction createFixedPointConstraint(int idx, const Eigen::Vector3d& target) {
        return [idx, target](const autodiff::VectorXvar<>& x) -> autodiff::VectorXvar<> {
            autodiff::VectorXvar<> c(3);
            c[0] = x[idx] - target[0];
            c[1] = x[idx + 1] - target[1];
            c[2] = x[idx + 2] - target[2];
            return c;
        };
    }
};

/**
 * @brief 平面约束形式
 * 
 * 约束某点位于平面上
 */
class PlaneConstraintForm : public AutoDiffConstraintForm {
public:
    /**
     * @brief 构造函数
     * 
     * @param idx 点的索引
     * @param normal 平面法向量
     * @param offset 平面偏移量（平面方程：n·x + d = 0）
     * @param stiffness 约束刚度
     */
    PlaneConstraintForm(int idx, const Eigen::Vector3d& normal, double offset, double stiffness = 1e6)
        : AutoDiffConstraintForm(
            createPlaneConstraint(idx, normal, offset),
            1,
            Equality,
            stiffness
        ), idx_(idx), normal_(normal), offset_(offset) {}

private:
    int idx_;
    Eigen::Vector3d normal_;
    double offset_;
    
    static ConstraintFunction createPlaneConstraint(int idx, const Eigen::Vector3d& normal, double offset) {
        return [idx, normal, offset](const autodiff::VectorXvar<>& x) -> autodiff::VectorXvar<> {
            autodiff::VectorXvar<> c(1);
            
            // 平面约束：n·p + d = 0
            autodiff::var dot = normal[0] * x[idx] + 
                               normal[1] * x[idx + 1] + 
                               normal[2] * x[idx + 2];
            c[0] = dot + offset;
            
            return c;
        };
    }
};

} // namespace NexDynIPC::Dynamics

// ============================================================================
// 使用示例
// ============================================================================
/*
 * // 示例 1: 距离约束
 * auto dist_constraint = std::make_shared<DistanceConstraintForm>(
 *     0,    // 第一个点从索引 0 开始
 *     3,    // 第二个点从索引 3 开始
 *     1.0,  // 目标距离 1.0
 *     1e6   // 刚度
 * );
 * world.addForm(dist_constraint);
 * 
 * // 示例 2: 固定点约束
 * Eigen::Vector3d target(0, 0, 0);
 * auto fixed_constraint = std::make_shared<FixedPointConstraintForm>(
 *     0,      // 固定第一个点
 *     target, // 目标位置
 *     1e8     // 高刚度确保严格固定
 * );
 * world.addForm(fixed_constraint);
 * 
 * // 示例 3: 自定义约束函数
 * auto custom_constraint = [](const autodiff::VectorXvar& x) -> autodiff::VectorXvar {
 *     autodiff::VectorXvar c(2);
 *     c[0] = x[0] + x[1] - 1.0;  // x0 + x1 = 1
 *     c[1] = x[0] - x[1];        // x0 = x1
 *     return c;
 * };
 * 
 * auto constraint_form = std::make_shared<AutoDiffConstraintForm>(
 *     custom_constraint,
 *     2,
 *     AutoDiffConstraintForm::Equality,
 *     1e6
 * );
 * world.addForm(constraint_form);
 * 
 * // 示例 4: 检查约束满足情况
 * Eigen::VectorXd x = ...;
 * bool satisfied = constraint_form->isSatisfied(x, 1e-6);
 * double violation = constraint_form->constraintViolation(x);
 * Eigen::MatrixXd J = constraint_form->constraintJacobian(x);
 */
// ============================================================================
