// ============================================================================
// NexDynIPC AutoDiff 屏障形式
// ============================================================================
// 使用 autodiff 库自动计算梯度和 Hessian 的屏障形式
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
 * @brief 使用 autodiff 自动计算梯度和 Hessian 的屏障形式
 * 
 * 相比手动实现的屏障形式，此类使用自动微分确保导数正确性，
 * 并支持更复杂的能量函数定义。
 * 
 * @example
 * // 定义 IPC 屏障函数
 * auto ipc_barrier = [](const autodiff::VectorXdual& x) -> autodiff::dual {
 *     autodiff::dual energy = 0.0;
 *     for (int i = 0; i < x.size() / 3; ++i) {
 *         autodiff::dual dist = sqrt(x[3*i]*x[3*i] + x[3*i+1]*x[3*i+1] + x[3*i+2]*x[3*i+2]);
 *         autodiff::double dhat = 0.1;
 *         if (dist < dhat) {
 *             autodiff::double t = dist / dhat;
 *             energy += -dhat * dhat * log(t) / 3.0;
 *         }
 *     }
 *     return energy;
 * };
 * 
 * auto barrier_form = std::make_shared<AutoDiffBarrierForm>(ipc_barrier);
 * world.addForm(barrier_form);
 */
class AutoDiffBarrierForm : public Form {
public:
    /**
     * @brief 屏障函数类型定义
     * 
     * 接受 VectorXdual 类型的位置向量，返回 dual 类型的能量值
     */
    using BarrierFunction = std::function<autodiff::dual(const autodiff::VectorXdual<>&)>;
    
    /**
     * @brief 构造函数
     * 
     * @param barrier_func 屏障函数，使用 autodiff 类型定义
     * @param stiffness 屏障刚度系数（默认 1.0）
     */
    explicit AutoDiffBarrierForm(BarrierFunction barrier_func, double stiffness = 1.0)
        : barrier_func_(std::move(barrier_func)), stiffness_(stiffness) {}
    
    /**
     * @brief 计算屏障能量值
     * 
     * @param x 位置向量
     * @return double 能量值
     */
    double value(const Eigen::VectorXd& x) const override {
        autodiff::VectorXdual<> xd = x.cast<autodiff::dual>();
        autodiff::dual result = barrier_func_(xd);
        return stiffness_ * result.val;
    }
    
    /**
     * @brief 计算屏障梯度
     * 
     * @param x 位置向量
     * @param grad 输出梯度（已预分配）
     */
    void gradient(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const override {
        using namespace autodiff;
        
        VectorXdual<> xd = x.cast<dual>();
        grad = stiffness_ * autodiff::gradient(barrier_func_, wrt(xd), at(xd));
    }
    
    /**
     * @brief 计算屏障 Hessian 矩阵（稀疏格式）
     * 
     * @param x 位置向量
     * @param triplets 输出稀疏矩阵三元组列表
     */
    void hessian(const Eigen::VectorXd& x, 
                 std::vector<Eigen::Triplet<double>>& triplets) const override {
        using namespace autodiff;
        
        VectorXdual2nd<> x2 = x.cast<dual2nd>();
        Eigen::MatrixXd H = stiffness_ * autodiff::hessian(barrier_func_, wrt(x2), at(x2));
        
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
     * @brief 计算屏障 Hessian 矩阵（稠密格式）
     * 
     * @param x 位置向量
     * @return Eigen::MatrixXd Hessian 矩阵
     */
    Eigen::MatrixXd hessianDense(const Eigen::VectorXd& x) const {
        using namespace autodiff;
        
        VectorXdual2nd<> x2 = x.cast<dual2nd>();
        return stiffness_ * autodiff::hessian(barrier_func_, wrt(x2), at(x2));
    }
    
    /**
     * @brief 同时计算值、梯度和 Hessian
     * 
     * @param x 位置向量
     * @return std::tuple<double, Eigen::VectorXd, Eigen::MatrixXd> (值, 梯度, Hessian)
     */
    std::tuple<double, Eigen::VectorXd, Eigen::MatrixXd> valueGradientAndHessian(
        const Eigen::VectorXd& x) const {
        using namespace autodiff;
        
        VectorXdual2nd<> x2 = x.cast<dual2nd>();
        dual2nd y = barrier_func_(x2);
        
        double val = stiffness_ * y.val;
        Eigen::VectorXd grad = stiffness_ * gradient(barrier_func_, wrt(x2), at(x2));
        Eigen::MatrixXd H = stiffness_ * hessian(barrier_func_, wrt(x2), at(x2));
        
        return { val, grad, H };
    }
    
    /**
     * @brief 设置屏障刚度
     */
    void setStiffness(double stiffness) { stiffness_ = stiffness; }
    
    /**
     * @brief 获取屏障刚度
     */
    double getStiffness() const { return stiffness_; }

private:
    BarrierFunction barrier_func_;
    double stiffness_;
};

/**
 * @brief 预定义的 IPC 屏障形式
 * 
 * 使用 Inverse Power 屏障函数：
 * b(d) = -κ * (d - d̂)² * log(d/d̂)  当 d < d̂
 * b(d) = 0                         当 d ≥ d̂
 */
class IPCBarrierForm : public AutoDiffBarrierForm {
public:
    /**
     * @brief 构造函数
     * 
     * @param dhat 屏障距离阈值
     * @param stiffness 屏障刚度系数
     * @param power 屏障幂次（默认 1，即标准 IPC）
     */
    explicit IPCBarrierForm(double dhat = 0.1, double stiffness = 1.0, int power = 1)
        : AutoDiffBarrierForm(createIPCBarrier(dhat, power), stiffness), dhat_(dhat), power_(power) {}
    
    /**
     * @brief 获取屏障距离阈值
     */
    double getDhat() const { return dhat_; }
    
    /**
     * @brief 设置屏障距离阈值
     */
    void setDhat(double dhat) { 
        dhat_ = dhat; 
        // 重新设置屏障函数
        barrier_func_ = createIPCBarrier(dhat_, power_);
    }

private:
    double dhat_;
    int power_;
    
    /**
     * @brief 创建 IPC 屏障函数
     */
    static AutoDiffBarrierForm::BarrierFunction createIPCBarrier(double dhat, int power) {
        return [dhat, power](const autodiff::VectorXdual<>& x) -> autodiff::dual {
            autodiff::dual energy = 0.0;
            
            // 假设 x 存储的是距离值（每3个元素一个点的距离）
            for (int i = 0; i < x.size(); ++i) {
                autodiff::dual d = x[i];
                
                if (d < dhat) {
                    autodiff::dual t = d / dhat;
                    
                    if (power == 1) {
                        // 标准 IPC 屏障：-d̂² * log(t) / 3
                        energy += -dhat * dhat * log(t) / 3.0;
                    } else {
                        // 广义 IPC 屏障：-d̂^(p+1) * t^p * log(t) / p
                        double p = static_cast<double>(power);
                        energy += -pow(dhat, p + 1) * pow(t, p) * log(t) / p;
                    }
                }
            }
            
            return energy;
        };
    }
};

/**
 * @brief 弹簧屏障形式
 * 
 * 使用二次弹簧势能：E = 0.5 * k * (d - d₀)²
 */
class SpringBarrierForm : public AutoDiffBarrierForm {
public:
    /**
     * @brief 构造函数
     * 
     * @param rest_length 弹簧静息长度
     * @param stiffness 弹簧刚度系数
     */
    explicit SpringBarrierForm(double rest_length = 1.0, double stiffness = 100.0)
        : AutoDiffBarrierForm(createSpringBarrier(rest_length), stiffness), 
          rest_length_(rest_length) {}
    
    /**
     * @brief 获取静息长度
     */
    double getRestLength() const { return rest_length_; }

private:
    double rest_length_;
    
    /**
     * @brief 创建弹簧屏障函数
     */
    static AutoDiffBarrierForm::BarrierFunction createSpringBarrier(double rest_length) {
        return [rest_length](const autodiff::VectorXdual<>& x) -> autodiff::dual {
            autodiff::dual energy = 0.0;
            
            // 计算每对点之间的距离
            for (int i = 0; i < x.size() / 6; ++i) {
                int i1 = 6 * i;
                int i2 = 6 * i + 3;
                
                autodiff::dual dx = x[i1] - x[i2];
                autodiff::dual dy = x[i1 + 1] - x[i2 + 1];
                autodiff::dual dz = x[i1 + 2] - x[i2 + 2];
                
                autodiff::dual dist = sqrt(dx*dx + dy*dy + dz*dz);
                autodiff::dual stretch = dist - rest_length;
                
                energy += 0.5 * stretch * stretch;
            }
            
            return energy;
        };
    }
};

} // namespace NexDynIPC::Dynamics

// ============================================================================
// 使用示例
// ============================================================================
/*
 * // 示例 1: 使用自定义屏障函数
 * auto my_barrier = [](const autodiff::VectorXdual& x) -> autodiff::dual {
 *     autodiff::dual energy = 0.0;
 *     for (int i = 0; i < x.size(); ++i) {
 *         energy += x[i] * x[i];  // 二次势能
 *     }
 *     return energy;
 * };
 * 
 * auto form = std::make_shared<AutoDiffBarrierForm>(my_barrier, 1.0);
 * world.addForm(form);
 * 
 * // 示例 2: 使用预定义的 IPC 屏障
 * auto ipc_form = std::make_shared<IPCBarrierForm>(0.1, 1000.0);
 * world.addForm(ipc_form);
 * 
 * // 示例 3: 使用弹簧屏障
 * auto spring_form = std::make_shared<SpringBarrierForm>(1.0, 100.0);
 * world.addForm(spring_form);
 */
// ============================================================================
