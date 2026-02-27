#pragma once

#include "NexDynIPC/Dynamics/TimeIntegrator.h"
#include "NexDynIPC/Math/NewtonSolver.h"
#include "NexDynIPC/TimeIntegration/ImplicitTimeIntegrator.h"
#include "NexDynIPC/Physics/Contact/CCD/CCD.h"
#include "NexDynIPC/Physics/Contact/AdaptiveBarrier.h"
#include "NexDynIPC/Dynamics/Forms/FrictionForm.h"
#include <memory>
#include <algorithm>

namespace NexDynIPC::Dynamics {

class IPCSolver : public TimeIntegrator {
public:
    IPCSolver();
    
    void setIntegrator(std::shared_ptr<TimeIntegration::ImplicitTimeIntegrator> integrator);
    
    void step(World& world, double dt) override;
    
    void enableCCD(bool enable) { enable_ccd_ = enable; }
    bool isCCDEnabled() const { return enable_ccd_; }
    
    void setCCDSafetyFactor(double factor) { ccd_safety_factor_ = factor; }
    double ccdSafetyFactor() const { return ccd_safety_factor_; }
    
    void enableAdaptiveBarrier(bool enable) { enable_adaptive_barrier_ = enable; }
    bool isAdaptiveBarrierEnabled() const { return enable_adaptive_barrier_; }
    
    Physics::Contact::AdaptiveBarrier& adaptiveBarrier() { return adaptive_barrier_; }
    const Physics::Contact::AdaptiveBarrier& adaptiveBarrier() const { return adaptive_barrier_; }
    
    // Friction support
    void enableFriction(bool enable) { enable_friction_ = enable; }
    bool isFrictionEnabled() const { return enable_friction_; }
    
    void setFrictionCoefficient(double mu) { friction_coeff_ = mu; }
    double frictionCoefficient() const { return friction_coeff_; }
    
    FrictionForm* frictionForm() { return friction_form_.get(); }

    // ALM controls
    void setALMMaxIters(int max_iters) { alm_max_iters_ = std::max(1, max_iters); }
    void setALMConstraintTolerance(double tol) { alm_constraint_tolerance_ = std::max(1e-12, tol); }
    void setALMDualTolerance(double tol) { alm_dual_tolerance_ = std::max(1e-12, tol); }
    void setALMHardeningTrigger(double trigger) { alm_hardening_trigger_ = std::max(1.0, trigger); }
    void setALMHardeningRatio(double ratio) { alm_hardening_ratio_ = std::max(1.0, ratio); }

    // Last-step diagnostics / KPI
    double lastAngularVelocityError() const { return last_angular_velocity_error_; }
    double lastTorqueSaturationRatio() const { return last_torque_saturation_ratio_; }
    double lastDualResidual() const { return last_dual_residual_; }
    double lastMaxConstraintViolation() const { return last_max_constraint_violation_; }

private:
    Math::NewtonSolver solver_;
    std::shared_ptr<TimeIntegration::ImplicitTimeIntegrator> integrator_;
    
    Physics::CCD::CCDSystem ccd_system_;
    bool enable_ccd_ = true;
    double ccd_safety_factor_ = 0.8;
    
    Physics::Contact::AdaptiveBarrier adaptive_barrier_;
    bool enable_adaptive_barrier_ = true;
    
    // Friction
    bool enable_friction_ = false;
    double friction_coeff_ = 0.0;
    std::shared_ptr<FrictionForm> friction_form_;

    // ALM runtime options
    int alm_max_iters_ = 10;
    double alm_constraint_tolerance_ = 1e-4;
    double alm_dual_tolerance_ = 1e-3;
    double alm_hardening_trigger_ = 2.0;
    double alm_hardening_ratio_ = 2.0;

    // Last-step KPI cache
    double last_angular_velocity_error_ = 0.0;
    double last_torque_saturation_ratio_ = 0.0;
    double last_dual_residual_ = 0.0;
    double last_max_constraint_violation_ = 0.0;
    
    void initializeAdaptiveBarrier(World& world);
    double computeMaxStep(World& world, double dt);
    
    // Compute normal forces from barrier potential for friction
    std::vector<double> computeNormalForces(
        World& world,
        const Eigen::VectorXd& x);
    
    // Detect contact pairs for friction
    std::vector<ContactPair> detectContacts(World& world);
};

} // namespace NexDynIPC::Dynamics
