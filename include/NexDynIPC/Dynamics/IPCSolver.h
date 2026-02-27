#pragma once

#include "NexDynIPC/Dynamics/TimeIntegrator.h"
#include "NexDynIPC/Math/NewtonSolver.h"
#include "NexDynIPC/TimeIntegration/ImplicitTimeIntegrator.h"
#include "NexDynIPC/Physics/Contact/CCD/CCD.h"
#include "NexDynIPC/Physics/Contact/AdaptiveBarrier.h"
#include "NexDynIPC/Physics/Contact/Distance.h"
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

    void setCCDMaxSubsteps(int max_substeps) { ccd_max_substeps_ = std::max(1, max_substeps); }
    int ccdMaxSubsteps() const { return ccd_max_substeps_; }

    void setCCDMinStepRatio(double ratio) { ccd_min_step_ratio_ = std::max(1e-4, std::min(1.0, ratio)); }
    double ccdMinStepRatio() const { return ccd_min_step_ratio_; }
    
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

    // Newton fallback controls
    void enableNewtonFallback(bool enable) { enable_newton_fallback_ = enable; }
    bool isNewtonFallbackEnabled() const { return enable_newton_fallback_; }

    void setNewtonFallbackRetries(int retries) { newton_fallback_retries_ = std::max(0, retries); }
    int newtonFallbackRetries() const { return newton_fallback_retries_; }

    void setNewtonFallbackDamping(double damping) { newton_fallback_damping_ = std::max(0.0, std::min(1.0, damping)); }
    double newtonFallbackDamping() const { return newton_fallback_damping_; }

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
    double lastCCDTOIRatio() const { return last_ccd_toi_ratio_; }
    int lastCCDSubsteps() const { return last_ccd_substeps_; }
    int lastNewtonFallbacks() const { return last_newton_fallbacks_; }
    bool lastSolverConverged() const { return last_solver_converged_; }

private:
    Math::NewtonSolver solver_;
    std::shared_ptr<TimeIntegration::ImplicitTimeIntegrator> integrator_;
    
    Physics::CCD::CCDSystem ccd_system_;
    bool enable_ccd_ = true;
    double ccd_safety_factor_ = 0.8;
    int ccd_max_substeps_ = 8;
    double ccd_min_step_ratio_ = 0.1;
    
    Physics::Contact::AdaptiveBarrier adaptive_barrier_;
    bool enable_adaptive_barrier_ = true;
    
    // Friction
    bool enable_friction_ = false;
    double friction_coeff_ = 0.0;
    std::shared_ptr<FrictionForm> friction_form_;
    std::vector<ContactPair> cached_contacts_;
    std::vector<double> cached_normal_forces_;

    // Newton fallback
    bool enable_newton_fallback_ = true;
    int newton_fallback_retries_ = 2;
    double newton_fallback_damping_ = 0.5;

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
    double last_ccd_toi_ratio_ = 1.0;
    int last_ccd_substeps_ = 1;
    int last_newton_fallbacks_ = 0;
    bool last_solver_converged_ = true;
    
    void initializeAdaptiveBarrier(World& world);
    double computeMaxStep(World& world, double dt);
    
    // Compute normal forces from barrier potential for friction
    std::vector<double> computeNormalForces(
        World& world,
        const Eigen::VectorXd& x);
    
    // Detect contact pairs for friction
    std::vector<ContactPair> detectContacts(World& world);

    // Perform one implicit solve substep
    void stepSingle(World& world, double dt);
};

} // namespace NexDynIPC::Dynamics
