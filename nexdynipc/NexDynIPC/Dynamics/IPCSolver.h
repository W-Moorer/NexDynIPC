#pragma once

#include "NexDynIPC/Dynamics/TimeIntegrator.h"
#include "NexDynIPC/Math/NewtonSolver.h"
#include "NexDynIPC/TimeIntegration/ImplicitTimeIntegrator.h"
#include "NexDynIPC/Physics/Contact/CCD/CCD.h"
#include "NexDynIPC/Physics/Contact/AdaptiveBarrier.h"
#include "NexDynIPC/Physics/Contact/BroadPhase.h"
#include "NexDynIPC/Physics/Contact/ContactAssembler.h"
#include "NexDynIPC/Dynamics/Forms/FrictionForm.h"
#include "NexDynIPC/Dynamics/Forms/DistanceBarrierForm.h"
#include <memory>
#include <algorithm>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace NexDynIPC::Dynamics {

class IPCSolver : public TimeIntegrator {
public:
    struct ContactResidualSample {
        int alm_iteration = 0;
        int newton_iteration = 0;
        double contact_residual = 0.0;
        double line_search_alpha = 0.0;
        double contact_value_before = 0.0;
        double contact_value_after = 0.0;
    };

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

    void setContactDistanceThreshold(double dhat) { adaptive_barrier_.setDhat(std::max(1e-9, dhat)); }
    void setContactStiffness(double stiffness) {
        if (stiffness > 0.0) {
            adaptive_barrier_.setStiffness(stiffness);
        }
    }

    void clearAllowedContactPairs() {
        allowed_contact_pair_keys_.clear();
        pair_friction_overrides_.clear();
    }

    void addAllowedContactPair(int body_id_a, int body_id_b, double friction = -1.0);
    
    FrictionForm* frictionForm() { return friction_form_.get(); }

    // ALM controls
    void setALMMaxIters(int max_iters) { alm_max_iters_ = std::max(1, max_iters); }
    void setALMConstraintTolerance(double tol) { alm_constraint_tolerance_ = std::max(1e-12, tol); }
    void setALMDualTolerance(double tol) { alm_dual_tolerance_ = std::max(1e-12, tol); }
    void setALMContactTolerance(double tol) { alm_contact_tolerance_ = std::max(1e-12, tol); }
    void setALMHardeningTrigger(double trigger) { alm_hardening_trigger_ = std::max(1.0, trigger); }
    void setALMHardeningRatio(double ratio) { alm_hardening_ratio_ = std::max(1.0, ratio); }

    // Last-step diagnostics / KPI
    double lastAngularVelocityError() const { return last_angular_velocity_error_; }
    double lastTorqueSaturationRatio() const { return last_torque_saturation_ratio_; }
    double lastDualResidual() const { return last_dual_residual_; }
    double lastContactResidual() const { return last_contact_residual_; }
    double lastMaxConstraintViolation() const { return last_max_constraint_violation_; }
    const std::vector<ContactResidualSample>& lastContactResidualSeries() const {
        return last_contact_residual_series_;
    }
    bool writeLastContactResidualSeriesCSV(
        const std::string& file_path = "output/contact_residual_series.csv") const;

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
    std::shared_ptr<DistanceBarrierForm> barrier_form_;

    // ALM runtime options
    int alm_max_iters_ = 10;
    double alm_constraint_tolerance_ = 1e-4;
    double alm_dual_tolerance_ = 1e-3;
    double alm_contact_tolerance_ = 1e-3;
    double alm_hardening_trigger_ = 2.0;
    double alm_hardening_ratio_ = 2.0;

    // Last-step KPI cache
    double last_angular_velocity_error_ = 0.0;
    double last_torque_saturation_ratio_ = 0.0;
    double last_dual_residual_ = 0.0;
    double last_contact_residual_ = 0.0;
    double last_max_constraint_violation_ = 0.0;
    std::vector<ContactResidualSample> last_contact_residual_series_;
    
    void initializeAdaptiveBarrier(World& world);
    double computeMaxStep(World& world, double dt);
    
    // Detect contact pairs for friction
    std::vector<ContactPair> detectContacts(World& world);
    std::vector<ContactPair> detectContactsAtState(World& world, const Eigen::VectorXd& x_state) const;

    double estimateBroadPhaseMinDistance(double inflation_radius);

    bool isContactPairAllowed(int body_id_a, int body_id_b) const;
    static std::uint64_t makePairKey(int body_id_a, int body_id_b);

    std::unordered_set<std::uint64_t> allowed_contact_pair_keys_;
    std::unordered_map<std::uint64_t, double> pair_friction_overrides_;
    Physics::Contact::ContactAssembler contact_assembler_;
    Physics::Contact::ContactCandidateSet contact_candidates_;
};

} // namespace NexDynIPC::Dynamics
