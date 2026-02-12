#pragma once

#include "NexDynIPC/Dynamics/TimeIntegrator.h"
#include "NexDynIPC/Math/NewtonSolver.h"
#include "NexDynIPC/TimeIntegration/ImplicitTimeIntegrator.h"
#include "NexDynIPC/Physics/Contact/CCD/CCD.h"
#include "NexDynIPC/Physics/Contact/AdaptiveBarrier.h"
#include "NexDynIPC/Dynamics/Forms/FrictionForm.h"
#include <memory>

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
