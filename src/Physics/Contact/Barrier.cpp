#include "NexDynIPC/Physics/Contact/Barrier.hpp"
#include <cmath>
#include <limits>

namespace NexDynIPC::Physics {

// Using the C2 clamped log barrier from IPC
// b(d) = -(d - d_hat)^2 * ln(d / d_hat)
// This vanishes at d = d_hat and explodes at d = 0.

double BarrierPotential::value(double d, double d_hat, double kappa) {
    if (d >= d_hat) return 0.0;
    if (d <= 0.0) return std::numeric_limits<double>::infinity();
    
    double r = d / d_hat;
    return -kappa * (d - d_hat) * (d - d_hat) * std::log(r);
}

double BarrierPotential::gradient(double d, double d_hat, double kappa) {
    if (d >= d_hat) return 0.0;
    if (d <= 0.0) return -std::numeric_limits<double>::infinity(); 

    // b(d) = -k * (d-dh)^2 * ln(d/dh)
    // b'(d) = -k * [ 2(d-dh)*ln(d/dh) + (d-dh)^2 * (1/d) ]
    double diff = d - d_hat;
    double log_r = std::log(d / d_hat);
    
    return -kappa * (2 * diff * log_r + diff * diff / d);
}

double BarrierPotential::hessian(double d, double d_hat, double kappa) {
    if (d >= d_hat) return 0.0;
    if (d <= 0.0) return std::numeric_limits<double>::infinity();

    // b'(d) = -k * [ 2(d-dh)ln(d/dh) + (d-dh)^2/d ]
    // term1 = 2(d-dh)ln(d/dh) -> deriv: 2*ln(d/dh) + 2(d-dh)/d
    // term2 = (d-dh)^2/d -> deriv: [2(d-dh)*d - (d-dh)^2]/d^2 = 2(d-dh)/d - (d-dh)^2/d^2
    
    // b''(d) = -k * [ 2*ln(d/dh) + 2(d-dh)/d + 2(d-dh)/d - (d-dh)^2/d^2 ]
    //        = -k * [ 2*ln(d/dh) + 4(d-dh)/d - (d-dh)^2/d^2 ]
    
    double diff = d - d_hat;
    double log_r = std::log(d / d_hat);
    
    return -kappa * (2 * log_r + 4 * diff / d - (diff * diff) / (d * d));
}

} // namespace NexDynIPC::Physics
