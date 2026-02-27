#pragma once

#include <cmath>

namespace NexDynIPC::Physics {

class BarrierPotential {
public:
    // Compute barrier potential value: b(d) = -(d - d_hat)^2 * log(d / d_hat)
    // This is the IPC 2020 log-barrier formulation approximation or similar
    // Actually, IPC uses -kappa * (d - d_hat)^2 * log(d/d_hat) is one form, 
    // or standard: kappa * sum( - log(d) ) ?
    // Let's use the standard IPC barrier:
    // b(d) = -(d - d_hat)^2 * ln(d / d_hat)  if d < d_hat
    //        0                               otherwise
    // Wait, the standard log barrier is -ln(d). 
    // IPC uses a C2 smooth barrier.
    
    // Let's implement the standard IPC C2 barrier.
    static double value(double d, double d_hat, double kappa);

    // Gradient w.r.t d
    static double gradient(double d, double d_hat, double kappa);

    // Hessian w.r.t d
    static double hessian(double d, double d_hat, double kappa);
};

} // namespace NexDynIPC::Physics
