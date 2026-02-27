#include "NexDynIPC/TimeIntegration/ImplicitTimeIntegrator.h"
#include "NexDynIPC/TimeIntegration/ImplicitNewmark.h"
#include "NexDynIPC/TimeIntegration/ImplicitEulerIntegrator.h"
#include <iostream>

namespace NexDynIPC::TimeIntegration {

    void ImplicitTimeIntegrator::init(const Eigen::VectorXd& x, const Eigen::VectorXd& v, const Eigen::VectorXd& a, double dt) {
        x_prev_ = x;
        v_prev_ = v;
        a_prev_ = a;
        dt_ = dt;
    }

    std::shared_ptr<ImplicitTimeIntegrator> ImplicitTimeIntegrator::create(const nlohmann::json& config) {
        std::string type = "ImplicitNewmark"; // Default
        if (config.contains("type")) {
            type = config["type"].get<std::string>();
        }

        if (type == "ImplicitNewmark" || type == "Newmark") {
            double beta = 0.25;
            double gamma = 0.5;
            if (config.contains("beta")) beta = config["beta"].get<double>();
            if (config.contains("gamma")) gamma = config["gamma"].get<double>();
            return std::make_shared<ImplicitNewmark>(beta, gamma);
        }
        else if (type == "ImplicitEuler" || type == "Euler") {
            return std::make_shared<ImplicitEuler>();
        }
        
        std::cerr << "Unknown integrator type: " << type << ". Defaulting to ImplicitNewmark." << std::endl;
        return std::make_shared<ImplicitNewmark>();
    }

} // namespace NexDynIPC::TimeIntegration
