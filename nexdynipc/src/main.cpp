#include "NexDynIPC/App/Simulation.h"
#include "NexDynIPC/App/JointTests.h"
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    // Check for test mode
    if (argc > 1 && std::string(argv[1]) == "--test") {
        NexDynIPC::App::JointTests::run();
        return 0;
    }

    std::cout << "Starting NexDynIPC Simulation (Headless)..." << std::endl;

    NexDynIPC::App::SimulationConfig config;
    config.dt = 0.001;
    config.max_time = 3.0; // Run longer to see pendulum swing
    config.output_dir = "output";
    
    // Experimental Settings (Tune these for accuracy/stability)
    config.newmark_gamma = 0.5;      // 0.5 = No Damping (Noisy Acc), > 0.5 = Damping (Smooth Acc)
    config.joint_stiffness = 1000.0; // Higher (e.g. 1e5, 1e6) = Less Position Drift, More Noise
    
    if (argc > 1) {
        config.scene_file = argv[1];
    } else {
        config.scene_file = "assets/double_pendulum.json"; // Default to JSON asset
    }

    if (argc > 2) {
        config.output_name = argv[2];
    }

    try {
        NexDynIPC::App::Simulation sim(config);
        sim.run(); // Run simulation
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
