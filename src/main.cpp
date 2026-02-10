#include "NexDynIPC/App/Simulation.hpp"
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "Starting NexDynIPC Simulation (Headless)..." << std::endl;

    NexDynIPC::App::SimulationConfig config;
    config.dt = 0.01;
    config.max_time = 3.0; // Run longer to see pendulum swing
    config.output_dir = "output";
    
    if (argc > 1) {
        config.scene_file = argv[1];
    } else {
        config.scene_file = "double_pendulum"; // Default to double pendulum for testing
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
