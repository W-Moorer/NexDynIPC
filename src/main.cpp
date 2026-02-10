#include "NexDynIPC/App/Simulation.hpp"
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "Starting NexDynIPC Simulation (Headless)..." << std::endl;

    NexDynIPC::App::SimulationConfig config;
    config.dt = 0.01;
    config.max_time = 1.0;
    config.output_dir = "output";
    config.scene_file = "scene.json"; // Placeholder

    try {
        NexDynIPC::App::Simulation sim(config);
        sim.run(); // Run simulation
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
