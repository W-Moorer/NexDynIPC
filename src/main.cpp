#include <iostream>
#include <spdlog/spdlog.h>
#include <Eigen/Dense>

int main() {
    spdlog::info("NexDynIPC simulation platform started.");
    
    Eigen::Vector3d gravity(0, 0, -9.81);
    spdlog::info("Gravity vector initialized: [{}, {}, {}]", gravity.x(), gravity.y(), gravity.z());

    return 0;
}
