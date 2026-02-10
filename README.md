# NexDynIPC

A headless physics simulation engine with Implicit Euler integration and IPC-based collision handling (Work In Progress).

## Prerequisites

- CMake 3.20+
- C++20 compatible compiler (MSVC 2019+, GCC 10+, Clang 10+)
- vcpkg (configured efficiently via `VCPKG_ROOT` environment variable or similar)

## Dependencies

Managed via vcpkg manifest mode:
- Eigen3
- spdlog
- nlohmann-json
- TBB
- SuiteSparse (CHOLMOD)
- OpenVDB

## Building

This project uses CMake with vcpkg integration.

1. **Configure**:
   ```pwsh
   cmake -S . -B build
   ```
   *Note: This will download and build dependencies via vcpkg if not already installed. This may take some time on the first run.*

2. **Build**:
   ```pwsh
   cmake --build build --config Debug
   ```
   *Or for Release capability:*
   ```pwsh
   cmake --build build --config Release
   ```

## Running the Simulation

After building, the executable is located in `build/Debug` (or `build/Release`).

1. **Run**:
   ```pwsh
   .\build\Debug\NexDynIPC.exe
   ```

2. **Check Output**:
   The simulation results are saved to `output/simulation_results.csv`.
   
   **CSV Format**:
   `frame_idx,time,body_id,body_name,pos_x,pos_y,pos_z,qx,qy,qz,qw,vx,vy,vz,ax,ay,az,wx,wy,wz,alphax,alphay,alphaz`

## Project Structure

- `include/NexDynIPC`: Header files organized by module (Math, Physics, Dynamics, App).
- `src`: Source files corresponding to headers.
- `output`: Simulation configuration and result files.
- `documents`: Design documentation.