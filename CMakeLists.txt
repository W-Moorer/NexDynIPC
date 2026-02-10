cmake_minimum_required(VERSION 3.20)

project(NexDynIPC)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/output)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/output)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/output)

# Find packages (using vcpkg manifest mode)
find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(TBB CONFIG REQUIRED)
# SuiteSparse is installed as separate components by vcpkg
find_package(CHOLMOD CONFIG REQUIRED) 
find_package(OpenVDB CONFIG REQUIRED)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# Add executable
file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")
add_executable(NexDynIPC ${SOURCES})

# Link libraries
target_link_libraries(NexDynIPC PRIVATE 
    Eigen3::Eigen 
    spdlog::spdlog 
    nlohmann_json::nlohmann_json
    TBB::tbb
    SuiteSparse::CHOLMOD
    OpenVDB::openvdb
)

# Compiler options
if(MSVC)
    target_compile_options(NexDynIPC PRIVATE /W4 /permissive-)
else()
    target_compile_options(NexDynIPC PRIVATE -Wall -Wextra -Wpedantic)
endif()
