cmake_minimum_required(VERSION 3.20)

project(NexDynIPC)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Output directories
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/output)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/output)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/output)

# Find packages (using vcpkg manifest mode)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Boost REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(TBB CONFIG REQUIRED)
# SuiteSparse is installed as separate components by vcpkg
find_package(CHOLMOD CONFIG REQUIRED) 
find_package(OpenVDB CONFIG REQUIRED)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# Collect source files (excluding main.cpp for library)
file(GLOB_RECURSE LIB_SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")
list(FILTER LIB_SOURCES EXCLUDE REGEX "src/main\\.cpp$")
message(STATUS "Detected LIB_SOURCES: ${LIB_SOURCES}")

# Create library
add_library(NexDynIPC_lib STATIC ${LIB_SOURCES})

# Link libraries to library
target_link_libraries(NexDynIPC_lib PUBLIC 
    Eigen3::Eigen 
    Boost::headers
    spdlog::spdlog 
    nlohmann_json::nlohmann_json
    TBB::tbb
    SuiteSparse::CHOLMOD
    OpenVDB::openvdb
)

# Compiler options for library
if(MSVC)
    target_compile_options(NexDynIPC_lib PRIVATE /W4 /permissive-)
else()
    target_compile_options(NexDynIPC_lib PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Create executable
add_executable(NexDynIPC ${CMAKE_SOURCE_DIR}/src/main.cpp)
target_link_libraries(NexDynIPC PRIVATE NexDynIPC_lib)

# Compiler options for executable
if(MSVC)
    target_compile_options(NexDynIPC PRIVATE /W4 /permissive-)
else()
    target_compile_options(NexDynIPC PRIVATE -Wall -Wextra -Wpedantic)
endif()

################################################################################
# Tests
################################################################################
enable_testing()
add_subdirectory(tests)
