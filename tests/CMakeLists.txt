################################################################################
# NexDynIPC Tests
################################################################################

# Find Catch2
find_package(Catch2 CONFIG REQUIRED)

# Test executable
add_executable(nexdynipc_tests
    main.cpp
    
    # Math module tests
    math/test_autodiff.cpp
    math/test_autodiff_types.cpp
    math/test_interval.cpp
    
    # Geometry module tests
    geometry/test_aabb.cpp
    geometry/test_bvh.cpp
    
    # Physics module tests
    physics/test_ccd.cpp
    physics/test_friction.cpp
    
    # Dynamics module tests
    dynamics/test_joint.cpp
)

# Include directories
target_include_directories(nexdynipc_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# Link libraries
target_link_libraries(nexdynipc_tests PRIVATE
    NexDynIPC_lib
    Catch2::Catch2WithMain
)

# Compiler options
if(MSVC)
    target_compile_options(nexdynipc_tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(nexdynipc_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Enable C++20
target_compile_features(nexdynipc_tests PRIVATE cxx_std_20)

# Register tests with CTest
include(Catch)
catch_discover_tests(nexdynipc_tests)
