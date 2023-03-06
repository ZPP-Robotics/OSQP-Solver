cmake_minimum_required(VERSION 3.22)
project(GOMP)

set(CMAKE_CXX_STANDARD 17)

# ---- from our organization
# place Kinematics-UR5e-arm repo next to this repo
file(GLOB SOURCES
    ../Kinematics-UR5e-arm/src/analytical_ik.h
    ../Kinematics-UR5e-arm/src/analytical_ik.cpp
)

include(FetchContent)
FetchContent_Declare(
        osqpcpp
        # Specify the commit you depend on and update it regularly.
        GIT_REPOSITORY https://github.com/google/osqp-cpp.git
)
FetchContent_MakeAvailable(osqpcpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# ---- Build header-only library qp_solver
file(GLOB_RECURSE sources src/*.h)
add_library(qp_solver INTERFACE ${sources} src/horizontal-line.h)
target_link_libraries(qp_solver INTERFACE osqp-cpp Eigen3::Eigen)
target_include_directories(qp_solver INTERFACE src)


# ---- Build usage example
add_executable(solver_example solver-example.cpp ${SOURCES})
target_link_libraries(solver_example qp_solver)


# ----- Build tests
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        origin/main
)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

add_executable(tests test/test.cpp ${SOURCES})
target_link_libraries(tests qp_solver gtest gmock gtest_main)

