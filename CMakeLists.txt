cmake_minimum_required(VERSION 3.22)
project(GOMP)

set(CMAKE_CXX_STANDARD 17)

# ---- from our organization
# place Kinematics-UR5e-arm repo next to this repo
file(GLOB SOURCES
    ../Kinematics-UR5e-arm/src/analytical_ik.h
    ../Kinematics-UR5e-arm/src/analytical_ik.cpp
)

find_library(ARMA_LIB armadillo)
find_path(ARMA_INCLUDE_DIR armadillo)

include(FetchContent)
FetchContent_Declare(
        osqpcpp
        # Specify the commit you depend on and update it regularly.
        GIT_REPOSITORY https://github.com/google/osqp-cpp.git
)
FetchContent_MakeAvailable(osqpcpp)

# ---- Build header-only library qp_solver
file(GLOB_RECURSE sources src/*.h)
add_library(qp_solver INTERFACE ${sources})
target_link_libraries(qp_solver INTERFACE osqp-cpp ${ARMA_LIB})
target_include_directories(qp_solver INTERFACE src ${ARMA_INCLUDE_DIR})


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

file(GLOB_RECURSE test_sources test/*.cpp)
add_executable(tests ${test_sources} ${SOURCES})
target_link_libraries(tests qp_solver gtest gmock gtest_main)

#[[
set(MUJOCO_PATH /Users/mat/Documents/mujoco)
set(DOWNLOADED_LIBS /opt/homebrew/)

list(APPEND CMAKE_PREFIX_PATH ${DOWNLOADED_LIBS})
list(APPEND CMAKE_PREFIX_PATH ${MUJOCO_PATH})

find_library(ARMA_LIB armadillo)
find_path(ARMA_INCLUDE_DIR armadillo)

find_library(GLFW_LIB glfw)
find_path(GLFW_INCLUDE_DIR glfw)

find_library(MUJOCO_LIB mujoco)
find_path(MUJOCO_INCLUDE_DIR mujoco)

# Mujoco test executable
add_executable(mujocotest main.cpp solver_example.cpp utils.h)

target_link_libraries(mujocotest ${MUJOCO_LIB} ${ARMA_LIB} ${GLFW_LIB})
target_include_directories(mujocotest PRIVATE ${MUJOCO_INCLUDE_DIR} ${ARMA_INCLUDE_DIR} ${GLFW_INCLUDE_DIR})

if(APPLE)
    set_target_properties(mujocotest PROPERTIES LINK_OPTIONS "-Wl,-rpath,${MUJOCO_PATH}")
elseif(UNIX)
    target_include_directories(mujocotest PRIVATE ${MUJOCO_INCLUDE_DIR})
endif()
]]
