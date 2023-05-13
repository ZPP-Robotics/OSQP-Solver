set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)

# ---- from our organization
FetchContent_Declare(
        Kinematics_UR5e_arm
        GIT_REPOSITORY https://github.com/ZPP-Robotics/Kinematics-UR5e-arm.git
        GIT_TAG        origin/main
)
set(Kinematics_UR5e_arm_BUILD_EXAMPLES OFF CACHE INTERNAL "Build kinematics example programs")
FetchContent_MakeAvailable(Kinematics_UR5e_arm)

FetchContent_Declare(
        osqpcpp
        # Specify the commit you depend on and update it regularly.
        GIT_REPOSITORY https://github.com/google/osqp-cpp.git
)
FetchContent_MakeAvailable(osqpcpp)

FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG master
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE)
set(EIGEN_BUILD_DOC OFF)
set(BUILD_TESTING OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
FetchContent_MakeAvailable(Eigen)

file(GLOB_RECURSE *.h)
file(GLOB_RECURSE constraints/*.h)
# analytical_ik
include_directories(${Kinematics_UR5e_arm_SOURCE_DIR}/src)
add_library(qp_solver gomp-solver.h)
target_link_libraries(qp_solver INTERFACE analytical_ik osqp-cpp Eigen3::Eigen)
set_target_properties(qp_solver PROPERTIES LINKER_LANGUAGE CXX)