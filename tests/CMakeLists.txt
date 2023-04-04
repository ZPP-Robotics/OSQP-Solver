include(FetchContent)

include_directories(${GOMP_SOURCE_DIR}/src)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        origin/kinematics_without_pi
)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

add_executable(tests test.cpp)
target_link_libraries(tests qp_solver gtest gmock gtest_main)

find_package(Python3 COMPONENTS Development)
add_library(gomp MODULE ../src/bindings/gomp-solver-binding.cpp)
target_include_directories(gomp PRIVATE /usr/include/eigen3/ /home/olaf/anaconda3/lib/python3.9/site-packages/pybind11/include/)
target_link_libraries(gomp PRIVATE Python3::Python qp_solver)
