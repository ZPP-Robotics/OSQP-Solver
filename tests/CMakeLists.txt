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