cmake_minimum_required(VERSION 3.23)
project(test3)

set(CMAKE_CXX_STANDARD 17)
set(MUJOCO_PATH /Users/mat/Documents/mujoco)

list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/")
list(APPEND CMAKE_PREFIX_PATH ${MUJOCO_PATH})

find_library(ARMA_LIB armadillo)
find_path(ARMA_INCLUDE_DIR armadillo)

find_library(GLFW_LIB glfw)
find_path(GLFW_INCLUDE_DIR glfw)

find_library(MUJOCO_LIB mujoco)

include(FetchContent)
FetchContent_Declare(
        osqpcpp
        # Specify the commit you depend on and update it regularly.
        URL https://github.com/google/osqp-cpp/archive/8cd904e2b49c24dd41d11f8c6e0adb113dd5e26c.zip
)
FetchContent_MakeAvailable(osqpcpp)

add_executable(test3 main.cpp)

target_link_libraries(test3
        osqp-cpp
        ${MUJOCO_LIB}
        ${ARMA_LIB}
        ${GLFW_LIB})

target_include_directories(test3 PRIVATE ${ARMA_INCLUDE_DIR} ${GLFW_INCLUDE_DIR})

if (APPLE)
    set_target_properties(test3 PROPERTIES LINK_OPTIONS "-Wl,-rpath,${MUJOCO_PATH}")
endif()