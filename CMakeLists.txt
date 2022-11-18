cmake_minimum_required(VERSION 3.23)
project(test3)

set(CMAKE_CXX_STANDARD 17)
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

include(FetchContent)
FetchContent_Declare(
        osqpcpp
        # Specify the commit you depend on and update it regularly.
        URL https://github.com/google/osqp-cpp/archive/8cd904e2b49c24dd41d11f8c6e0adb113dd5e26c.zip
)
FetchContent_MakeAvailable(osqpcpp)

# Mujoco test executable
add_executable(mujocotest main.cpp solver.cpp)
target_link_libraries(mujocotest ${MUJOCO_LIB} ${ARMA_LIB} ${GLFW_LIB})
target_include_directories(mujocotest PRIVATE ${MUJOCO_INCLUDE_DIR} ${ARMA_INCLUDE_DIR} ${GLFW_INCLUDE_DIR})

if(APPLE)
    set_target_properties(mujocotest PROPERTIES LINK_OPTIONS "-Wl,-rpath,${MUJOCO_PATH}")
elseif(UNIX)
    target_include_directories(mujocotest PRIVATE ${MUJOCO_INCLUDE_DIR})
endif()

# OSQP solver test executable
add_executable(osqptest solver.cpp constraint_builder.h)
target_link_libraries(osqptest osqp-cpp)