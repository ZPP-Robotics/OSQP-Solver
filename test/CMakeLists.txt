# Compiling shared library for Python bindings.

cmake_minimum_required(VERSION 3.22)
project(gomp-solver)

find_package(Python3 COMPONENTS Development)

add_library(gomp ../src/bindings/gomp-solver-binding.cpp)
target_compile_options(gomp PRIVATE -O3 -Wall -std=gnu++11)
target_link_libraries(gomp PRIVATE Python3::Python)
target_include_directories(gomp PRIVATE /usr/include/eigen3/ /home/olaf/anaconda3/lib/python3.9/site-packages/pybind11/include/)

include(FetchContent)
FetchContent_Declare(
        osqpcpp
        # Specify the commit you depend on and update it regularly.
        GIT_REPOSITORY https://github.com/google/osqp-cpp.git
)
FetchContent_MakeAvailable(osqpcpp)

target_link_libraries(gomp INTERFACE osqp-cpp Eigen3::Eigen)

target_include_directories(gomp PRIVATE ${osqpcpp_SOURCE_DIR}/include)
