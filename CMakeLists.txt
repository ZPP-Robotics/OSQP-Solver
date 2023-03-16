cmake_minimum_required(VERSION 3.22)
project(GOMP)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(src)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  option(GOMP_BUILD_EXAMPLES "Build examples" ON)
  option(GOMP_BUILD_TESTS "Build tests" ON)

  if(GOMP_BUILD_EXAMPLES)
    add_subdirectory(examples)  
  endif()
  if(GOMP_BUILD_TESTS)
    add_subdirectory(tests)  
  endif()

endif()

