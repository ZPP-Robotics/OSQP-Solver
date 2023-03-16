include_directories(${GOMP_SOURCE_DIR}/src ${Kinematics_UR5e_arm_SOURCE_DIR}/src)

add_executable(solver_example solver-example.cpp)
target_link_libraries(solver_example qp_solver)