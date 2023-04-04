# Compile gomp shared library before running this test!
import gomp

N_DIM = 6
Q_MIN = -6.283185307179586232
Q_MAX = 6.283185307179586232
M_PI = 3.14159265358979323846
INF = 1.00000000000000002e+30

time_step = 0.1
waypoints_count = 50 + 2

start_pos_joints = [0] * N_DIM
end_pos_joints = [M_PI] + [0] * (N_DIM - 1)

position_constraints = ([Q_MIN] * N_DIM, [Q_MAX] * N_DIM)
velocity_constraints = ([-M_PI] * N_DIM, [M_PI] * N_DIM)
acceleration_constraints = ([-M_PI * 800 / 180], [M_PI * 800 / 180])

constraints_3d = ((-INF, -0.3, -0.1), (INF, INF, INF))

obstacles = [((0, 1, 0), (0, 0, 0.6), True)]

result = gomp.solve_1(start_pos_joints, end_pos_joints, time_step, waypoints_count, velocity_constraints, acceleration_constraints, position_constraints, constraints_3d, obstacles)
print(result)