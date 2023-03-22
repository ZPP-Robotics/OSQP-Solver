# Compile gomp shared library before running this test!
import gomp

start_pos_joints = [8.0, 0.0, 0.0, 0.0, 0.0, 0.0]
end_pos_joints = [8.0, 0.0, 0.0, 0.0, 0.0, 0.0]
time_step = 0.1
waypoints_count = 10
velocity_constraints = ([1,2,3,4,5,6], [1,2,3,4,5,6])
acceleration_constraints = ([1,2,3,4,5,6], [1,2,3,4,5,6])
position_constraints = ([1,2,3,4,5,6], [1,2,3,4,5,6])
obstacles = [((1,2,3), (4,5,6)), ((1,2,3), (4,5,6))]

result = gomp.solve_1(start_pos_joints, end_pos_joints, time_step, waypoints_count, velocity_constraints, acceleration_constraints, position_constraints, obstacles)
print(result)