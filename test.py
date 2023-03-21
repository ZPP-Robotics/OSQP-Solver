import ctypes
import gomp

start_pos_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
end_pos_tcp = [1.0, 2.0, 3.0]
time_step = 0.1
waypoints_count = 10
velocity_constraints = ([], [])
acceleration_constraints = ([], [])
position_constraints = ([], [])
obstacles = [gomp.Obstacle(1), gomp.Obstacle(2), gomp.Obstacle(3)]

result = gomp.solve(start_pos_joints, end_pos_tcp, time_step, waypoints_count, velocity_constraints, acceleration_constraints, position_constraints, obstacles)
print(result)