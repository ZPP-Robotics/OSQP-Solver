import ctypes

# Load the shared object library
mylib = ctypes.CDLL("./gomp.cpython-39-x86_64-linux-gnu.so")


start_pos_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
end_pos_tcp = [1.0, 2.0, 3.0]
time_step = 0.1
waypoints_count = 10
velocity_constraints = ([], [])
acceleration_constraints = ([], [])
position_constraints = ([], [])
obstacles = [mylib.Obstacle(1), mylib.Obstacle(2), mylib.Obstacle(3)]

result = mylib.solve(start_pos_joints, end_pos_tcp, time_step, waypoints_count, velocity_constraints, acceleration_constraints, position_constraints, obstacles)
print(result)