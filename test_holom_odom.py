import numpy as np
import math
import time

# Constants for the robot (mecanum wheel configuration)
R = 0.05  # Wheel radius in meters (example: 5 cm)
L = 0.3   # Distance between front and back wheels (example: 30 cm)
W = 0.2   # Distance between left and right wheels (example: 20 cm)

# Initial pose of the robot
x = 0.0   # Initial x position (meters)
y = 0.0   # Initial y position (meters)
theta = 0.0  # Initial orientation (radians)

# Time step
dt = 0.1  # Time step in seconds (10 updates per second)

# Function to compute the robot's global velocity from the wheel velocities
def calculate_odometry(v1, v2, v3, v4, x, y, theta, dt):
    # Kinematic matrix for mecanum wheels
    v_wheel = np.array([v1, v2, v3, v4])
    
    # Transformation matrix from wheel velocities to global velocities
    T = (R / 4) * np.array([[1,  1,  1,  1],
                            [-1, 1, -1, 1],
                            [-1/(L + W), 1/(L + W), -1/(L + W), 1/(L + W)]])
    
    # Compute the robot's velocity in the global frame
    global_vel = T @ v_wheel  # [vx, vy, omega] in robot frame
    
    # Extract the velocities
    vx = global_vel[0]
    vy = global_vel[1]
    omega = global_vel[2]

    # Update position and orientation
    theta_new = theta + omega * dt

    # If rotating in place, keep x and y unchanged
    if omega != 0:
        x_new = x  # No change in x
        y_new = y  # No change in y
    else:
        # Update the position based on current orientation only if there's movement
        x_new = x + vx * math.cos(theta) * dt - vy * math.sin(theta) * dt
        y_new = y + vx * math.sin(theta) * dt + vy * math.cos(theta) * dt

    return x_new, y_new, theta_new

def simulate_robot_motion():
    global x, y, theta
    
    # Wheel velocity for rotation in place (positive for counterclockwise, negative for clockwise)
    v = 1.0  # You can adjust this velocity value as needed

    # Set wheel velocities for in-place rotation (counterclockwise)
    v1 = -v  # Front-left wheel
    v2 = v   # Front-right wheel
    v3 = -v  # Back-left wheel
    v4 = v   # Back-right wheel

    # Simulate for 10 seconds (100 updates)
    for step in range(100):
        # Calculate new odometry based on wheel velocities
        x, y, theta = calculate_odometry(v1, v2, v3, v4, x, y, theta, dt)
        
        # Print updated position and orientation
        print(f"Time: {step * dt:.1f}s | Position: ({x:.2f}, {y:.2f}) | Orientation: {math.degrees(theta):.2f} degrees")
        
        # Sleep for the time step duration (simulate real-time)
        time.sleep(dt)

# Run the simulation
simulate_robot_motion()
