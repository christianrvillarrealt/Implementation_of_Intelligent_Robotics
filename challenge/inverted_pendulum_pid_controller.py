import numpy as np
import matplotlib.pyplot as plt

# System parameters
m = 0.1 # Mass of the pendulum (kg)
M = 1.5 # Mass of the cart (kg)
l = 0.4 # Length of the pendulum (m)
g = 9.81 # Gravity (m/s^2)

# Initial conditions
x_c0 = 0 # Initial position of the cart (m)
x_dot_c0 = 0 # Initial velocity of the cart (m/s)
theta0 = 0.1 # Initial angle of the pendulum (rad)
theta_dot0 = 0 # Initial angular velocity of the pendulum (rad/s)

# Desired final values
x_cd = 1 # Desired position of the cart (m)
x_dot_cd = 0 # Desired velocity of the cart (m/s)
theta_d = 0 # Desired angle of the pendulum (rad/s)
theta_dot_d = 0 # Desired angular velocity of the pendulum (rad/s)

# PID gains for the cart
Kp_c = 50 # Proportional gain
Ki_c = 1 # Integral gain
Kd_c = 40 # Derivative gain

# PID gains for the pendulum
Kp_p = 50 # Proportional gain
Ki_p = 1 # Integral gain
Kd_p = 20 # Derivative gain

# Simulation parameters
t_start = 0 # Initial time (s)
t_end = 15 # End time (s)
dt = 0.01 # Time step (s)
time = np.arange(t_start, t_end, dt) # Time NumPy array

# State variables initialized
x_c = x_c0
x_dot_c = x_dot_c0
theta = theta0
theta_dot = theta_dot0

# Error signals initialized
int_e_c = 0 # Integral of cart positon error
int_e_p = 0 # Integral of pendulum angle error

# Histories of variable values for plots
x_c_h = []
x_dot_c_h = []
theta_h = []
theta_dot_h = []
t_h = []
u_h = [] # Control input history

# Simulation
for t in time:
    # Error calculation
    e_c = x_cd - x_c # Cart position error
    e_c_dot = x_dot_cd - x_dot_c # Cart velocity error
    e_p = theta_d - theta # Pendulum angle error
    e_p_dot = theta_dot_d - theta_dot # Pendulum angle velocity error

    # Update error integrals
    int_e_c += e_c * dt # Product of cart position error and time step
    int_e_p += e_p * dt # Product of pendulum angle error and time step

    # Calculate PID outputs
    u_c = Kp_c * e_c + Ki_c * int_e_c + Kd_c * e_c_dot # Cart PID
    u_p = Kp_p * e_p + Ki_p * int_e_p + Kd_p * e_p_dot # Pendulum PID

    # Calculate and store control input u
    u = u_c + u_p

    # Update the system dynamics using the equations of motion
    x_ddot_c = (u - m * g * theta) / M # Linear acceleration of cart
    theta_ddot = (u - (m + M) * g * theta) / (M * l) # Angular acceleration of pendulum

    # Update state variables using Euler integration
    x_dot_c += x_ddot_c * dt # Cart velocity update with acceleration and time step
    x_c += x_dot_c * dt # Cart position with velocity and time step
    theta_dot += theta_ddot * dt # Pendulum velocity update with angular acceleration and time step
    theta += theta_dot * dt # Pendulum position with angular velocity and time step

    # Results are recorded for plots
    x_c_h.append(x_c)
    x_dot_c_h.append(x_dot_c)
    theta_h.append(theta)
    theta_dot_h.append(theta_dot)
    t_h.append(t)
    u_h.append(u)

# Plot is started once 15 seconds are done
plt.figure(figsize=(12,10))

# Cart position
plt.subplot(3, 2, 1)
plt.plot(t_h, x_c_h, label='Cart Position (x_c)')
plt.axhline(y=x_cd, color='r', linestyle='--', label='Desired Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Cart Position vs. Time')
plt.legend()
plt.grid()

# Cart velocity
plt.subplot(3, 2, 2)
plt.plot(t_h, x_dot_c_h, label='Cart Velocity (x_dot_c_h)')
plt.axhline(y=x_dot_cd, color='r', linestyle='--', label='Desired Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Cart Velocity vs. Time')
plt.legend()
plt.grid()

# Pendulum angle
plt.subplot(3, 2, 3)
plt.plot(t_h, theta_h, label='Pendulum Angle (theta)')
plt.axhline(y=theta_d, color='r', linestyle='--', label='Desired Angle')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Pendulum Angle vs. Time')
plt.legend()
plt.grid()

# Pendulum angular position
plt.subplot(3, 2, 4)
plt.plot(t_h, theta_dot_h, label='Pendulum Angular Velocity (rad/s)')
plt.axhline(y=theta_dot_d, color='r', linestyle='--', label='Desired Angular Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Pendulum Angular Velocity vs. Time')
plt.legend()
plt.grid()

plt.subplot(3, 2, 5)
plt.plot(t_h, u_h, label='Control Input (u)')
plt.xlabel('Time (s)')
plt.ylabel('Control Input (N)')
plt.title('Control Input vs. Time (s)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()