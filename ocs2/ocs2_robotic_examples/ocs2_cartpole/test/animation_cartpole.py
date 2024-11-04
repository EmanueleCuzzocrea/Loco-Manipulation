import numpy as np
import os
import rospkg
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Rectangle
from ocs2_cartpole import mpc_interface
from ocs2_cartpole import (
    scalar_array,
    vector_array,
    TargetTrajectories,
)
from CartPolePyBindingTest import compute_mpc_control

# Path of the package
packageDir = rospkg.RosPack().get_path('ocs2_cartpole')
taskFile = os.path.join(packageDir, 'config/mpc/task.info')
libFolder = os.path.join(packageDir, 'auto_generated')

# Initialize MPC interface
mpc = mpc_interface(taskFile, libFolder)

# State and input dimensions
stateDim = 4
inputDim = 1

# Set the goal
desiredTimeTraj = scalar_array()
desiredTimeTraj.push_back(0.0)
desiredInputTraj = vector_array()
desiredInputTraj.push_back(np.zeros(inputDim))
desiredStateTraj = vector_array()
desiredStateTraj.push_back(np.zeros(stateDim))
targetTrajectories = TargetTrajectories(desiredTimeTraj, desiredStateTraj, desiredInputTraj)
mpc.reset(targetTrajectories)

# Parameters
M = 2.0
m = 0.2
l = 1.0
g = -9.81
h = 0.01

# Initial conditions
theta = 3.14
x = 0.0
theta_dot = 0.0
x_dot = 0.0
current_state = np.array([theta, x, theta_dot, x_dot])

# Lists to collect data
theta_list = []
x_list = []
theta_dot_list = []
x_dot_list = []
control_list = []
time_list = []
x_pole = []
y_pole = []

# Main loop
for i in range(2000):
    control, predicted_state = compute_mpc_control(mpc, current_state)

    # Euler integration
    x_ddot = (1/(M + m*pow(np.sin(theta),2)))*(control + m*np.sin(theta)*(l*pow(theta_dot,2) + g*np.cos(theta)))
    theta_ddot = (1/(l*(M + m*pow(np.sin(theta),2))))*(-control*np.cos(theta) - m*l*pow(theta_dot,2)*np.cos(theta)*np.sin(theta) - (M + m)*g*np.sin(theta))
    x_dot += float(h*x_ddot)
    x += float(h*x_dot)
    theta_dot += float(h*theta_ddot)
    theta += float(h*theta_dot)
    
    # Update the state
    current_state = np.array([theta, x, theta_dot, x_dot])

    # Collect data for plot
    theta_list.append(theta)
    x_list.append(x)
    theta_dot_list.append(theta_dot)
    x_dot_list.append(x_dot)
    control_list.append(control)
    time_list.append(i * h)

    x_pole.append(x + l*np.sin(theta))
    y_pole.append(l*np.cos(theta))

    # Print
    print(f"Iteration {i + 1}:")
    print(f"Control applied: {control[0]:.4f}")
    print(f"Current state: {theta:.4f}, {x:.4f}, {theta_dot:.4f}, {x_dot:.4f}")




# Create Animation:
# Setup Figure:
fig, ax = plt.subplots()
p, = ax.plot([], [], color='royalblue')
min_lim = -5
max_lim = 5
ax.axis('equal')
ax.set_xlim([min_lim, max_lim])
ax.set_ylim([min_lim, max_lim])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Cartpole Simulation:')
title = "simulation"

# Setup Animation Writer:
FPS = 20
sample_rate = int(1 / (h * FPS))  # Real Time Playback
dpi = 300
writerObj = FFMpegWriter(fps=FPS)

# Initialize Patch: (Cart)
width = 1  # Width of Cart
height = width / 2  # Height of Cart
xy_cart = (x_list[0] - width / 2, - height / 2)  # Bottom Left Corner of Cart
r = Rectangle(xy_cart, width, height, color='cornflowerblue')  # Rectangle Patch
ax.add_patch(r)  # Add Patch to Plot

# Draw the Ground:
ground = ax.hlines(-height / 2, min_lim, max_lim, colors='black')
height_hatch = 0.25
width_hatch = max_lim - min_lim
xy_hatch = (min_lim, - height / 2 - height_hatch)
ground_hatch = Rectangle(xy_hatch, width_hatch, height_hatch, facecolor='None', linestyle='None', hatch='/')
ax.add_patch(ground_hatch)

# Animate:
with writerObj.saving(fig, title + ".mp4", dpi):
    for i in range(len(time_list)):
        if (i % 5 == 0):
            # Update Pendulum Arm:
            x_pole_arm = [x_list[i], x_pole[i]]
            y_pole_arm = [0, y_pole[i]]
            p.set_data(x_pole_arm, y_pole_arm)
            # Update Cart Patch:
            r.set(xy=(x_list[i] - width / 2,  - height / 2))
            # Update Drawing:
            fig.canvas.draw()
            # Save Frame:
            writerObj.grab_frame()