import numpy as np
import os
import rospkg
import matplotlib.pyplot as plt
from ocs2_mobile_manipulator import mpc_interface
from ocs2_mobile_manipulator import (
    scalar_array,
    vector_array,
    TargetTrajectories,
)
from MobileManipulatorPyBindingTest import compute_mpc_control

# Path of the package
packageDir = rospkg.RosPack().get_path('ocs2_mobile_manipulator')
taskFile = os.path.join(packageDir, 'config/franka/task.info')
libFolder = os.path.join(packageDir, 'auto_generated')
urdfDir = rospkg.RosPack().get_path('ocs2_robotic_assets')
urdf_ = os.path.join(urdfDir, 'resources/mobile_manipulator/franka/urdf/panda.urdf')

# Initialize MPC interface
mpc = mpc_interface(taskFile, libFolder, urdf_)

# State and input dimensions
stateDim = 7
inputDim = 7

# Set the goal
desiredTimeTraj = scalar_array()
desiredTimeTraj.push_back(0.0)
desiredInputTraj = vector_array()
desiredInputTraj.push_back(np.zeros(inputDim))
desiredStateTraj = vector_array()
#desiredStateTraj.push_back(np.array([0.5, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]))
desiredStateTraj.push_back(np.array([0.5, 0.5, 0.7, 0.0, 0.707, 0.0, 0.707]))
#[0.5, 0.5, 0.7, 0.707, 0, 0.707, 0]
targetTrajectories = TargetTrajectories(desiredTimeTraj, desiredStateTraj, desiredInputTraj)
mpc.reset(targetTrajectories)


# Initial conditions
#current_state = np.array([0.0, 0.81, 0.0, -0.46, 3.14, 2.1, 0.78])
current_state = np.array([0.0, -0.569, 0.0, -2.810, 0.0, 3.037, 0.741])
# goal_state = np.array([0.0, 1.06, 0.0, 0.59, 3.14, 2.67, 0.78])
h = 0.01

t_list = []
x_list = []
u_list = []


# Main loop
for i in range(1000):
    control, predicted_state = compute_mpc_control(mpc, current_state)

    # Update the state
    current_state = predicted_state

    t_list.append(i * h)
    x_list.append(current_state)
    u_list.append(control)

    # Print
    print(f"Iteration {i + 1}:")
    print(f"Current state: {current_state[0]:.3f}, {current_state[1]:.3f}, {current_state[2]:.3f}, {current_state[3]:.3f}, {current_state[4]:.3f}, {current_state[5]:.3f}, {current_state[6]:.3f}")


# Plots
plt.figure(figsize=(10, 8))

# x and x_dot plots
plt.subplot(2, 1, 1)
plt.plot(t_list, x_list, label='state [rad]')
plt.title('Joint angles', fontsize=18)
plt.xlabel('Time [s]', fontsize=14)
plt.legend(fontsize=14)
plt.grid()


## Control plot
plt.subplot(2, 1, 2)
plt.plot(t_list, u_list, label='Control [N]')
plt.title('Control input', fontsize=18)
plt.xlabel('Time [s]', fontsize=14)
plt.legend(fontsize=14)
plt.grid()

# Show plots
plt.tight_layout()
plt.show()