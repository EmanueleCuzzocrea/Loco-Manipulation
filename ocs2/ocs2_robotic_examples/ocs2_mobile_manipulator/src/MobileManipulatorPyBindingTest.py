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

def compute_mpc_control(mpc, current_state):
    # Set MPC observations
    u = np.zeros(12)
    mpc.setObservation(0.0, current_state, u)

    # Optimization problem
    mpc.advanceMpc()

    # Get MPC solutions
    t_result = scalar_array()
    x_result = vector_array()
    u_result = vector_array()
    mpc.getMpcSolution(t_result, x_result, u_result)

    # Return only the first input value
    return u_result[0], x_result[1]