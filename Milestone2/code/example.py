import modern_robotics as mr
import numpy as np
import TrajectoryGenerator

"""Example code of how to use the TrajectoryGenerator function
"""

# Intial position of the end effector chosen by you, I took it from the final step of the assignment
Tse_intial = np.array([[0, 0, 1, 0.0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])

# Intial position of the cube
Tsc_intial = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]])

# Final goal position of the cube
Tsc_goal = np.array([[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]])

"""Grasping position of the end effector when picking up the cube 
which is at a 135deg angle with respect to y
"""
theta = (3*np.pi) / 4
Tce_grasp = np.array(
    [
        [np.cos(theta), 0, np.sin(theta), 0],
        [0, 1, 0, 0],
        [-np.sin(theta), 0, np.cos(theta), 0],
        [0, 0, 0, 1],
    ]
)

"""Standoff position between end effector and cube before and after pickup 
which is at 0.3m above in z axis
"""
Tce_standoff = np.array(
    [
        [np.cos(theta), 0, np.sin(theta), 0],
        [0, 1, 0, 0],
        [-np.sin(theta), 0, np.cos(theta), 0.3],
        [0, 0, 0, 1],
    ]
)

"""Using TrajectoryGenerator to give us the Tse matrix
for the entire path in 26 seconds which is defined in the function"""
Traj = TrajectoryGenerator.TrajectoryGenerator(
    Tse_intial, Tsc_intial, Tsc_goal, Tce_grasp, Tce_standoff, 1
)

# Now we write these to a csv file
np.savetxt("results.csv", Traj, delimiter=",")
