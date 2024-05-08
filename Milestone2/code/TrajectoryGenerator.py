import modern_robotics as mr
import numpy as np


def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k):
    """Computes the trajectories needed to move the end effector to grab
    and move an object

    Inputs:
    :param Tse_initial: Initial configuration of the end effector
    :param Tsc_initial: Initial configuration of the cube object
    :param Tsc_final: Goal configuration of the cube object
    :param Tce_grasp: Initial configuration of the end effector
    :param Tce_standoff: Initial configuration of the end effector

    Outputs:
    Returns: The screw trajectory to move the end_effector along the set of points
    :param Traj: the resultant trajectory array containing the r matrices, p values and the gripper state
    It is a 13 by N matrix that is of the form "r11,r12,r13,r21,r22,r23,r31,r32,r33,p1,p2,p3,gripper_state". 
    This is so that the .csv files is easy to generate. 
    """

    #Intializing the timing for each action. Kept everything as 2 seconds
    t_segment = np.array([4, 4, 1, 4, 4, 4, 1, 4]) * (k / 0.01)

    # Initialzing the final returns
    Traj = []

    # Moving from intial to the standoff
    Tend = Tsc_initial@Tce_standoff
    T_se1 = mr.ScrewTrajectory(
        Tse_initial,
        Tend,
        (t_segment[0] * 0.01) / k,
        t_segment[0],
        5,
    )
    for t in T_se1:
        t_s = np.concatenate([t[:3,:3].flatten(),t[:3,3],[np.array(0.0)]])
        Traj.append(t_s)

    # Moving from standoff to grasping
    Tstart = Tsc_initial@Tce_standoff
    Tend = Tsc_initial@Tce_grasp
    T_se2 = mr.ScrewTrajectory(
        Tstart,
        Tend,
        t_segment[1] * 0.01 / k,
        t_segment[1],
        5,
    )
    for t in T_se2:
        t_s = np.concatenate([t[:3,:3].flatten(),t[:3,3],[np.array(0.0)]])
        Traj.append(t_s)

    # Closing the gripper
    Tstart = Tsc_initial@Tce_grasp
    Tend = Tsc_initial@Tce_grasp
    T_se3 = mr.ScrewTrajectory(
        Tstart,
        Tend,
        t_segment[2] * 0.01 / k,
        t_segment[2],
        5,
    )
    for t in T_se3:
        t_s = np.concatenate([t[:3,:3].flatten(),t[:3,3],[np.array(1.0)]])
        Traj.append(t_s)

    # Moving from grasping to standoff
    Tstart = Tsc_initial@Tce_grasp
    Tend = Tsc_initial@Tce_standoff
    T_se4 = mr.ScrewTrajectory(
        Tstart,
        Tend,
        t_segment[3] * 0.01 / k,
        t_segment[3],
        5,
    )
    for t in T_se4:
        t_s = np.concatenate([t[:3,:3].flatten(),t[:3,3],[np.array(1.0)]])
        Traj.append(t_s)

    # Moving from standoff near initial to standoff near final
    Tstart = Tsc_initial@Tce_standoff
    Tend = Tsc_final@Tce_standoff
    T_se5 = mr.ScrewTrajectory(
        Tstart,
        Tend,
        t_segment[4] * 0.01 / k,
        t_segment[4],
        5,
    )
    for t in T_se5:
        t_s = np.concatenate([t[:3,:3].flatten(),t[:3,3],[np.array(1.0)]])
        Traj.append(t_s)

    # Moving from standoff to final object configuration
    Tstart = Tsc_final@Tce_standoff
    Tend = Tsc_final@Tce_grasp
    T_se6 = mr.ScrewTrajectory(
        Tstart,
        Tend,
        t_segment[5] * 0.01 / k,
        t_segment[5],
        5,
    )
    for t in T_se6:
        t_s = np.concatenate([t[:3,:3].flatten(),t[:3,3],[np.array(1.0)]])
        Traj.append(t_s)

    # Opening the gripper
    Tstart = Tsc_final@Tce_grasp
    Tend = Tsc_final@Tce_grasp
    T_se7 = mr.ScrewTrajectory(
        Tstart,
        Tend,
        t_segment[6] * 0.01 / k,
        t_segment[6],
        5,
    )
    for t in T_se7:
        t_s = np.concatenate([t[:3,:3].flatten(),t[:3,3],[np.array(0.0)]])
        Traj.append(t_s)

    # Moving from releasing back to standoff near final
    Tstart = Tsc_final@Tce_grasp
    Tend = Tsc_final@Tce_standoff
    T_se8 = mr.ScrewTrajectory(
        Tstart,
        Tend,
        t_segment[7] * 0.01 / k,
        t_segment[7],
        5,
    )
    for t in T_se8:
        t_s = np.concatenate([t[:3,:3].flatten(),t[:3,3],[np.array(0.0)]])
        Traj.append(t_s)

    """At the end of each screw trajectory calculation the rotation matrix, the position vector
    and the gripper state is all appended to a Traj matrice.
    """

    return Traj
