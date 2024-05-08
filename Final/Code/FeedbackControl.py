import modern_robotics as mr
import numpy as np


def FeedbackControl(X,Xd,Xd_next,Xint,Kp,Ki,dt):
    """Computes the motion of the robot and generates a reference trajectory for the end-effector

    Inputs:
    :param X: Current actual end-effector config
    :param Traj: Current end-effector reference config
    :param Traj_next: End-effector reference config at the next timestep
    :param Kp: Proportional gain of the controller
    :param Ki: Integral gain of the controller
    :param dt: Timestep

    Outputs:
    Returns: The commanded end-effector velocity in teh end-effector frame
    :param V: twist V in {e} frame
    :param Xerr: err of X to be plotted
    :param Xint: integral err of X
    """
    
    Xerr = mr.MatrixLog6(mr.TransInv(X)@Xd)
    Xint = Xint + Xerr*dt
    V_d = (1/dt)*(mr.MatrixLog6(mr.TransInv(Xd)@Xd_next))
    V = (mr.Adjoint(mr.TransInv(X)@Xd)@mr.se3ToVec(V_d)) + (Kp@mr.se3ToVec(Xerr)) + (Ki@mr.se3ToVec(Xint))
    return V, Xerr, Xint
