import modern_robotics as mr
import numpy as np
import NextState
import TrajectoryGenerator
import FeedbackControl


# Testing file to test different milestones

# Milestone 1

# config = np.array([0,0,0,0,0,0.2,-1.6,0,0,0,0,0,0])
# config_list = np.zeros((101,13))
# config_list[0,:] = config
# vel = np.array([10,10,10,10,0,0,0,0,0])
# for i in range(100):
#     config = NextState.NextState(config,vel,0.01,(4*np.pi))
#     config_list[i+1,:] = config
# np.savetxt("odom.csv", config_list, delimiter=",")

# Milestone 2

# Tse_intial = np.array([[0, 0, 1, 0.0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])
# Tsc_intial = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]])
# Tsc_goal = np.array([[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]])
# theta = (3*np.pi) / 4
# Tce_grasp = np.array(
#     [
#         [np.cos(theta), 0, np.sin(theta), 0],
#         [0, 1, 0, 0],
#         [-np.sin(theta), 0, np.cos(theta), 0],
#         [0, 0, 0, 1],
#     ]
# )
# Tce_standoff = np.array(
#     [
#         [np.cos(theta), 0, np.sin(theta), 0],
#         [0, 1, 0, 0],
#         [-np.sin(theta), 0, np.cos(theta), 0.3],
#         [0, 0, 0, 1],
#     ]
# )
# Traj = TrajectoryGenerator.TrajectoryGenerator(
#     Tse_intial, Tsc_intial, Tsc_goal, Tce_grasp, Tce_standoff, 1
# )
# print(Traj)

# Milestone 3

X = np.array([
    [0.170,0,0.985,0.387],
    [0,1,0,0],
    [-0.985,0,0.170,0.570],
    [0,0,0,1]
])
Xd = np.array([
    [0,0,1,0.5],
    [0,1,0,0],
    [-1,0,0,0.5],
    [0,0,0,1]
])
Xd_next = np.array([
    [0,0,1,0.6],
    [0,1,0,0],
    [-1,0,0,0.3],
    [0,0,0,1]
])
Xint = np.zeros((4,4))
Kp = np.zeros((6,6))
Ki = np.zeros((6,6))
dt = 0.01
V, Xerr, Xint = FeedbackControl.FeedbackControl(X,Xd,Xd_next,Xint,Kp,Ki,dt)
B1 = np.array([0, 0, 1, 0, 0.033, 0])
B2 = np.array([0, -1, 0, -0.5076, 0, 0])
B3 = np.array([0, -1, 0, -0.3526, 0, 0])
B4 = np.array([0, -1, 0, -0.2176, 0, 0])
B5 = np.array([0, 0, 1, 0, 0, 0])
Blist = np.vstack((B1, B2, B3, B4, B5)).T
thetalist = np.array([0,0,0.2,-1.6,0])
Jarm = mr.JacobianBody(Blist,thetalist)
M = np.array([
    [1,0,0,0.033],
    [0,1,0,0],
    [0,0,1,0.6546],
    [0,0,0,1]
])
T_0e = mr.FKinBody(M,Blist,thetalist)
T_b0 = np.array([
    [1,0,0,0.1662],
    [0,1,0,0],
    [0,0,1,0.0026],
    [0,0,0,1]
])
r = 0.0475
l = 0.47/2
w = 0.3/2
F6 = (r / 4) * np.array(
        [
            [-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
            [1, 1, 1, 1],
            [-1, 1, -1, 1],
        ]
    )
F = np.vstack((np.zeros((2,4)),F6,np.zeros((1,4))))
Jbase = (mr.Adjoint(mr.TransInv(T_0e)@mr.TransInv(T_b0)))@F
Je = np.hstack((Jbase,Jarm))
print(mr.se3ToVec(Xerr))
# print(Je)
# print(V[0])
velocity = np.linalg.pinv(Je,1e-5)@V[0]
# print(velocity)
