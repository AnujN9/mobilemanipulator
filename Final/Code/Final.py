import modern_robotics as mr
import numpy as np
import TrajectoryGenerator
import FeedbackControl
import NextState
import matplotlib.pyplot as plt


# Initial conditions of the youBot
Tsc_intial = np.array([[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 0.025], [0, 0, 0, 1]])
Tsc_goal = np.array([[-1, 0, 0, 0], [0, -1, 0, -0.5], [0, 0, 1, 0.025], [0, 0, 0, 1]])
init_config = np.array([0.2, 0.5, 1.5, -3.2, -0.4, 0.2, -1.6, 0.1,0,0,0,0,0])
Tse_intial = np.array([[0, 0, 1, 0.0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])
Xint = np.zeros((4,4))
Kp = 8*np.eye(6)
Ki = 0.8*np.eye(6)
k = 1
dt = 0.01

# Configuration of the youBot
Tb0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]])
M = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]])
B1 = np.array([0, 0, 1, 0, 0.033, 0])
B2 = np.array([0, -1, 0, -0.5076, 0, 0])
B3 = np.array([0, -1, 0, -0.3526, 0, 0])
B4 = np.array([0, -1, 0, -0.2176, 0, 0])
B5 = np.array([0, 0, 1, 0, 0, 0])
Blist = np.vstack((B1, B2, B3, B4, B5)).T
r = 0.0475
l = 0.47 / 2
w = 0.3 / 2
F6 = (r / 4) * np.array(
    [
        [-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
        [1, 1, 1, 1],
        [-1, 1, -1, 1],
    ]
)
F = np.vstack((np.zeros((2, 4)), F6, np.zeros((1, 4))))
vel_max = 100

# Initial values for Trajectory Generator
theta = (3 * np.pi) / 4
Tce_grasp = np.array(
    [
        [np.cos(theta), 0, np.sin(theta), 0],
        [0, 1, 0, 0],
        [-np.sin(theta), 0, np.cos(theta), 0],
        [0, 0, 0, 1],
    ]
)
Tce_standoff = np.array(
    [
        [np.cos(theta), 0, np.sin(theta), 0],
        [0, 1, 0, 0],
        [-np.sin(theta), 0, np.cos(theta), 0.15],
        [0, 0, 0, 1],
    ]
)

# Generating Trajectories
Traj = TrajectoryGenerator.TrajectoryGenerator(
    Tse_intial, Tsc_intial, Tsc_goal, Tce_grasp, Tce_standoff, k
)
# np.savetxt("Traj.csv", Traj, delimiter=",")

# Initializing the final csv output
f_config = np.zeros((len(Traj) * k, 13))
f_config[0, :] = init_config
Xerr_list = []

# Iterating through the loop to get the csv and error values
for i in range(len(f_config)-1):
    phi = f_config[i, 0]
    Tsb = np.array(
        [
            [np.cos(phi), -np.sin(phi), 0, f_config[i, 1]],
            [np.sin(phi), np.cos(phi), 0, f_config[i, 2]],
            [0, 0, 1, 0.0963],
            [0, 0, 0, 1],
        ]
    )
    T0e = mr.FKinBody(M,Blist,f_config[i,3:8])
    Tse_current = Tsb@Tb0@T0e
    Xd = np.array([
        [Traj[i][0],Traj[i][1],Traj[i][2],Traj[i][9]],
        [Traj[i][3],Traj[i][4],Traj[i][5],Traj[i][10]],
        [Traj[i][6],Traj[i][7],Traj[i][8],Traj[i][11]],
        [0,0,0,1]])
    Xd_next = np.array([
        [Traj[i+1][0],Traj[i+1][1],Traj[i+1][2],Traj[i+1][9]],
        [Traj[i+1][3],Traj[i+1][4],Traj[i+1][5],Traj[i+1][10]],
        [Traj[i+1][6],Traj[i+1][7],Traj[i+1][8],Traj[i+1][11]],
        [0,0,0,1]])
    # Using feedback controller
    V,Xerr,Xint = FeedbackControl.FeedbackControl(Tse_current,Xd,Xd_next,Xint,Kp,Ki,dt)
    Xerr_v = mr.se3ToVec(Xerr)
    Xerr_list.append(Xerr_v)
    Jarm = mr.JacobianBody(Blist,f_config[i,3:8])
    Jbase = (mr.Adjoint(mr.TransInv(T0e)@mr.TransInv(Tb0)))@F
    Je = np.hstack((Jbase,Jarm))
    velocity = np.linalg.pinv(Je,1e-5)@V
    # Getting the next states
    f_config[i+1,:] = NextState.NextState(f_config[i,:],velocity,dt,vel_max)
    f_config[i,12] = Traj[i][12]

print("Generating .csv file")
np.savetxt("new_task.csv", f_config, delimiter=",")

Xe = np.array(Xerr_list)
np.savetxt("new_task_err.csv", f_config, delimiter=",")

plt.plot(Xe[:,0], label='Xerr_1')
plt.plot(Xe[:,1], label='Xerr_2')
plt.plot(Xe[:,2], label='Xerr_3')
plt.plot(Xe[:,3], label='Xerr_4')
plt.plot(Xe[:,4], label='Xerr_5')
plt.plot(Xe[:,5], label='Xerr_6')

# Adding title and legend
plt.title('Xerr Plot vs time')
plt.xlabel('Time')
plt.ylabel('Error')
plt.legend()

# Display the plot
plt.show()