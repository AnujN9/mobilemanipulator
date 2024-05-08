These were the values I used for the gains of the PI controller and
intial condition of the youbot Robot and the cube position.

kp = 8
ki = 0.8
Tsc_intial = np.array([[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 0.025], [0, 0, 0, 1]])
Tsc_goal = np.array([[-1, 0, 0, 0], [0, -1, 0, -0.5], [0, 0, 1, 0.025], [0, 0, 0, 1]])
init_config = np.array([0.2, 0.5, 1.5, -3.2, -0.4, 0.2, -1.6, 0.1,0,0,0,0,0])
