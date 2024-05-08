import numpy as np


def NextState(config, vel, dt, vel_max):
    """Computes the next config state of the youBot

    Inputs:
    :param config: Initial configuration of the chasis and arm
    :param vel: Velocities applied to the robot
    :param dt: Time step
    :param vel_max: Max velocities of the joints

    Outputs:
    Returns: The new 12 vector config of the robot
    :param new_config: new configuration updated by the applied V_b 
    """
    # Setting max velocities
    for i in range(len(vel)):
        if vel[i] > vel_max:
            vel[i] = vel_max
        elif vel[i] < -vel_max:
            vel[i] = -vel_max

    # Config of the chasis
    r = 0.0475
    l = 0.47 / 2
    w = 0.3 / 2

    # Creating the H Pseudo Inverse
    H_pinv = (r / 4) * np.array(
        [
            [-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
            [1, 1, 1, 1],
            [-1, 1, -1, 1],
        ]
    )

    # Determining the V_b
    V_b = (H_pinv@(vel[0:4].T))*dt
    phi = config[0]

    # Generating the odom q values
    if abs(V_b[0]) < 1e-3:
        dq_b = np.array([0,V_b[1],V_b[2]])
    else:
        dq_b = np.array([V_b[0],
                         ((V_b[1]*np.sin(V_b[0]))+(V_b[2]*(np.cos(V_b[0])-1)))/V_b[0],
                         ((V_b[2]*np.sin(V_b[0]))+(V_b[1]*(1-np.cos(V_b[0]))))/V_b[0]])
    mat_q = np.array([
        [1,0,0],
        [0,np.cos(phi),-np.sin(phi)],
        [0,np.sin(phi),np.cos(phi)]
        ])
    # Updating the chasis config
    new_config = np.zeros(13)
    new_config[0:3] = config[0:3] + (mat_q@dq_b)[0:3]
    new_config[3:8] = config[3:8] + vel[4:9]*dt
    new_config[8:12] = config[8:12] + vel[0:4]*dt
    return new_config
