import time
import random
import numpy as np
import pybullet as p

#### Uncomment the following 2 lines if "module gym_pybullet_drones cannot be found"
# import sys
# sys.path.append('../')

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync
from sysid_controller import SYSID_controller

DURATION = 50
"""int: The duration of the simulation in seconds."""
GUI = False
"""bool: Whether to use PyBullet graphical interface."""
RECORD = False
"""bool: Whether to save a video under /files/videos. Requires ffmpeg"""

if __name__ == "__main__":

    #### Create the ENVironment ################################
    # specify initial state
    z_pos_init = 3.0
    z_pos_init_offset = 0.0
    initial_state = np.array([[0.0, 0.0, z_pos_init + z_pos_init_offset]])
    # specify initial velocity
    z_vel_init = 0.0
    z_vel_init_offset = 1.0

    ENV = CtrlAviary(gui=GUI, record=RECORD, initial_xyzs=initial_state)
    PYB_CLIENT = ENV.getPyBulletClient()

    # set initial velocity (not exposed yet)
    # drone_id  = 0 # single agent
    # # p.resetBaseVelocity(drone_id, [x,y,z], [wx, wy,wz], physicsClientId=PYB_CLIENT)
    # p.resetBaseVelocity(drone_id, [0.0, 0.0, z_vel_init + z_vel_init_offset], [0.0, 0.0, 0.0], physicsClientId=PYB_CLIENT)

    #### Initialize the controller #############################
    control_frequency = 50 # in Hz
    CTRL = SYSID_controller(ENV, DURATION*control_frequency)

    # Different frequencies for control and physics
    common_frequency = np.lcm(ENV.SIM_FREQ, control_frequency)
    sim_steps = int(common_frequency / ENV.SIM_FREQ)
    control_steps = int(common_frequency / control_frequency)

    #### Initialize the LOGGER #################################
    LOGGER = Logger(logging_freq_hz=control_frequency)

    #### Initialize the ACTION #################################
    ACTION = {}
    OBS = ENV.reset()
    STATE = OBS["0"]["state"]

    ACTION["0"], control_input = CTRL.compute_control()

    #### Run the simulation ####################################
    START = time.time()
    for i in range(0, DURATION*common_frequency):

        ### Secret control performance booster #####################
        # if i/ENV.SIM_FREQ>3 and i%30==0 and i/ENV.SIM_FREQ<10: p.loadURDF("duck_vhacd.urdf", [random.gauss(0, 0.3), random.gauss(0, 0.3), 3], p.getQuaternionFromEuler([random.randint(0, 360),random.randint(0, 360),random.randint(0, 360)]), physicsClientId=PYB_CLIENT)

        #### Step the simulation ###################################
        if i % sim_steps == 0:
            OBS, _, _, _ = ENV.step(ACTION)

        #### Compute control #######################################
        if i % control_steps == 0:
            STATE = OBS["0"]["state"]
            ACTION["0"], control_input = CTRL.compute_control()

            #### Log the simulation ####################################
            LOGGER.log(drone=0, timestamp=i/common_frequency, state=STATE, control_input=control_input)

        #### Printout ##############################################
        if i % sim_steps == 0:
            ENV.render()

        #### Sync the simulation ###################################
        if GUI:
            sync(i, START, ENV.TIMESTEP)

    #### Close the ENVironment #################################
    ENV.close()

    #### Save the simulation results ###########################
    LOGGER.save()

    #### Plot the simulation results ###########################
    LOGGER.plot()
