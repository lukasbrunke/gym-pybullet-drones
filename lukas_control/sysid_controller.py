import numpy as np
from gym_pybullet_drones.envs.BaseAviary import BaseAviary

class SYSID_controller():
    ################################################################################

    def __init__(self, env: BaseAviary, total_num_iterations, period=100):
        """

        Parameters
        ----------
        env : BaseAviary
            The PyBullet-based simulation environment.

        total_num_iterations
            The number of iterations the controller is applied.

        period
            The period of the oscillation.

        """
        self.g = env.G
        """float: Gravity acceleration, in meters per second squared."""
        self.mass = env.M
        """float: The mass of quad from environment."""
        self.timestep = env.TIMESTEP
        """float: Simulation and control timestep."""
        self.kf_coeff = env.KF
        """float: RPMs to force coefficient."""
        self.km_coeff = env.KM
        """float: RPMs to torque coefficient."""
        self.num_iterations = total_num_iterations
        """int: number of iterations"""
        self.period = period
        """int: period of the input oscillation"""
        self.K_lqr = np.array([-3.0406, -3.8247])

        self.reset()

    ################################################################################

    def reset(self):
        """ Resets the controller counter."""
        self.control_counter = 0

    ################################################################################

    def compute_control(self):
        # Determine magnitude of the oscillation signal based on the gravitational constant
        magnitude = self.g / self.num_iterations * self.control_counter
        # oscillation control input signal
        u = magnitude * np.sin(self.control_counter / self.period *  2 * np.pi)

        self.control_counter += 1

        # turn_rate = sqrt( (m*u + m*g) / (4*Kf) )
        propellers_rpm = np.sqrt((u*self.mass + self.g*self.mass) / (4 * self.kf_coeff))

        # For up-down motion, assign the same turn rates to all motors
        propellers_0_and_3_rpm, propellers_1_and_2_rpm = propellers_rpm, propellers_rpm

        if self.control_counter%(1/self.timestep) == 0:
            print("control input", u)
            print("propeller {} RPM".format(0), propellers_0_and_3_rpm)
            print("propeller {} RPM".format(1), propellers_1_and_2_rpm)
            print("propeller {} RPM".format(2), propellers_1_and_2_rpm)
            print("propeller {} RPM".format(3), propellers_0_and_3_rpm)

        return np.array([propellers_0_and_3_rpm, propellers_1_and_2_rpm,
                         propellers_1_and_2_rpm, propellers_0_and_3_rpm]), u

    def compute_state_feedback(self, current_state):
        u = self.K_lqr @ current_state

        self.control_counter += 1

        # turn_rate = sqrt( (m*u + m*g) / (4*Kf) )
        propellers_rpm = np.sqrt((u*self.mass + self.g*self.mass) / (4 * self.kf_coeff))

        # For up-down motion, assign the same turn rates to all motors
        propellers_0_and_3_rpm, propellers_1_and_2_rpm = propellers_rpm, propellers_rpm

        if self.control_counter%(1/self.timestep) == 0:
            print("control input", u)
            print("propeller {} RPM".format(0), propellers_0_and_3_rpm)
            print("propeller {} RPM".format(1), propellers_1_and_2_rpm)
            print("propeller {} RPM".format(2), propellers_1_and_2_rpm)
            print("propeller {} RPM".format(3), propellers_0_and_3_rpm)

        return np.array([propellers_0_and_3_rpm, propellers_1_and_2_rpm,
                         propellers_1_and_2_rpm, propellers_0_and_3_rpm]), u

