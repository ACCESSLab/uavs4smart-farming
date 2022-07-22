import numpy as np 
import time
from threading import Thread 

class PoseController(Thread):
    def __init__(self, init_pos:list, gains:list, dt:float):
        """
        @param init_pos: initial pose [x, y, theta]
        @param gains: pid gains [kpRho, kpAlpha, kpBeta]
        """
        super().__init__()
        self._curr_state = np.array(init_pos, dtype=float)
        self._target_state = self._curr_state.copy()
        self.Kp_rho = gains[0]
        self.Kp_alpha = gains[1]
        self.Kp_beta = gains[2]
        self._dt = dt
        self.terminate = False  
    
    def setGoal(self, target:list):
        """
        @param target: target pose [x, y, theta]
        """
        y = np.array(target)
        x = self._curr_state[:2]
        delta = y - x 
        q = np.arctan2(delta[1], delta[0])
        self._target_state = np.array([target[0], target[1], q])
    
    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):
        """
        Returns the control command for the linear and angular velocities as
        well as the distance to goal
        Parameters
        ----------
        x_diff : The position of target with respect to current robot position
                 in x direction
        y_diff : The position of target with respect to current robot position
                 in y direction
        theta : The current heading angle of robot with respect to x axis
        theta_goal: The target angle of robot with respect to x axis
        Returns
        -------
        rho : The distance between the robot and the goal position
        v : Command linear velocity
        w : Command angular velocity
        """

        # Description of local variables:
        # - alpha is the angle to the goal relative to the heading of the robot
        # - beta is the angle between the robot's position and the goal
        #   position plus the goal angle
        # - Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards
        #   the goal
        # - Kp_beta*beta rotates the line so that it is parallel to the goal
        #   angle
        #
        # Note:
        # we restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
        v = self.Kp_rho * rho
        w = self.Kp_alpha * alpha - self.Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        return rho, v, w

    def publish_state(self, state:list):
        raise NotImplemented

    def run(self):
        while not self.terminate:
            state_diff = self._target_state - self._curr_state
            rho, v, w = self.calc_control_command(state_diff[0], state_diff[1], self._curr_state[-1],
                                              self._target_state[-1])

            if rho <= 0.01:
                v, w = 0, 0 

            theta = self._curr_state[2] + w * self._dt
            self._curr_state[0] += v * np.cos(theta) 
            self._curr_state[1] += v * np.sin(theta)
            self._curr_state[2] = theta
            self.publish_state(self._curr_state.tolist())
            time.sleep(self._dt)

    
    def get_state(self):
        return self._curr_state