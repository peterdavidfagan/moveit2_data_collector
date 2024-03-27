"""
A basic dm environment for transporter data collection on the Franka robot.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
import dm_env

class TransporterFrankaTable(dm_env.Environment):
    """
    This dm_env is intended to be used in conjunction with PyQt data collection application.
    The management of ROS communication is handled by the data collection application.
    This application is intended to make data collection compatible with env_logger.
    """

    def __init__(self):
        pass

    def reset(self):
        pass

    def step(self, action):
        pass

    def observation_spec(self):
        pass

    def action_spec(self):
        pass

    def _compute_observation(self):
        pass
    
    def _compute_reward(self):
        pass

    def _compute_done(self):
        pass

    def close(self):
        pass




