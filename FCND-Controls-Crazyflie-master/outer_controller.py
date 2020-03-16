"""Controller that generates velocity commands

Class for the specific impelementation of a controller that will be comanding velocity commands.
Contains the solution for P controllers on altitude and lateral position to commands velocities
to a crazyflie over the wireless link.

@author: Adrien Perkins
"""

import numpy as np


class OuterLoopController(object):
    """controller class for computing velocity commands to control lateral position and altitude.

    solution implementation to a controller that computes velocity commands from position and
    altitude commands.
    """

    def __init__(self):

        # define all the gains that will be needed
        self._kp_pos = 0.0  # gain for lateral position error
        self._kp_alt = 0.0  # gain for altitude error

        # some limits to use
        self._v_max = 0.3       # the maximum horizontal velocity in [m/s]
        self._hdot_max = 0.4    # the maximum vertical velocity in [m/s]

    def lateral_position_control(self, pos_cmd, pos, vel_cmd):
        """compute the North and East velocity command to control the lateral position.

        Use a PID controller (or your controller of choice) to compute the North and East velocity
        commands to be send to the crazyflie given the commanded position and current position.

        Args:
            pos_cmd: the commanded position [north, east, down] in [m]
            pos: the current position [north, east, down] in [m]
            vel_cmd: the commanded velocity [vn, ve, vd] in [m/s]

        Returns:
            the velocity command as a 2 element numpy array [Vn, Ve]
            numpy array
        """

        # Student TODO: compute a [Vn, Ve] command


        return lateral_vel_cmd

    def altitude_control(self, alt_cmd, alt, hdot_cmd=0.0):
        """compute the vertical velocity command to control the altitude.

        Use a PID controller (or your controller of choice) to compute the vertical velocity
        command to be send to the crazyflie given the commanded altitude and current altitude.

        Args:
            alt_cmd: the commanded altitude in [m] (positive up)
            alt: the current altitude in [m] (positive up)
            hdot_cmd: the commanded vertical velocity in [m/s] (positive up)

        Returns:
            the vertical velocity to command
            float
        """

        # Student TODO: compute a [Vup] command


        return hdot_cmd
