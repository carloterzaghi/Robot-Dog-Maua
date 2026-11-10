"""Stance Leg Trajectory Controller (Stance Controller).

Calculates linear and angular position increments for legs in ground support,
generating the traction forces required to propel the chassis at the commanded velocity.
"""

from __future__ import annotations
from typing import Tuple
import numpy as np
from numpy.typing import NDArray

from Command import Command
from Config import Configuration
from State import State
from Transformations import euler2mat


class StanceController:
    """Computes Cartesian trajectories for feet in ground support."""

    def __init__(self, config: Configuration) -> None:
        """Initializes the controller with robot dynamic configurations.

        Args:
            config: Configuration object containing dt, gains, and velocity bounds.
        """
        self.config = config

    def position_delta(
        self, 
        leg_index: int, 
        state: State, 
        command: Command
    ) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Calculates foot position and rotation increments for the next time step.

        Translates the foot opposite to the commanded body velocity and regulates
        body height Z via a proportional first-order filter.

        Args:
            leg_index: Leg index (0=FR, 1=FL, 2=RR, 3=RL).
            state: Current state containing foot locations and height.
            command: Command vector (vx, vy, yaw_rate, target height).

        Returns:
            Tuple of (delta_p, delta_R) containing the linear displacement vector (3,) [m]
            and the 3x3 incremental rotation matrix.
        """
        z = state.foot_locations[2, leg_index]
        v_xy = np.array([
            -command.horizontal_velocity[0],
            -command.horizontal_velocity[1],
            (1.0 / self.config.z_time_constant) * (state.height - z),
        ])
        delta_p = v_xy * self.config.dt
        delta_r = euler2mat(0.0, 0.0, -command.yaw_rate * self.config.dt)
        return delta_p, delta_r

    def next_foot_location(
        self, 
        leg_index: int, 
        state: State, 
        command: Command
    ) -> NDArray[np.float64]:
        """Calculates updated 3D foot Cartesian coordinates for the next time step.

        Args:
            leg_index: Leg index (0 to 3).
            state: Current kinematic state.
            command: Operator velocity and attitude commands.

        Returns:
            3D Cartesian coordinate vector [x, y, z] for the stance foot [m].
        """
        foot_location = state.foot_locations[:, leg_index]
        delta_p, delta_r = self.position_delta(leg_index, state, command)
        incremented_location = delta_r @ foot_location + delta_p
        return incremented_location
