"""Swing Leg Trajectory Controller (Swing Controller).

Computes 3D aerial trajectories for feet repositioning forward in flight,
utilizing Marc Raibert's touchdown heuristic and a smooth vertical ground
clearance profile.
"""

from __future__ import annotations
import numpy as np
from numpy.typing import NDArray

from Command import Command
from Config import Configuration
from State import State
from Transformations import euler2mat


class SwingController:
    """Manages aerial foot kinematics during quadruped locomotion."""

    def __init__(self, config: Configuration) -> None:
        """Initializes the controller with swing and clearance configurations.

        Args:
            config: Robot configuration object.
        """
        self.config = config

    def raibert_touchdown_location(
        self, 
        leg_index: int, 
        command: Command
    ) -> NDArray[np.float64]:
        """Calculates optimal foot landing position using Raibert's heuristic.

        Estimates forward foot displacement ahead of the center of mass to maintain
        dynamic equilibrium based on horizontal velocity and yaw rate:
        p_touchdown = R_yaw * p_default + alpha * T_stance * V_linear.

        Args:
            leg_index: Leg index (0=FR, 1=FL, 2=RR, 3=RL).
            command: Operator velocity commands.

        Returns:
            Target 3D touchdown location vector [x, y, z] in meters [m].
        """
        delta_p_2d = (
            self.config.alpha
            * self.config.stance_ticks
            * self.config.dt
            * command.horizontal_velocity
        )
        delta_p = np.array([delta_p_2d[0], delta_p_2d[1], 0.0])

        theta = (
            self.config.beta
            * self.config.stance_ticks
            * self.config.dt
            * command.yaw_rate
        )
        rot_r = euler2mat(0.0, 0.0, theta)
        return rot_r @ self.config.default_stance[:, leg_index] + delta_p

    def swing_height(self, swing_phase: float, triangular: bool = True) -> float:
        """Calculates instantaneous vertical foot clearance above ground.

        Args:
            swing_phase: Normalized swing phase progress in [0, 1].
            triangular: If True, uses symmetric linear clearance profile.

        Returns:
            Vertical foot elevation above ground in meters [m].
        """
        if triangular:
            if swing_phase < 0.5:
                return float((swing_phase / 0.5) * self.config.z_clearance)
            else:
                return float(self.config.z_clearance * (1.0 - (swing_phase - 0.5) / 0.5))
        return float(self.config.z_clearance)

    def next_foot_location(
        self,
        swing_prop: float,
        leg_index: int,
        state: State,
        command: Command,
    ) -> NDArray[np.float64]:
        """Calculates the next 3D Cartesian coordinates for a foot swinging in the air.

        Interpolates horizontally toward the Raibert touchdown target while adding
        vertical ground clearance.

        Args:
            swing_prop: Normalized swing phase progress (0.0 at liftoff, 1.0 at touchdown).
            leg_index: Leg index (0 to 3).
            state: Current kinematic state.
            command: Operator command vector.

        Returns:
            Updated 3D Cartesian position vector [x, y, z] for the swing foot [m].
        """
        swing_prop = float(np.clip(swing_prop, 0.0, 1.0))
        foot_location = state.foot_locations[:, leg_index]
        swing_h = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(leg_index, command)

        time_left = max(self.config.dt * self.config.swing_ticks * (1.0 - swing_prop), 1e-6)
        v = (touchdown_location - foot_location) / time_left * np.array([1.0, 1.0, 0.0])
        delta_foot_location = v * self.config.dt

        z_vector = np.array([0.0, 0.0, swing_h + command.height])
        return foot_location * np.array([1.0, 1.0, 0.0]) + z_vector + delta_foot_location
