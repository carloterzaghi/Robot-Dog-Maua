"""Teleoperation Command Vector.

Encapsulates all user motion intentions issued from the gamepad controller
(linear velocities, yaw rate, desired attitude, and button event flags).
"""

from __future__ import annotations
from typing import TYPE_CHECKING
import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:
    from Config import Configuration


class Command:
    """Data structure containing control targets and discrete button events."""

    def __init__(self, config: Configuration | None = None) -> None:
        """Initializes command variables in idle rest state.

        Args:
            config: Optional global configuration instance.
        """
        # Desired translational velocity [vx, vy] in horizontal plane [m/s]
        self.horizontal_velocity: NDArray[np.float64] = np.array([0.0, 0.0])

        # Desired yaw angular velocity [rad/s]
        self.yaw_rate: float = 0.0

        # Target center-of-mass height [m]
        self.height: float = -0.22

        # Desired body orientation (radians)
        self.pitch: float = 0.0
        self.roll: float = 0.0
        self.activation: int = 0

        # Discrete button events
        self.activate_event: bool = False
        self.trot_event: bool = False
        self.hop_event: bool = False
        self.A_button: bool = False
        self.X_button: bool = False
        self.L3_button: bool = False
