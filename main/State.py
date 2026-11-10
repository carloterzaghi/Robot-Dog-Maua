"""Robot State Definitions and Behavioral Modes.

Maintains data structures recording telemetry, foot locations, joint angles,
and operational modes of the quadruped robot.
"""

from __future__ import annotations
from enum import Enum
import numpy as np
from numpy.typing import NDArray


class BehaviorState(Enum):
    """High-level finite state machine of robot behaviors."""
    DEACTIVATED = -1  # Inactive rest (robot folded safely on ground)
    REST = 0         # Active standing support (responds to roll, pitch, height)
    TROT = 1         # Continuous dynamic trotting locomotion


class State:
    """Consolidated kinematic, temporal, and behavioral telemetry state."""

    def __init__(self) -> None:
        """Initializes state vectors to default rest values."""
        # Estimated linear velocities [vx, vy] in m/s
        self.horizontal_velocity: NDArray[np.float64] = np.array([0.0, 0.0])
        self.yaw_rate: float = 0.0

        # Body height in meters (negative Z relative to body origin)
        self.height: float = -0.22

        # Attitude angles (radians)
        self.pitch: float = 0.0
        self.roll: float = 0.0
        self.activation: int = 0
        self.quat_orientation: NDArray[np.float64] = np.array([1.0, 0.0, 0.0, 0.0])

        # Internal tick counter
        self.ticks: int = 0

        # Fundamental kinematic matrices (3 joints x 4 legs)
        self.foot_locations: NDArray[np.float64] = np.zeros((3, 4))
        self.joint_angles: NDArray[np.float64] = np.zeros((3, 4))

        # Behavioral flags
        self.behavior_state: BehaviorState = BehaviorState.DEACTIVATED
        self.is_sitting: bool = False
        self.is_laying: bool = True
        self.is_calibration: bool = False
