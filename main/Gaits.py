"""Gait Phase Manager (Gait Controller).

Controls timing sequences for dynamic trotting, determining at each time step (tick)
the current gait phase index and which feet should be in ground support (Stance = 1)
or swinging in the air (Swing = 0).
"""

from __future__ import annotations
import numpy as np
from numpy.typing import NDArray

from Config import Configuration


class GaitController:
    """Computes gait phases and contact modes for quadruped locomotion."""

    def __init__(self, config: Configuration) -> None:
        """Initializes the controller with gait timing parameters.

        Args:
            config: Configuration object containing timing settings (dt, overlap, swing).
        """
        self.config = config

    def phase_index(self, ticks: int) -> int:
        """Calculates the gait cycle phase index for the current time step.

        Args:
            ticks: Integer counter of elapsed control time steps.

        Returns:
            Current phase index in the cycle (0 to `num_phases - 1`).
        """
        phase_time = ticks % self.config.phase_length
        phase_sum = 0
        for i in range(self.config.num_phases):
            phase_sum += self.config.phase_ticks[i]
            if phase_time < phase_sum:
                return i
        return 0

    def subphase_ticks(self, ticks: int) -> int:
        """Calculates elapsed ticks since the start of the current phase.

        Args:
            ticks: Integer counter of elapsed control time steps.

        Returns:
            Number of ticks elapsed within the active phase.
        """
        phase_time = ticks % self.config.phase_length
        phase_sum = 0
        for i in range(self.config.num_phases):
            phase_sum += self.config.phase_ticks[i]
            if phase_time < phase_sum:
                return int(phase_time - phase_sum + self.config.phase_ticks[i])
        return 0

    def contacts(self, ticks: int) -> NDArray[np.int_]:
        """Determines the ground contact state for all 4 legs at the given tick.

        Args:
            ticks: Integer counter of elapsed control time steps.

        Returns:
            A 4-element NumPy array (FR, FL, RR, RL) where 1 indicates stance
            (ground contact) and 0 indicates swing (flight).
        """
        return self.config.contact_phases[:, self.phase_index(ticks)]
