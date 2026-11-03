"""Filtering and Signal Processing Utility Functions.

Implements symmetric deadband filters and first-order low-pass filters with
slew-rate limiting for smooth control signal conditioning.
"""

from __future__ import annotations
import numpy as np


def deadband(value: float, band_radius: float) -> float:
    """Applies a symmetric deadband filter to eliminate noise around zero.

    Args:
        value: Input signal.
        band_radius: Deadband threshold radius (e.g., 0.08 for 8% deadband).

    Returns:
        Filtered signal, zero within [-band_radius, band_radius],
        and linearly proportional outside it.
    """
    return float(max(value - band_radius, 0.0) + min(value + band_radius, 0.0))


def clipped_first_order_filter(
    current_value: float, 
    target_value: float, 
    max_rate: float, 
    tau: float
) -> float:
    """Computes a first-order rate-limited response to smooth target transitions.

    Prevents abrupt attitude changes and excessive servo acceleration.

    Args:
        current_value: Current state value.
        target_value: Desired target value.
        max_rate: Maximum allowed rate of change per second.
        tau: Filter time constant (seconds).

    Returns:
        Rate of change saturated within [-max_rate, max_rate].
    """
    rate = (target_value - current_value) / max(tau, 1e-6)
    return float(np.clip(rate, -max_rate, max_rate))
