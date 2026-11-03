"""Spatial Rotation Transformations and Attitude Conversions.

Implements pure, optimized NumPy conversions between Euler angles (Roll, Pitch, Yaw)
and unit Quaternions [w, x, y, z], eliminating external dependencies like `transforms3d`.
"""

from __future__ import annotations
from typing import Sequence, Tuple
import numpy as np
from numpy.typing import NDArray


def euler2mat(ai: float, aj: float, ak: float) -> NDArray[np.float64]:
    """Converts Euler angles (roll=ai, pitch=aj, yaw=ak) to a 3x3 rotation matrix.

    Uses the intrinsic 'sxyz' rotation convention (exact equivalent to
    `transforms3d.euler.euler2mat(ai, aj, ak, 'sxyz')`).

    Args:
        ai: Roll angle (rotation about X) [rad].
        aj: Pitch angle (rotation about Y) [rad].
        ak: Yaw angle (rotation about Z) [rad].

    Returns:
        Orthogonal 3x3 rotation matrix.
    """
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)

    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    return np.array([
        [cj * ck, sj * sc - cs, sj * cc + ss],
        [cj * sk, sj * ss + cc, sj * cs - sc],
        [-sj,     cj * si,      cj * ci     ]
    ], dtype=np.float64)


def quat2euler(q: Sequence[float] | NDArray[np.float64]) -> Tuple[float, float, float]:
    """Converts a unit quaternion [w, x, y, z] to Euler angles (Roll, Pitch, Yaw).

    Equivalent to standard 'sxyz' in `transforms3d`.

    Args:
        q: 4-element sequence or array [w, x, y, z].

    Returns:
        Tuple of (roll, pitch, yaw) in radians [rad].
    """
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])

    # Roll (rotation about X)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation about Y)
    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw (rotation about Z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return float(roll), float(pitch), float(yaw)
