"""3D Analytical Inverse Kinematics and Geometric Leg Transformations.

Computes 3D analytical inverse kinematics (IK) for 4 legs with 3 degrees of
freedom each (Abduction, Femur, Tibia), handling anatomical offsets, physical
angular limits, and servo axis orientation masks.
"""

from __future__ import annotations
import math as m
from typing import Sequence, Tuple, Union
import numpy as np
from numpy.linalg import norm
from numpy.typing import NDArray

from Config import Configuration


def point_to_rad(p1: float, p2: float) -> float:
    """Converts a 2D Cartesian coordinate (x, y) into a continuous polar angle in [0, 2π).

    Args:
        p1: Horizontal coordinate.
        p2: Vertical coordinate.

    Returns:
        Polar angle in radians [rad].
    """
    theta = m.atan2(p2, p1)
    return float((theta + 2 * m.pi) % (2 * m.pi))


def calculate_4_bar(
    th2: float, 
    a: float, 
    b: float, 
    c: float, 
    d: float
) -> Tuple[float, float, float]:
    """Finds internal angles of a four-bar linkage (Freudenstein's method).

    Used for closed pantographic linkage kinematics.

    Args:
        th2: Actuating joint input angle DAB [rad].
        a, b, c, d: Link lengths defined clockwise from point A.

    Returns:
        Tuple of remaining internal angles: (ABC, BCD, CDA) in radians.
    """
    x_b = a * np.cos(th2)
    y_b = a * np.sin(th2)

    # Intermediate diagonal f
    f = np.sqrt((d - x_b) ** 2 + y_b ** 2)
    beta = np.arccos((f ** 2 + c ** 2 - b ** 2) / (2 * f * c))
    gamma = np.arctan2(y_b, d - x_b)

    th4 = np.pi - gamma - beta

    x_c = c * np.cos(th4) + d
    y_c = c * np.sin(th4)

    th3 = np.arctan2((y_c - y_b), (x_c - x_b))

    abc = np.pi - th2 + th3
    bcd = th4 - th3
    cda = np.pi * 2 - th2 - abc - bcd

    return float(abc), float(bcd), float(cda)


def RotMatrix3D(
    rotation: Sequence[float] = (0.0, 0.0, 0.0), 
    is_radians: bool = True, 
    order: str = 'xyz'
) -> np.matrix:
    """Generates a 3D rotation matrix from Euler angles (Roll, Pitch, Yaw).

    Args:
        rotation: Sequence of (roll, pitch, yaw).
        is_radians: If True, angles are in radians; if False, converts from degrees.
        order: Intrinsic rotation order (default: 'xyz').

    Returns:
        3x3 rotation matrix (np.matrix).
    """
    roll, pitch, yaw = rotation[0], rotation[1], rotation[2]

    if not is_radians:
        roll = m.radians(roll)
        pitch = m.radians(pitch)
        yaw = m.radians(yaw)

    rot_x = np.matrix([
        [1, 0, 0], 
        [0, m.cos(roll), -m.sin(roll)], 
        [0, m.sin(roll), m.cos(roll)]
    ])
    rot_y = np.matrix([
        [m.cos(pitch), 0, m.sin(pitch)], 
        [0, 1, 0], 
        [-m.sin(pitch), 0, m.cos(pitch)]
    ])
    rot_z = np.matrix([
        [m.cos(yaw), -m.sin(yaw), 0], 
        [m.sin(yaw), m.cos(yaw), 0], 
        [0, 0, 1]
    ])

    if order == 'xyz':
        return rot_z * rot_y * rot_x
    elif order == 'xzy':
        return rot_y * rot_z * rot_x
    elif order == 'yxz':
        return rot_z * rot_x * rot_y
    elif order == 'yzx':
        return rot_x * rot_z * rot_y
    elif order == 'zxy':
        return rot_y * rot_x * rot_z
    elif order == 'zyx':
        return rot_x * rot_y * rot_z

    return rot_z * rot_y * rot_x


def apply_physical_limits(angles: Union[Sequence[float], NDArray[np.float64]]) -> list[float]:
    """Applies mechanical and anti-collision limits to leg joint angles.

    Args:
        angles: List or array of [theta_abduction, theta_femur, theta_tibia] in degrees [°].

    Returns:
        List of three bounded angles within safe physical ranges.
    """
    result = [float(angles[0]), float(angles[1]), float(angles[2])]

    # Hard limits for abduction
    result[0] = float(np.clip(result[0], 60.0, 120.0))

    # Limits for femur
    result[1] = max(25.0, min(180.0, result[1]))

    # Dynamic limits for tibia as a function of femur position to prevent chassis strike
    lower_min = max(42.0, result[1] - 109.0)
    lower_max = min(125.0, result[1] + 20.0)
    result[2] = max(lower_min, min(lower_max, result[2]))

    return result


# Backward-compatible alias
phisicalLimits = apply_physical_limits


def angle_corrector(angles: Sequence[float]) -> list[float]:
    """Corrects servo phase angles, wraps to [0, 2π), and applies physical bounds.

    Args:
        angles: [theta_1, theta_2, theta_3] in radians.

    Returns:
        Angles converted to degrees and clamped within physical safety limits.
    """
    corrected = [0.0, 0.0, 0.0]
    corrected[0] = 2 * m.pi - angles[0] + m.pi / 2
    corrected[2] = angles[1] + angles[2] - 5 / 4 * m.pi  # Offset theta_3
    corrected[1] = angles[1] - m.pi / 2                  # Offset theta_2

    for index, theta in enumerate(corrected):
        theta = np.mod(theta, 2 * m.pi)
        if theta > m.pi:
            theta = 2 * m.pi - theta
        corrected[index] = theta

    return apply_physical_limits(np.rad2deg(corrected))


def Calculate_IK(
    r_body_foot: NDArray[np.float64], 
    leg_index: int, 
    config: Configuration
) -> list[float]:
    """Computes 3D analytical inverse kinematics for a single leg.

    Args:
        r_body_foot: 3D vector [x, y, z] of desired foot location relative to leg base [m].
        leg_index: Leg index (0=FR, 1=FL, 2=RR, 3=RL).
        config: Robot configuration object.

    Returns:
        List containing [theta_abduction, theta_femur, theta_tibia] in degrees [°].
    """
    x = float(r_body_foot[0])
    y = float(r_body_foot[1])
    z = float(r_body_foot[2])

    # Symmetry mirroring for right-side legs (FR=0, RR=2)
    is_right = leg_index in (0, 2)
    if is_right:
        y = -y

    # 1. Abduction angle (Projection on frontal YZ plane)
    len_a = norm([0, y, z])
    a_1 = point_to_rad(y, z)

    arg_a2 = m.sin(config.phi) * config.ABDUCTION_OFFSET / max(len_a, 1e-9)
    a_2 = m.asin(float(np.clip(arg_a2, -1.0, 1.0)))
    a_3 = m.pi - a_2 - config.phi

    theta_1 = (a_1 + a_3) % (2 * m.pi)

    # 2. Transformation to rotated 2D leg plane
    offset = np.array([
        0.0, 
        config.ABDUCTION_OFFSET * m.cos(theta_1), 
        config.ABDUCTION_OFFSET * m.sin(theta_1)
    ])
    translated_frame = np.array([x, y, z]) - offset
    r2 = theta_1 + config.phi - m.pi / 2

    rot_mtx = RotMatrix3D([-r2, 0, 0], is_radians=True)
    j4_2_vec = np.ravel(rot_mtx * (np.reshape(translated_frame, [3, 1])))

    x_, _, z_ = j4_2_vec[0], j4_2_vec[1], j4_2_vec[2]
    len_b = float(norm([x_, 0, z_]))

    # Clamp extension to prevent singularity
    if len_b >= 0.245:
        len_b = 0.245

    # 3. Law of Cosines for Femur and Tibia
    b_1 = point_to_rad(x_, z_)

    arg_b2 = (config.LEG_L2 ** 2 + len_b ** 2 - config.LEG_L1 ** 2) / (2 * config.LEG_L2 * len_b)
    b_2 = m.acos(float(np.clip(arg_b2, -1.0, 1.0)))

    arg_b3 = (config.LEG_L2 ** 2 + config.LEG_L1 ** 2 - len_b ** 2) / (2 * config.LEG_L2 * config.LEG_L1)
    b_3 = m.acos(float(np.clip(arg_b3, -1.0, 1.0)))

    theta_2 = b_1 - b_2
    theta_3 = m.pi - b_3

    # 4. Phase correction and mechanical limits
    revised = angle_corrector([theta_1, theta_2, theta_3])
    return [revised[0], revised[1], revised[2]]


def four_legs_inverse_kinematics(
    r_body_foot: NDArray[np.float64], 
    config: Configuration
) -> NDArray[np.float64]:
    """Finds joint angles for all 12 DOF corresponding to the body-relative foot matrix.

    Args:
        r_body_foot: A (3, 4) matrix of body-frame foot coordinates [x, y, z] in meters.
            Each column corresponds to a leg: [0]=FR, [1]=FL, [2]=RR, [3]=RL.
        config: Robot configuration object.

    Returns:
        A (3, 4) matrix of joint angles in degrees [°].
    """
    alpha = np.zeros((3, 4))
    for i in range(4):
        body_offset = config.LEG_ORIGINS[:, i]
        alpha[:, i] = Calculate_IK(
            r_body_foot[:, i] - body_offset, i, config
        )

    # Apply physical servo orientation mask
    alpha = abs(config.servo_orientation_adjusts - alpha)
    return alpha


if __name__ == "__main__":
    # Interactive standalone bench test
    cfg = Configuration()
    foot_pos = cfg.default_stance + np.array([0, 0, cfg.default_z_ref])[:, np.newaxis]
    angles = four_legs_inverse_kinematics(foot_pos, cfg)
    print("Computed angles in default standing pose (3x4 in degrees):")
    print(np.round(angles, 2))
