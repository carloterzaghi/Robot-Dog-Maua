import numpy as np


def euler2mat(ai, aj, ak):
    """
    Converte ângulos de Euler (roll=ai, pitch=aj, yaw=ak) em matriz de rotação 3x3.
    Equivalente exato ao transforms3d.euler.euler2mat(ai, aj, ak, 'sxyz').
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
    ])


def quat2euler(q):
    """
    Converte quatérnion [w, x, y, z] em ângulos de Euler (roll, pitch, yaw).
    Equivalente ao transforms3d.euler.quat2euler(q, 'sxyz').
    """
    w, x, y, z = q

    # Roll (rotação em torno de X)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (rotação em torno de Y)
    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw (rotação em torno de Z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)
