"""
core/kinematics.py — Cinemática Inversa (IK) e funções matemáticas do robô.

Implementação única que substitui as 6 cópias dispersas pelos arquivos originais
(leg_test.py × 4, leg_flexion.py × 1, stabilization.py × 1).

Modelo geométrico — vista lateral (plano X-Z):
  - Z cresce para cima (negativo = abaixo do corpo).
  - X cresce para frente do robô.
  - Fêmur (UPPER_LEG) conecta o ombro ao joelho.
  - Tíbia (LOWER_LEG) conecta o joelho ao pé.
  - O alcance máximo da perna é MAX_RADIUS mm a partir do ombro.

Mapeamento de ângulos para servo:
  - Pernas do lado DIREITO: ângulo direto (sem espelho).
  - Pernas do lado ESQUERDO: 180° − ângulo (montagem espelhada).
"""

import math
import numpy as np
from numpy.linalg import norm

from configs.robot_config import UPPER_LEG, LOWER_LEG, MAX_RADIUS, MAX_Z, TIBIA_OFFSET


# ── Interpolação ──────────────────────────────────────────────────────────────

def ease(start: float, end: float, n: int) -> np.ndarray:
    """
    Gera n pontos interpolados de start até end usando a curva Smootherstep (C²).

    f(t) = 6t⁵ − 15t⁴ + 10t³ — velocidade zero nas extremidades,
    elimina picos de corrente no arranque e na parada dos servos.
    """
    t = np.linspace(0.0, 1.0, n)
    t_smooth = t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
    return start + (end - start) * t_smooth


def cubic_bezier(
    p0: list | np.ndarray,
    p1: list | np.ndarray,
    p2: list | np.ndarray,
    p3: list | np.ndarray,
    n: int,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Gera n pontos ao longo de uma curva de Bézier cúbica 2D.

    Cada Pi é [x, z]. Retorna (x_array, z_array).

    B(t) = (1−t)³·P0 + 3(1−t)²t·P1 + 3(1−t)t²·P2 + t³·P3,  t ∈ [0, 1]
    """
    p0, p1, p2, p3 = (np.asarray(p, dtype=float) for p in (p0, p1, p2, p3))
    t = np.linspace(0.0, 1.0, n).reshape(-1, 1)
    pts = (
        (1 - t) ** 3 * p0
        + 3 * (1 - t) ** 2 * t * p1
        + 3 * (1 - t) * t ** 2 * p2
        + t ** 3 * p3
    )
    return pts[:, 0], pts[:, 1]


# ── Funções auxiliares de ângulo ──────────────────────────────────────────────

def _point_to_rad(x: float, z: float) -> float:
    """Converte coordenadas (x, z) para ângulo polar em [0, 2π)."""
    theta = math.atan2(z, x)
    return (theta + 2 * math.pi) % (2 * math.pi)


def _angle_corrector(angles: list[float]) -> list[float]:
    """
    Aplica a correção de referencial do modelo geométrico da perna:
      θ_tibia_corrigida = θ_femur + θ_tibia − 5π/4
      θ_femur_corrigido = θ_femur − π/2
    """
    angles[1] = angles[0] + angles[1] - 5 / 4 * math.pi
    angles[0] = angles[0] - math.pi / 2
    return angles


# ── Cinemática Inversa ────────────────────────────────────────────────────────

def ik(
    x: float,
    z: float,
    upper: int = UPPER_LEG,
    lower: int = LOWER_LEG,
    max_radius: int = MAX_RADIUS,
    max_z: int = MAX_Z,
) -> tuple[float, float]:
    """
    Cinemática inversa: posição (x, z) em mm → ângulos (θ_femur, θ_tibia) em radianos.

    Aplica clamping automático:
      - z > max_z → z = max_z  (não pode subir mais que o teto)
      - |pé| > max_radius → reescala para ficar no raio máximo

    Retorna: (θ_femur_rad, θ_tibia_rad) — valores SEM correção de espelho.
    Use ik_to_servo_angles() para obter graus prontos para o servo.
    """
    if z > max_z:
        z = max_z

    len_b = norm([x, 0.0, z])
    if len_b > max_radius:
        scale = max_radius / len_b
        x *= scale
        z *= scale
        len_b = max_radius

    arg_b2 = np.clip(
        (upper ** 2 + len_b ** 2 - lower ** 2) / (2 * upper * len_b), -1.0, 1.0
    )
    arg_b3 = np.clip(
        (upper ** 2 + lower ** 2 - len_b ** 2) / (2 * upper * lower), -1.0, 1.0
    )

    b1 = _point_to_rad(x, z)
    b2 = math.acos(arg_b2)
    b3 = math.acos(arg_b3)

    theta2 = b1 - b2
    theta3 = math.pi - b3

    return tuple(_angle_corrector([theta2, theta3]))


def ik_to_servo_angles(
    x: float,
    z: float,
    mirror: bool = False,
    tibia_offset: float = TIBIA_OFFSET,
) -> tuple[float, float]:
    """
    IK completa → ângulos em graus prontos para atribuir ao servo.

    Args:
        x, z:         posição do pé em mm.
        mirror:       True para pernas do lado esquerdo (180° − valor).
        tibia_offset: correção adicional da tíbia (default −8°).

    Returns:
        (femur_deg, tibia_deg) — valores em graus [0, 180].
    """
    theta_femur, theta_tibia = ik(x, z)
    femur_deg = math.degrees(theta_femur)
    tibia_deg = math.degrees(theta_tibia) + tibia_offset

    if mirror:
        femur_deg = 180.0 - femur_deg
        tibia_deg = 180.0 - tibia_deg

    return femur_deg, tibia_deg
