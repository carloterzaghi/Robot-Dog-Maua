import numpy as np
from numpy.linalg import norm
import math as m
import time
from adafruit_servokit import ServoKit        # type: ignore

# ── Inicialização ─────────────────────────────────────────────────────────────
kit = ServoKit(channels=16)
servo_femur = kit.servo[3]
servo_tibia = kit.servo[7]

# ── Parâmetros da perna ───────────────────────────────────────────────────────
UPPER_LEG  = 120   # mm
LOWER_LEG  = 130   # mm
MAX_RADIUS = 220   # mm
MAX_Z      = -80   # mm — z não pode ser maior que -80

# ── Parâmetros da marcha ──────────────────────────────────────────────────────
Z_APOIO  = -150   # mm — z com pé no chão (fase de apoio)
Z_SWING  =  -85   # mm — z com pé levantado (fase de balanço)
X_FRENTE =   60   # mm — posição x mais à frente
X_ATRAS  =  -60   # mm — posição x mais atrás
N_PONTOS =   20   # pontos de interpolação por fase
DELAY    = 0.015  # s  — tempo entre pontos


def point_to_rad(p1, p2):
    theta = m.atan2(p2, p1)
    return (theta + 2 * m.pi) % (2 * m.pi)


def angle_corrector(angles):
    angles[1] = angles[0] + angles[1] - 5/4 * m.pi
    angles[0] = angles[0] - m.pi / 2
    return angles


def move_leg(x, z):
    # Aplica restrição de z
    if z > MAX_Z:
        z = MAX_Z

    # Aplica restrição de raio
    len_B = norm([x, 0, z])
    if len_B > MAX_RADIUS:
        scale = MAX_RADIUS / len_B
        x *= scale
        z *= scale
        len_B = MAX_RADIUS

    # Cinemática inversa
    arg_b2 = np.clip((UPPER_LEG**2 + len_B**2 - LOWER_LEG**2) / (2 * UPPER_LEG * len_B), -1, 1)
    arg_b3 = np.clip((UPPER_LEG**2 + LOWER_LEG**2 - len_B**2) / (2 * UPPER_LEG * LOWER_LEG), -1, 1)

    b_1 = point_to_rad(x, z)
    b_2 = m.acos(arg_b2)
    b_3 = m.acos(arg_b3)

    theta_2 = b_1 - b_2
    theta_3 = m.pi - b_3

    angulos = angle_corrector([theta_2, theta_3])

    servo_femur.angle = angulos[0] * 180 / m.pi
    servo_tibia.angle = angulos[1] * 180 / m.pi - 8


# ── Posição inicial: centro, chão ─────────────────────────────────────────────
print("Indo para posição inicial...")
move_leg(0, Z_APOIO)
time.sleep(1)

print("Iniciando locomoção. Ctrl+C para parar.\n")
try:
    while True:
        # Fase 1 — Swing: arco senoidal em z (atrás → frente)
        for i, t in enumerate(np.linspace(0, np.pi, N_PONTOS)):
            x = X_ATRAS + (X_FRENTE - X_ATRAS) * i / (N_PONTOS - 1)
            z = Z_APOIO + (Z_SWING - Z_APOIO) * np.sin(t)
            move_leg(x, z)
            time.sleep(DELAY)

        # Fase 2 — Apoio: pé no chão, empurra para trás (frente → atrás)
        for x in np.linspace(X_FRENTE, X_ATRAS, N_PONTOS):
            move_leg(x, Z_APOIO)
            time.sleep(DELAY)

except KeyboardInterrupt:
    print("\nParado. Retornando à posição inicial.")
    move_leg(0, Z_APOIO)
