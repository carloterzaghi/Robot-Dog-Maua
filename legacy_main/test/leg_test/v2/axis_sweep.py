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
UPPER_LEG  = 120
LOWER_LEG  = 130
MAX_RADIUS = 220
MAX_Z      = -80   # mm — z não pode ser maior que -80

N_PONTOS   = 50    # pontos de interpolação por varredura
DELAY      = 0.02  # s — tempo entre pontos


def point_to_rad(p1, p2):
    theta = m.atan2(p2, p1)
    return (theta + 2 * m.pi) % (2 * m.pi)


def angle_corrector(angles):
    angles[1] = angles[0] + angles[1] - 5/4 * m.pi
    angles[0] = angles[0] - m.pi / 2
    return angles


def move_leg(x, z):
    if z > MAX_Z:
        z = MAX_Z

    len_B = norm([x, 0, z])
    if len_B > MAX_RADIUS:
        scale = MAX_RADIUS / len_B
        x *= scale
        z *= scale
        len_B = MAX_RADIUS

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


# ── Loop principal ────────────────────────────────────────────────────────────
while True:
    print("\nEixo de movimento (x / z):")
    eixo = input().strip().lower()

    if eixo == 'z':
        print("Valor fixo de x (mm):")
        try:
            x_fixo = float(input())
        except ValueError:
            print("[ERRO] Valor inválido.")
            continue

        # Range válido de z: z <= MAX_Z e len_B <= MAX_RADIUS
        z_topo = MAX_Z                                                   # -80 mm (mais alto permitido)
        z_fundo = -np.sqrt(max(0.0, MAX_RADIUS**2 - x_fixo**2))        # limite pelo raio

        if z_fundo > z_topo:
            print(f"[ERRO] x={x_fixo} mm excede o raio máximo para qualquer z válido.")
            continue

        print(f"Percorrendo z de {z_fundo:.1f} até {z_topo:.1f} mm  (x fixo = {x_fixo} mm)")
        print("Ctrl+C para parar e voltar ao menu.\n")

        try:
            while True:
                for z in np.linspace(z_fundo, z_topo, N_PONTOS):
                    move_leg(x_fixo, z)
                    time.sleep(DELAY)
                for z in np.linspace(z_topo, z_fundo, N_PONTOS):
                    move_leg(x_fixo, z)
                    time.sleep(DELAY)
        except KeyboardInterrupt:
            print("\nParado.")

    elif eixo == 'x':
        print("Valor fixo de z (mm):")
        try:
            z_fixo = float(input())
        except ValueError:
            print("[ERRO] Valor inválido.")
            continue

        if z_fixo > MAX_Z:
            z_fixo = MAX_Z
            print(f"[AVISO] z ajustado para {MAX_Z} mm")

        # Range válido de x: len_B <= MAX_RADIUS
        x_max = np.sqrt(max(0.0, MAX_RADIUS**2 - z_fixo**2))
        x_min = -x_max

        print(f"Percorrendo x de {x_min:.1f} até {x_max:.1f} mm  (z fixo = {z_fixo} mm)")
        print("Ctrl+C para parar e voltar ao menu.\n")

        try:
            while True:
                for x in np.linspace(x_min, x_max, N_PONTOS):
                    move_leg(x, z_fixo)
                    time.sleep(DELAY)
                for x in np.linspace(x_max, x_min, N_PONTOS):
                    move_leg(x, z_fixo)
                    time.sleep(DELAY)
        except KeyboardInterrupt:
            print("\nParado.")

    else:
        print("[ERRO] Eixo inválido. Use 'x' ou 'z'.")
