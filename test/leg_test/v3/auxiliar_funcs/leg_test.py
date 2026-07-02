import time
import numpy as np
from numpy.linalg import norm
import math as m

def frente_dir(self, stop_event, use_angular=True):
    # ── Parâmetros da perna ───────────────────────────────────────────────────────
    UPPER_LEG  = 120
    LOWER_LEG  = 130
    MAX_RADIUS = 220
    MAX_Z      = -80

    # ── Parâmetros da marcha ──────────────────────────────────────────────────────
    Z_APOIO  = -150
    Z_SWING  =  -85
    X_FRENTE =   60
    X_ATRAS  =  -60
    N_PONTOS =   20
    DELAY    = 0.015

    # ── Parâmetros do servo angular ───────────────────────────────────────────────
    ANG_MIN  =  70
    ANG_MAX  = 120

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

        self.frente_femur_dir.angle = angulos[0] * 180 / m.pi
        self.frente_tibia_dir.angle = angulos[1] * 180 / m.pi - 8

    print("Indo para posição inicial...")
    move_leg(0, Z_APOIO)
    if use_angular:
        self.frente_angular_dir.angle = ANG_MIN
    time.sleep(1)

    print("Iniciando locomoção. Ctrl+C para parar.\n")
    while not stop_event.is_set():
            # Fase 1 — Swing: arco senoidal em z (atrás → frente)
            ang_swing = np.linspace(ANG_MIN, ANG_MAX, N_PONTOS)
            for i, t in enumerate(np.linspace(0, np.pi, N_PONTOS)):
                if stop_event.is_set():
                    return
                x = X_ATRAS + (X_FRENTE - X_ATRAS) * i / (N_PONTOS - 1)
                z = Z_APOIO + (Z_SWING - Z_APOIO) * np.sin(t)
                move_leg(x, z)
                if use_angular:
                    self.frente_angular_dir.angle = ang_swing[i]
                time.sleep(DELAY)

            # Fase 2 — Apoio: pé no chão, empurra para trás (frente → atrás)
            ang_apoio = np.linspace(ANG_MAX, ANG_MIN, N_PONTOS)
            for i, x in enumerate(np.linspace(X_FRENTE, X_ATRAS, N_PONTOS)):
                if stop_event.is_set():
                    return
                move_leg(x, Z_APOIO)
                if use_angular:
                    self.frente_angular_dir.angle = ang_apoio[i]
                time.sleep(DELAY)

def frente_esq(self, stop_event, use_angular=True):
    # ── Parâmetros da perna ───────────────────────────────────────────────────────
    UPPER_LEG  = 120
    LOWER_LEG  = 130
    MAX_RADIUS = 220
    MAX_Z      = -80

    # ── Parâmetros da marcha ──────────────────────────────────────────────────────
    Z_APOIO  = -150
    Z_SWING  =  -85
    X_FRENTE =   60
    X_ATRAS  =  -60
    N_PONTOS =   20
    DELAY    = 0.015

    # ── Parâmetros do servo angular (espelhado: 180 - valor_dir) ─────────────────
    ANG_MIN  =  90   # espelho de ANG_MAX dir (180 - 120)
    ANG_MAX  = 135   # espelho de ANG_MIN dir (180 - 70)

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

        # Ângulos espelhados — servo montado em direção oposta
        self.frente_femur_esq.angle = 180 - angulos[0] * 180 / m.pi
        self.frente_tibia_esq.angle = 180 - (angulos[1] * 180 / m.pi - 8)

    print("Indo para posição inicial (esq)...")
    move_leg(0, Z_APOIO)
    if use_angular:
        self.frente_angular_esq.angle = ANG_MAX  # posição inicial espelhada
    time.sleep(1)

    print("Iniciando locomoção (esq). Ctrl+C para parar.\n")
    while not stop_event.is_set():
        # Fase 1 — Swing: angular espelhado ANG_MAX → ANG_MIN (110 → 60)
        ang_swing = np.linspace(ANG_MAX, ANG_MIN, N_PONTOS)
        for i, t in enumerate(np.linspace(0, np.pi, N_PONTOS)):
            if stop_event.is_set():
                return
            x = X_ATRAS + (X_FRENTE - X_ATRAS) * i / (N_PONTOS - 1)
            z = Z_APOIO + (Z_SWING - Z_APOIO) * np.sin(t)
            move_leg(x, z)
            if use_angular:
                self.frente_angular_esq.angle = ang_swing[i]
            time.sleep(DELAY)

        # Fase 2 — Apoio: angular espelhado ANG_MIN → ANG_MAX (60 → 110)
        ang_apoio = np.linspace(ANG_MIN, ANG_MAX, N_PONTOS)
        for i, x in enumerate(np.linspace(X_FRENTE, X_ATRAS, N_PONTOS)):
            if stop_event.is_set():
                return
            move_leg(x, Z_APOIO)
            if use_angular:
                self.frente_angular_esq.angle = ang_apoio[i]
            time.sleep(DELAY)

def tras_dir(self, stop_event):
    pass

def tras_esq(self, stop_event):
    pass