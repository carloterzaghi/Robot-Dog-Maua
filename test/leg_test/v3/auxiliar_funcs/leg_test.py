import time
import numpy as np
from numpy.linalg import norm
import math as m


def _ease(start, end, n):
    """Interpolação smootherstep (C²): aceleração zero nas extremidades, sem pico de corrente."""
    t = np.linspace(0, 1, n)
    t_smooth = t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
    return start + (end - start) * t_smooth


def _cubic_bezier(P0, P1, P2, P3, n):
    """
    Gera n pontos ao longo de uma curva de Bézier cúbica 2D.
    Cada Pi é um array [x, z].  Retorna (x_array, z_array).

    B(t) = (1-t)³·P0 + 3(1-t)²t·P1 + 3(1-t)t²·P2 + t³·P3,  t ∈ [0, 1]
    """
    P0, P1, P2, P3 = (np.asarray(p, dtype=float) for p in (P0, P1, P2, P3))
    t = np.linspace(0, 1, n).reshape(-1, 1)
    pts = ((1 - t)**3 * P0 +
           3 * (1 - t)**2 * t * P1 +
           3 * (1 - t) * t**2 * P2 +
           t**3 * P3)
    return pts[:, 0], pts[:, 1]


# ── Parâmetros da rampa de inicialização ──────────────────────────────────────
N_RAMP    = 40      # quantidade de passos da rampa
RAMP_DELAY = 0.025  # intervalo entre passos da rampa (s)

# ── Alturas de apoio — ajuste para nivelar o robô ────────────────────────────
# Se a frente parece mais esticada que o trás: aumente Z_APOIO_FRENTE (ex: -210)
# Se o trás parece mais esticado que a frente: aumente Z_APOIO_TRAS  (ex: -210)
Z_APOIO_FRENTE = -210   # mm — pernas da frente
Z_APOIO_TRAS   = -219.8   # mm — pernas de trás


def frente_dir(self, stop_event, use_angular=True, delay_before_descent=0.0, sync_barrier=None, shared_state=None):
    # ── Parâmetros da perna ───────────────────────────────────────────────────────
    UPPER_LEG  = 120
    LOWER_LEG  = 130
    MAX_RADIUS = 220
    MAX_Z      = -80

    # ── Parâmetros da marcha ──────────────────────────────────────────────────────
    Z_APOIO  = Z_APOIO_FRENTE
    Z_SWING  = -154.8
    X_FRENTE =   90
    X_ATRAS  =  -90
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

    # ── Rampa suave de inicialização ─────────────────────────────────────────────
    # Interpola diretamente os ângulos dos servos (posição atual → alvo)
    # usando ease (cosseno) para evitar pico de corrente no arranque.
    print("Indo para posição inicial (rampa suave)...")

    # Calcula ângulos-alvo via IK para o centro (0, Z_APOIO)
    x_init, z_init = 0, Z_APOIO
    if z_init > MAX_Z:
        z_init = MAX_Z
    len_B = norm([x_init, 0, z_init])
    if len_B > MAX_RADIUS:
        scale = MAX_RADIUS / len_B
        x_init *= scale
        z_init *= scale
        len_B = MAX_RADIUS
    arg_b2 = np.clip((UPPER_LEG**2 + len_B**2 - LOWER_LEG**2) / (2 * UPPER_LEG * len_B), -1, 1)
    arg_b3 = np.clip((UPPER_LEG**2 + LOWER_LEG**2 - len_B**2) / (2 * UPPER_LEG * LOWER_LEG), -1, 1)
    b_1 = point_to_rad(x_init, z_init)
    b_2 = m.acos(arg_b2)
    b_3 = m.acos(arg_b3)
    theta_2 = b_1 - b_2
    theta_3 = m.pi - b_3
    angulos_alvo = angle_corrector([theta_2, theta_3])
    femur_alvo = angulos_alvo[0] * 180 / m.pi
    tibia_alvo = angulos_alvo[1] * 180 / m.pi - 8
    angular_alvo = ANG_MIN if use_angular else 100  # 100° = posição fixa sem angular

    # Lê ângulos atuais (fallback para o alvo caso None)
    femur_atual   = self.frente_femur_dir.angle   if self.frente_femur_dir.angle   is not None else femur_alvo
    tibia_atual   = self.frente_tibia_dir.angle   if self.frente_tibia_dir.angle   is not None else tibia_alvo
    angular_atual = self.frente_angular_dir.angle if self.frente_angular_dir.angle is not None else angular_alvo

    # Interpola suavemente cada servo (angular sempre é rampado)
    femur_ramp   = _ease(femur_atual,   femur_alvo,   N_RAMP)
    tibia_ramp   = _ease(tibia_atual,   tibia_alvo,   N_RAMP)
    angular_ramp = _ease(angular_atual, angular_alvo, N_RAMP)

    if delay_before_descent > 0:
        time.sleep(delay_before_descent)

    for i in range(N_RAMP):
        if stop_event.is_set():
            return
        self.frente_femur_dir.angle = femur_ramp[i]
        self.frente_tibia_dir.angle = tibia_ramp[i]
        self.frente_angular_dir.angle = angular_ramp[i]
        time.sleep(RAMP_DELAY)

    if sync_barrier is not None:
        try:
            import threading
            sync_barrier.wait(timeout=5)
        except Exception:
            pass

    print("Iniciando locomoção. Ctrl+C para parar.\n")
    while not stop_event.is_set():
        z_pitch = shared_state.get("z_pitch_frente", 0.0) if shared_state is not None else 0.0
        z_apoio_atual = Z_APOIO + z_pitch
        if shared_state is not None:
            speed = shared_state.get("speed", 0)
            yaw   = shared_state.get("yaw", 0.0)
            if speed == 0 and yaw == 0.0:
                move_leg(0, z_apoio_atual)
                time.sleep(0.05)
                continue
            if abs(yaw) > abs(speed):
                direction = -1 if yaw > 0 else 1  # perna direita recua ao girar para a direita
            else:
                direction = shared_state.get("direction", 1)
        else:
            direction = 1

        # Fase 1 — Swing: curva de Bézier cúbica (atrás → frente)
        # P0 = decolagem, P1 = subida vertical rápida,
        # P2 = alto antes de descer, P3 = pouso suave
        swing_x, swing_z = _cubic_bezier(
            P0=[X_ATRAS * direction,  z_apoio_atual],   # decolagem (chão, atrás)
            P1=[X_ATRAS * direction,  Z_SWING],          # sobe rápido (vertical)
            P2=[X_FRENTE * direction, Z_SWING],          # mantém alto até a frente
            P3=[X_FRENTE * direction, z_apoio_atual],    # pouso (chão, frente)
            n=N_PONTOS,
        )
        ang_swing = np.linspace(ANG_MIN, ANG_MAX, N_PONTOS)
        for i in range(N_PONTOS):
            if stop_event.is_set():
                return
            move_leg(swing_x[i], swing_z[i])
            if use_angular:
                self.frente_angular_dir.angle = ang_swing[i]
            time.sleep(DELAY)

        # Fase 2 — Apoio: pé no chão, empurra para trás (frente → atrás)
        ang_apoio = np.linspace(ANG_MAX, ANG_MIN, N_PONTOS)
        for i, x in enumerate(np.linspace(X_FRENTE * direction, X_ATRAS * direction, N_PONTOS)):
            if stop_event.is_set():
                return
            move_leg(x, z_apoio_atual)
            if use_angular:
                self.frente_angular_dir.angle = ang_apoio[i]
            time.sleep(DELAY)

def frente_esq(self, stop_event, use_angular=True, delay_before_descent=0.0, sync_barrier=None, shared_state=None):
    # ── Parâmetros da perna ───────────────────────────────────────────────────────
    UPPER_LEG  = 120
    LOWER_LEG  = 130
    MAX_RADIUS = 220
    MAX_Z      = -80

    # ── Parâmetros da marcha ──────────────────────────────────────────────────────
    Z_APOIO  = Z_APOIO_FRENTE
    Z_SWING  = -154.8
    X_FRENTE =   90
    X_ATRAS  =  -90
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

    # ── Rampa suave de inicialização ─────────────────────────────────────────────
    # Interpola diretamente os ângulos dos servos (posição atual → alvo)
    # usando ease (cosseno) para evitar pico de corrente no arranque.
    print("Indo para posição inicial (esq, rampa suave)...")

    # Calcula ângulos-alvo via IK para o início do apoio (X_FRENTE, Z_APOIO)
    # A esq começa no apoio (defasada 180° em relação à dir)
    x_init, z_init = 0, Z_APOIO
    if z_init > MAX_Z:
        z_init = MAX_Z
    len_B = norm([x_init, 0, z_init])
    if len_B > MAX_RADIUS:
        scale = MAX_RADIUS / len_B
        x_init *= scale
        z_init *= scale
        len_B = MAX_RADIUS
    arg_b2 = np.clip((UPPER_LEG**2 + len_B**2 - LOWER_LEG**2) / (2 * UPPER_LEG * len_B), -1, 1)
    arg_b3 = np.clip((UPPER_LEG**2 + LOWER_LEG**2 - len_B**2) / (2 * UPPER_LEG * LOWER_LEG), -1, 1)
    b_1 = point_to_rad(x_init, z_init)
    b_2 = m.acos(arg_b2)
    b_3 = m.acos(arg_b3)
    theta_2 = b_1 - b_2
    theta_3 = m.pi - b_3
    angulos_alvo = angle_corrector([theta_2, theta_3])
    # Ângulos espelhados para a perna esquerda
    femur_alvo   = 180 - angulos_alvo[0] * 180 / m.pi
    tibia_alvo   = 180 - (angulos_alvo[1] * 180 / m.pi - 8)
    angular_alvo = ANG_MIN if use_angular else 105  # ANG_MIN = início do apoio (defasado)

    # Lê ângulos atuais (fallback para o alvo caso None)
    femur_atual   = self.frente_femur_esq.angle   if self.frente_femur_esq.angle   is not None else femur_alvo
    tibia_atual   = self.frente_tibia_esq.angle   if self.frente_tibia_esq.angle   is not None else tibia_alvo
    angular_atual = self.frente_angular_esq.angle if self.frente_angular_esq.angle is not None else angular_alvo

    # Interpola suavemente cada servo (angular sempre é rampado)
    femur_ramp   = _ease(femur_atual,   femur_alvo,   N_RAMP)
    tibia_ramp   = _ease(tibia_atual,   tibia_alvo,   N_RAMP)
    angular_ramp = _ease(angular_atual, angular_alvo, N_RAMP)

    if delay_before_descent > 0:
        time.sleep(delay_before_descent)

    for i in range(N_RAMP):
        if stop_event.is_set():
            return
        self.frente_femur_esq.angle = femur_ramp[i]
        self.frente_tibia_esq.angle = tibia_ramp[i]
        self.frente_angular_esq.angle = angular_ramp[i]
        time.sleep(RAMP_DELAY)

    if sync_barrier is not None:
        try:
            import threading
            sync_barrier.wait(timeout=5)
        except Exception:
            pass

    print("Iniciando locomoção (esq — defasada). Ctrl+C para parar.\n")
    while not stop_event.is_set():
        z_pitch = shared_state.get("z_pitch_frente", 0.0) if shared_state is not None else 0.0
        z_apoio_atual = Z_APOIO + z_pitch
        if shared_state is not None:
            speed = shared_state.get("speed", 0)
            yaw   = shared_state.get("yaw", 0.0)
            if speed == 0 and yaw == 0.0:
                move_leg(0, z_apoio_atual)
                time.sleep(0.05)
                continue
            if abs(yaw) > abs(speed):
                direction = 1 if yaw > 0 else -1  # perna esquerda avança ao girar para a direita
            else:
                direction = shared_state.get("direction", 1)
        else:
            direction = 1

        # Fase 1 — Apoio: pé no chão, empurra para trás (frente → atrás)
        # Acontece enquanto a dir está no swing (subindo)
        ang_apoio = np.linspace(ANG_MIN, ANG_MAX, N_PONTOS)
        for i, x in enumerate(np.linspace(X_FRENTE * direction, X_ATRAS * direction, N_PONTOS)):
            if stop_event.is_set():
                return
            move_leg(x, z_apoio_atual)
            if use_angular:
                self.frente_angular_esq.angle = ang_apoio[i]
            time.sleep(DELAY)

        # Fase 2 — Swing: curva de Bézier cúbica (atrás → frente)
        # Acontece enquanto a dir está no apoio (empurrando)
        swing_x, swing_z = _cubic_bezier(
            P0=[X_ATRAS * direction,  z_apoio_atual],   # decolagem (chão, atrás)
            P1=[X_ATRAS * direction,  Z_SWING],          # sobe rápido (vertical)
            P2=[X_FRENTE * direction, Z_SWING],          # mantém alto até a frente
            P3=[X_FRENTE * direction, z_apoio_atual],    # pouso (chão, frente)
            n=N_PONTOS,
        )
        ang_swing = np.linspace(ANG_MAX, ANG_MIN, N_PONTOS)
        for i in range(N_PONTOS):
            if stop_event.is_set():
                return
            move_leg(swing_x[i], swing_z[i])
            if use_angular:
                self.frente_angular_esq.angle = ang_swing[i]
            time.sleep(DELAY)

def tras_dir(self, stop_event, use_angular=True, delay_before_descent=0.0, sync_barrier=None, shared_state=None):
    # ── Parâmetros da perna ───────────────────────────────────────────────────────
    UPPER_LEG  = 120
    LOWER_LEG  = 130
    MAX_RADIUS = 220
    MAX_Z      = -80

    # ── Parâmetros da marcha ──────────────────────────────────────────────────────
    Z_APOIO  = Z_APOIO_TRAS
    Z_SWING  = -154.8
    X_FRENTE =   90
    X_ATRAS  =  -90
    N_PONTOS =   20
    DELAY    = 0.015

    # ── Parâmetros do servo angular ───────────────────────────────────────────────
    # Montado oposto à frente_dir → mesmo espectro de frente_esq
    ANG_MIN  =  90
    ANG_MAX  = 135

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

        # Mesma montagem que frente_dir (lado direito, sem espelhar)
        self.tras_femur_dir.angle = angulos[0] * 180 / m.pi
        self.tras_tibia_dir.angle = angulos[1] * 180 / m.pi - 8

    # ── Rampa suave de inicialização ─────────────────────────────────────────────
    # tras_dir inicia no APOIO — mesmo diagonal que frente_esq (X_FRENTE, Z_APOIO)
    print("Indo para posição inicial - tras_dir (rampa suave)...")

    x_init, z_init = 0, Z_APOIO
    if z_init > MAX_Z:
        z_init = MAX_Z
    len_B = norm([x_init, 0, z_init])
    if len_B > MAX_RADIUS:
        scale = MAX_RADIUS / len_B
        x_init *= scale
        z_init *= scale
        len_B = MAX_RADIUS
    arg_b2 = np.clip((UPPER_LEG**2 + len_B**2 - LOWER_LEG**2) / (2 * UPPER_LEG * len_B), -1, 1)
    arg_b3 = np.clip((UPPER_LEG**2 + LOWER_LEG**2 - len_B**2) / (2 * UPPER_LEG * LOWER_LEG), -1, 1)
    b_1 = point_to_rad(x_init, z_init)
    b_2 = m.acos(arg_b2)
    b_3 = m.acos(arg_b3)
    theta_2 = b_1 - b_2
    theta_3 = m.pi - b_3
    angulos_alvo = angle_corrector([theta_2, theta_3])
    femur_alvo   = angulos_alvo[0] * 180 / m.pi
    tibia_alvo   = angulos_alvo[1] * 180 / m.pi - 8
    angular_alvo = ANG_MIN if use_angular else 100

    femur_atual   = self.tras_femur_dir.angle   if self.tras_femur_dir.angle   is not None else femur_alvo
    tibia_atual   = self.tras_tibia_dir.angle   if self.tras_tibia_dir.angle   is not None else tibia_alvo
    angular_atual = self.tras_angular_dir.angle if self.tras_angular_dir.angle is not None else angular_alvo

    femur_ramp   = _ease(femur_atual,   femur_alvo,   N_RAMP)
    tibia_ramp   = _ease(tibia_atual,   tibia_alvo,   N_RAMP)
    angular_ramp = _ease(angular_atual, angular_alvo, N_RAMP)

    if delay_before_descent > 0:
        time.sleep(delay_before_descent)

    for i in range(N_RAMP):
        if stop_event.is_set():
            return
        self.tras_femur_dir.angle   = femur_ramp[i]
        self.tras_tibia_dir.angle   = tibia_ramp[i]
        self.tras_angular_dir.angle = angular_ramp[i]
        time.sleep(RAMP_DELAY)

    if sync_barrier is not None:
        try:
            import threading
            sync_barrier.wait(timeout=5)
        except Exception:
            pass

    print("Iniciando locomoção - tras_dir (apoio primeiro). Ctrl+C para parar.\n")
    while not stop_event.is_set():
        z_pitch = shared_state.get("z_pitch_tras", 0.0) if shared_state is not None else 0.0
        z_apoio_atual = Z_APOIO + z_pitch
        if shared_state is not None:
            speed = shared_state.get("speed", 0)
            yaw   = shared_state.get("yaw", 0.0)
            if speed == 0 and yaw == 0.0:
                move_leg(0, z_apoio_atual)
                time.sleep(0.05)
                continue
            if abs(yaw) > abs(speed):
                direction = -1 if yaw > 0 else 1  # perna direita recua ao girar para a direita
            else:
                direction = shared_state.get("direction", 1)
        else:
            direction = 1

        # Fase 1 — Apoio: pé no chão, frente → atrás (sincronizado com frente_esq)
        ang_apoio = np.linspace(ANG_MIN, ANG_MAX, N_PONTOS)
        for i, x in enumerate(np.linspace(X_FRENTE * direction, X_ATRAS * direction, N_PONTOS)):
            if stop_event.is_set():
                return
            move_leg(x, z_apoio_atual)
            if use_angular:
                self.tras_angular_dir.angle = ang_apoio[i]
            time.sleep(DELAY)

        # Fase 2 — Swing: Bézier cúbica (atrás → frente)
        swing_x, swing_z = _cubic_bezier(
            P0=[X_ATRAS * direction,  z_apoio_atual],
            P1=[X_ATRAS * direction,  Z_SWING],
            P2=[X_FRENTE * direction, Z_SWING],
            P3=[X_FRENTE * direction, z_apoio_atual],
            n=N_PONTOS,
        )
        ang_swing = np.linspace(ANG_MAX, ANG_MIN, N_PONTOS)
        for i in range(N_PONTOS):
            if stop_event.is_set():
                return
            move_leg(swing_x[i], swing_z[i])
            if use_angular:
                self.tras_angular_dir.angle = ang_swing[i]
            time.sleep(DELAY)


def tras_esq(self, stop_event, use_angular=True, delay_before_descent=0.0, sync_barrier=None, shared_state=None):
    # ── Parâmetros da perna ───────────────────────────────────────────────────────
    UPPER_LEG  = 120
    LOWER_LEG  = 130
    MAX_RADIUS = 220
    MAX_Z      = -80

    # ── Parâmetros da marcha ──────────────────────────────────────────────────────
    Z_APOIO  = Z_APOIO_TRAS
    Z_SWING  = -154.8
    X_FRENTE =   90
    X_ATRAS  =  -90
    N_PONTOS =   20
    DELAY    = 0.015

    # ── Parâmetros do servo angular (espelhado: mesmo espectro de frente_dir) ─────
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

        # Ângulos espelhados — servo montado em direção oposta (lado esquerdo)
        self.tras_femur_esq.angle = 180 - angulos[0] * 180 / m.pi
        self.tras_tibia_esq.angle = 180 - (angulos[1] * 180 / m.pi - 8)

    # ── Rampa suave de inicialização ─────────────────────────────────────────────
    # tras_esq inicia no SWING — mesmo diagonal que frente_dir (X_ATRAS, Z_APOIO)
    print("Indo para posição inicial - tras_esq (rampa suave)...")

    x_init, z_init = 0, Z_APOIO
    if z_init > MAX_Z:
        z_init = MAX_Z
    len_B = norm([x_init, 0, z_init])
    if len_B > MAX_RADIUS:
        scale = MAX_RADIUS / len_B
        x_init *= scale
        z_init *= scale
        len_B = MAX_RADIUS
    arg_b2 = np.clip((UPPER_LEG**2 + len_B**2 - LOWER_LEG**2) / (2 * UPPER_LEG * len_B), -1, 1)
    arg_b3 = np.clip((UPPER_LEG**2 + LOWER_LEG**2 - len_B**2) / (2 * UPPER_LEG * LOWER_LEG), -1, 1)
    b_1 = point_to_rad(x_init, z_init)
    b_2 = m.acos(arg_b2)
    b_3 = m.acos(arg_b3)
    theta_2 = b_1 - b_2
    theta_3 = m.pi - b_3
    angulos_alvo = angle_corrector([theta_2, theta_3])
    # Ângulos espelhados para perna esquerda
    femur_alvo   = 180 - angulos_alvo[0] * 180 / m.pi
    tibia_alvo   = 180 - (angulos_alvo[1] * 180 / m.pi - 8)
    angular_alvo = ANG_MIN if use_angular else 105

    femur_atual   = self.tras_femur_esq.angle   if self.tras_femur_esq.angle   is not None else femur_alvo
    tibia_atual   = self.tras_tibia_esq.angle   if self.tras_tibia_esq.angle   is not None else tibia_alvo
    angular_atual = self.tras_angular_esq.angle if self.tras_angular_esq.angle is not None else angular_alvo

    femur_ramp   = _ease(femur_atual,   femur_alvo,   N_RAMP)
    tibia_ramp   = _ease(tibia_atual,   tibia_alvo,   N_RAMP)
    angular_ramp = _ease(angular_atual, angular_alvo, N_RAMP)

    if delay_before_descent > 0:
        time.sleep(delay_before_descent)

    for i in range(N_RAMP):
        if stop_event.is_set():
            return
        self.tras_femur_esq.angle   = femur_ramp[i]
        self.tras_tibia_esq.angle   = tibia_ramp[i]
        self.tras_angular_esq.angle = angular_ramp[i]
        time.sleep(RAMP_DELAY)

    if sync_barrier is not None:
        try:
            import threading
            sync_barrier.wait(timeout=5)
        except Exception:
            pass

    print("Iniciando locomoção - tras_esq (swing primeiro). Ctrl+C para parar.\n")
    while not stop_event.is_set():
        z_pitch = shared_state.get("z_pitch_tras", 0.0) if shared_state is not None else 0.0
        z_apoio_atual = Z_APOIO + z_pitch
        if shared_state is not None:
            speed = shared_state.get("speed", 0)
            yaw   = shared_state.get("yaw", 0.0)
            if speed == 0 and yaw == 0.0:
                move_leg(0, z_apoio_atual)
                time.sleep(0.05)
                continue
            if abs(yaw) > abs(speed):
                direction = 1 if yaw > 0 else -1  # perna esquerda avança ao girar para a direita
            else:
                direction = shared_state.get("direction", 1)
        else:
            direction = 1

        # Fase 1 — Swing: Bézier cúbica (atrás → frente) — sincronizado com frente_dir
        swing_x, swing_z = _cubic_bezier(
            P0=[X_ATRAS * direction,  z_apoio_atual],
            P1=[X_ATRAS * direction,  Z_SWING],
            P2=[X_FRENTE * direction, Z_SWING],
            P3=[X_FRENTE * direction, z_apoio_atual],
            n=N_PONTOS,
        )
        ang_swing = np.linspace(ANG_MIN, ANG_MAX, N_PONTOS)
        for i in range(N_PONTOS):
            if stop_event.is_set():
                return
            move_leg(swing_x[i], swing_z[i])
            if use_angular:
                self.tras_angular_esq.angle = ang_swing[i]
            time.sleep(DELAY)

        # Fase 2 — Apoio: pé no chão, frente → atrás
        ang_apoio = np.linspace(ANG_MAX, ANG_MIN, N_PONTOS)
        for i, x in enumerate(np.linspace(X_FRENTE * direction, X_ATRAS * direction, N_PONTOS)):
            if stop_event.is_set():
                return
            move_leg(x, z_apoio_atual)
            if use_angular:
                self.tras_angular_esq.angle = ang_apoio[i]
            time.sleep(DELAY)
