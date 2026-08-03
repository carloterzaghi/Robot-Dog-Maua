"""
Estabilização completa do robô (4 pernas) usando pitch e roll (MPU6050 + filtro de Kalman).

Roll  → servos angulares (canais 2, 6, 10, 14) compensam inclinação lateral.
Pitch → servos fêmur e tíbia compensam inclinação frontal via IK variando Z.

Montagem dos servos angulares:
  - frente_dir & tras_esq: faixa 70° a 120° (centro 95°)
  - frente_esq & tras_dir: faixa 90° a 135° (centro 112.5°)
"""
import time
import math
import numpy as np
from numpy.linalg import norm
from smbus2 import SMBus

# ── MPU6050 ────────────────────────────────────────────────────────────────────
MPU6050_ADDR  = 0x68
PWR_MGMT_1    = 0x6B
ACCEL_XOUT_H  = 0x3B
GYRO_XOUT_H   = 0x43

# ── Limites dos servos angulares (roll) ───────────────────────────────────────
ANG_DIR_MIN = 70;   ANG_DIR_MAX = 120
ANG_ESQ_MIN = 90;   ANG_ESQ_MAX = 135

ANG_DIR_CENTER = (ANG_DIR_MIN + ANG_DIR_MAX) / 2   # 95°
ANG_ESQ_CENTER = (ANG_ESQ_MIN + ANG_ESQ_MAX) / 2   # 112.5°
ANG_DIR_RANGE  = (ANG_DIR_MAX - ANG_DIR_MIN) / 2   # 25°
ANG_ESQ_RANGE  = (ANG_ESQ_MAX - ANG_ESQ_MIN) / 2   # 22.5°

# ── Parâmetros da perna (IK) ──────────────────────────────────────────────────
UPPER_LEG  = 120   # mm
LOWER_LEG  = 130   # mm
MAX_RADIUS = 220   # mm
MAX_Z      = -80   # mm (teto de Z, mais perto do corpo)

# ── Parâmetros da estabilização por pitch ─────────────────────────────────────
X_NOMINAL      = 0      # mm — posição X neutra (pé embaixo do ombro)
Z_NOMINAL      = -150   # mm — altura nominal de pé (mesmo que Z_APOIO do leg_test)
Z_PITCH_RANGE  = 40     # mm — faixa máxima de ajuste de Z por pitch

# ── Faixas máximas de ângulo para mapeamento ──────────────────────────────────
ROLL_MAX_DEG  = 30.0
PITCH_MAX_DEG = 30.0

# Frequência do loop de controle (~20 Hz)
LOOP_DELAY = 0.05


# ── Leitura I2C ───────────────────────────────────────────────────────────────
def _read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low  = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value


# ── Filtro de Kalman ──────────────────────────────────────────────────────────
class _KalmanFilter:
    def __init__(self):
        self.Q_angle   = 0.001
        self.Q_bias    = 0.003
        self.R_measure = 0.03
        self.angle     = 0.0
        self.bias      = 0.0
        self.P         = [[0.0, 0.0], [0.0, 0.0]]

    def get_angle(self, new_angle, new_rate, dt):
        rate = new_rate - self.bias
        self.angle += dt * rate
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt
        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]
        y = new_angle - self.angle
        self.angle += K[0] * y
        self.bias  += K[1] * y
        P00 = self.P[0][0]; P01 = self.P[0][1]
        self.P[0][0] -= K[0] * P00
        self.P[0][1] -= K[0] * P01
        self.P[1][0] -= K[1] * P00
        self.P[1][1] -= K[1] * P01
        return self.angle


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


# ── Cinemática Inversa (IK) ──────────────────────────────────────────────────
def _point_to_rad(p1, p2):
    theta = math.atan2(p2, p1)
    return (theta + 2 * math.pi) % (2 * math.pi)


def _angle_corrector(angles):
    angles[1] = angles[0] + angles[1] - 5/4 * math.pi
    angles[0] = angles[0] - math.pi / 2
    return angles


def _ik(x, z):
    """Cinemática inversa: (x, z) em mm → [ângulo_femur, ângulo_tibia] em rad."""
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
    b_1 = _point_to_rad(x, z)
    b_2 = math.acos(arg_b2)
    b_3 = math.acos(arg_b3)
    theta_2 = b_1 - b_2
    theta_3 = math.pi - b_3
    return _angle_corrector([theta_2, theta_3])


def _ik_to_servos_dir(x, z):
    """Retorna (femur°, tibia°) para as pernas do lado DIREITO."""
    ang = _ik(x, z)
    return (ang[0] * 180 / math.pi,
            ang[1] * 180 / math.pi - 8)


def _ik_to_servos_esq(x, z):
    """Retorna (femur°, tibia°) para as pernas do lado ESQUERDO (espelhadas)."""
    ang = _ik(x, z)
    return (180 - ang[0] * 180 / math.pi,
            180 - (ang[1] * 180 / math.pi - 8))


# ── Função principal de estabilização ─────────────────────────────────────────
def stabilize(robot_leg, stop_event):
    """
    Loop de estabilização completa (roll + pitch) nas 4 pernas.

    Roll  → ajusta servos angulares (canais 2, 6, 10, 14).
    Pitch → ajusta fêmur e tíbia via IK variando a altura Z da perna.

    Pitch positivo (nariz para baixo) → frente estende, trás retrai.
    Pitch negativo (nariz para cima)  → frente retrai, trás estende.
    """
    kalman_roll  = _KalmanFilter()
    kalman_pitch = _KalmanFilter()

    with SMBus(1) as bus:
        # Acorda o MPU6050
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
        time.sleep(0.1)
        print("MPU6050 inicializado para estabilização das 4 pernas.\n")

        # Leitura inicial para seed dos filtros de Kalman
        ax = _read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H)     / 16384.0
        ay = _read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 2) / 16384.0
        az = _read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4) / 16384.0

        roll_acc  = math.degrees(math.atan2(-ax, az))
        pitch_acc = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
        kalman_roll.angle  = roll_acc
        kalman_pitch.angle = pitch_acc

        # ── Posição inicial via IK (Z_NOMINAL) ───────────────────────────────
        femur_dir_alvo, tibia_dir_alvo = _ik_to_servos_dir(X_NOMINAL, Z_NOMINAL)
        femur_esq_alvo, tibia_esq_alvo = _ik_to_servos_esq(X_NOMINAL, Z_NOMINAL)

        # ── Rampa suave para posição inicial ──────────────────────────────────
        N_RAMP     = 40
        RAMP_DELAY = 0.025

        # Lê ângulos atuais (fallback para alvo se None)
        f_dir_ang_atual = robot_leg.frente_angular_dir.angle or ANG_DIR_CENTER
        f_esq_ang_atual = robot_leg.frente_angular_esq.angle or ANG_ESQ_CENTER
        t_dir_ang_atual = robot_leg.tras_angular_dir.angle   or ANG_ESQ_CENTER
        t_esq_ang_atual = robot_leg.tras_angular_esq.angle   or ANG_DIR_CENTER

        f_dir_fem_atual = robot_leg.frente_femur_dir.angle   or femur_dir_alvo
        f_dir_tib_atual = robot_leg.frente_tibia_dir.angle   or tibia_dir_alvo
        f_esq_fem_atual = robot_leg.frente_femur_esq.angle   or femur_esq_alvo
        f_esq_tib_atual = robot_leg.frente_tibia_esq.angle   or tibia_esq_alvo

        t_dir_fem_atual = robot_leg.tras_femur_dir.angle     or femur_dir_alvo
        t_dir_tib_atual = robot_leg.tras_tibia_dir.angle     or tibia_dir_alvo
        t_esq_fem_atual = robot_leg.tras_femur_esq.angle     or femur_esq_alvo
        t_esq_tib_atual = robot_leg.tras_tibia_esq.angle     or tibia_esq_alvo

        for step in range(1, N_RAMP + 1):
            if stop_event.is_set():
                return
            t = step / N_RAMP
            s = t * t * t * (t * (t * 6.0 - 15.0) + 10.0)  # smootherstep (C²)

            robot_leg.frente_angular_dir.angle = f_dir_ang_atual + (ANG_DIR_CENTER - f_dir_ang_atual) * s
            robot_leg.frente_angular_esq.angle = f_esq_ang_atual + (ANG_ESQ_CENTER - f_esq_ang_atual) * s
            robot_leg.tras_angular_dir.angle   = t_dir_ang_atual + (ANG_ESQ_CENTER - t_dir_ang_atual) * s
            robot_leg.tras_angular_esq.angle   = t_esq_ang_atual + (ANG_DIR_CENTER - t_esq_ang_atual) * s

            robot_leg.frente_femur_dir.angle   = f_dir_fem_atual + (femur_dir_alvo - f_dir_fem_atual) * s
            robot_leg.frente_tibia_dir.angle   = f_dir_tib_atual + (tibia_dir_alvo - f_dir_tib_atual) * s
            robot_leg.frente_femur_esq.angle   = f_esq_fem_atual + (femur_esq_alvo - f_esq_fem_atual) * s
            robot_leg.frente_tibia_esq.angle   = f_esq_tib_atual + (tibia_esq_alvo - f_esq_tib_atual) * s

            robot_leg.tras_femur_dir.angle     = t_dir_fem_atual + (femur_dir_alvo - t_dir_fem_atual) * s
            robot_leg.tras_tibia_dir.angle     = t_dir_tib_atual + (tibia_dir_alvo - t_dir_tib_atual) * s
            robot_leg.tras_femur_esq.angle     = t_esq_fem_atual + (femur_esq_alvo - t_esq_fem_atual) * s
            robot_leg.tras_tibia_esq.angle     = t_esq_tib_atual + (tibia_esq_alvo - t_esq_tib_atual) * s

            time.sleep(RAMP_DELAY)

        print("Estabilização ativa (4 pernas — roll + pitch). Ctrl+C para parar.\n")

        timer = time.time()

        # ── Loop de controle ──────────────────────────────────────────────────
        while not stop_event.is_set():
            # Leitura do acelerômetro (3 eixos)
            ax = _read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H)     / 16384.0
            ay = _read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 2) / 16384.0
            az = _read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4) / 16384.0

            # Leitura do giroscópio (X → pitch, Y → roll)
            gx = _read_word(bus, MPU6050_ADDR, GYRO_XOUT_H)      / 131.0
            gy = _read_word(bus, MPU6050_ADDR, GYRO_XOUT_H + 2)  / 131.0

            # dt
            now = time.time()
            dt  = now - timer
            timer = now

            # ── Roll via Kalman → servos angulares ────────────────────────────
            roll_acc  = math.degrees(math.atan2(-ax, az))
            roll      = kalman_roll.get_angle(roll_acc, gy, dt)
            roll_norm = _clamp(roll / ROLL_MAX_DEG, -1.0, 1.0)

            ang_frente_dir = _clamp(ANG_DIR_CENTER + roll_norm * ANG_DIR_RANGE,
                                    ANG_DIR_MIN, ANG_DIR_MAX)
            ang_frente_esq = _clamp(ANG_ESQ_CENTER + roll_norm * ANG_ESQ_RANGE,
                                    ANG_ESQ_MIN, ANG_ESQ_MAX)

            # tras_dir se move no mesmo sentido de frente_esq (centro 112.5°)
            # tras_esq se move no mesmo sentido de frente_dir (centro 95°)
            ang_tras_dir   = _clamp(ANG_ESQ_CENTER - roll_norm * ANG_ESQ_RANGE,
                                    ANG_ESQ_MIN, ANG_ESQ_MAX)
            ang_tras_esq   = _clamp(ANG_DIR_CENTER - roll_norm * ANG_DIR_RANGE,
                                    ANG_DIR_MIN, ANG_DIR_MAX)

            # ── Pitch via Kalman → fêmur e tíbia (IK) ────────────────────────
            pitch_acc  = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
            pitch      = kalman_pitch.get_angle(pitch_acc, gx, dt)
            pitch_norm = _clamp(pitch / PITCH_MAX_DEG, -1.0, 1.0)

            # Inversão do Pitch conforme solicitado:
            z_frente = _clamp(Z_NOMINAL + pitch_norm * Z_PITCH_RANGE, -(MAX_RADIUS - 1), MAX_Z)
            z_tras   = _clamp(Z_NOMINAL - pitch_norm * Z_PITCH_RANGE, -(MAX_RADIUS - 1), MAX_Z)


            femur_frente_dir, tibia_frente_dir = _ik_to_servos_dir(X_NOMINAL, z_frente)
            femur_frente_esq, tibia_frente_esq = _ik_to_servos_esq(X_NOMINAL, z_frente)

            femur_tras_dir,   tibia_tras_dir   = _ik_to_servos_dir(X_NOMINAL, z_tras)
            femur_tras_esq,   tibia_tras_esq   = _ik_to_servos_esq(X_NOMINAL, z_tras)

            # ── Aplica nos servos ─────────────────────────────────────────────
            # Roll → angulares
            robot_leg.frente_angular_dir.angle = ang_frente_dir
            robot_leg.frente_angular_esq.angle = ang_frente_esq
            robot_leg.tras_angular_dir.angle   = ang_tras_dir
            robot_leg.tras_angular_esq.angle   = ang_tras_esq

            # Pitch → fêmur e tíbia
            robot_leg.frente_femur_dir.angle   = femur_frente_dir
            robot_leg.frente_tibia_dir.angle   = tibia_frente_dir
            robot_leg.frente_femur_esq.angle   = femur_frente_esq
            robot_leg.frente_tibia_esq.angle   = tibia_frente_esq

            robot_leg.tras_femur_dir.angle     = femur_tras_dir
            robot_leg.tras_tibia_dir.angle     = tibia_tras_dir
            robot_leg.tras_femur_esq.angle     = femur_tras_esq
            robot_leg.tras_tibia_esq.angle     = tibia_tras_esq

            print(f"Roll={roll:+6.1f}° Pitch={pitch:+6.1f}°"
                  f" | Ang FD={ang_frente_dir:5.1f}° FE={ang_frente_esq:5.1f}°"
                  f" TD={ang_tras_dir:5.1f}° TE={ang_tras_esq:5.1f}°", end="\r")

            time.sleep(LOOP_DELAY)

    print()  # nova linha ao sair


def stabilize_angular(stop_event, robot_leg):
    """
    Corrige roll em tempo real ajustando apenas os servos angulares (hip abduction).
    Projetado para rodar em paralelo com as threads de locomoção (use_angular=False),
    que não tocam nos angulares — evitando qualquer conflito de acesso.
    """
    kalman = _KalmanFilter()
    try:
        with SMBus(1) as bus:
            bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
            time.sleep(0.1)

            # Seed do filtro com leitura inicial do acelerômetro
            ax = _read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H)     / 16384.0
            az = _read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4) / 16384.0
            kalman.angle = math.degrees(math.atan2(-ax, az))

            timer = time.time()
            print("[stabilize_angular] Ativo — corrigindo roll durante locomoção.")

            while not stop_event.is_set():
                ax = _read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H)     / 16384.0
                az = _read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4) / 16384.0
                gy = _read_word(bus, MPU6050_ADDR, GYRO_XOUT_H + 2)  / 131.0

                now   = time.time()
                dt    = now - timer
                timer = now

                roll_acc  = math.degrees(math.atan2(-ax, az))
                roll      = kalman.get_angle(roll_acc, gy, dt)
                roll_norm = _clamp(roll / ROLL_MAX_DEG, -1.0, 1.0)

                robot_leg.frente_angular_dir.angle = _clamp(
                    ANG_DIR_CENTER + roll_norm * ANG_DIR_RANGE, ANG_DIR_MIN, ANG_DIR_MAX)
                robot_leg.frente_angular_esq.angle = _clamp(
                    ANG_ESQ_CENTER + roll_norm * ANG_ESQ_RANGE, ANG_ESQ_MIN, ANG_ESQ_MAX)
                robot_leg.tras_angular_dir.angle   = _clamp(
                    ANG_ESQ_CENTER - roll_norm * ANG_ESQ_RANGE, ANG_ESQ_MIN, ANG_ESQ_MAX)
                robot_leg.tras_angular_esq.angle   = _clamp(
                    ANG_DIR_CENTER - roll_norm * ANG_DIR_RANGE, ANG_DIR_MIN, ANG_DIR_MAX)

                time.sleep(LOOP_DELAY)
    except Exception as e:
        print(f"[stabilize_angular] Erro: {e}")
