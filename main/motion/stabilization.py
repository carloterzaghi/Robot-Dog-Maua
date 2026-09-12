"""
motion/stabilization.py — Estabilização do robô (roll + pitch) via MPU6050.

Refatorado para usar core.kinematics (IK, _point_to_rad, _angle_corrector)
em vez de cópias locais. Todas as funções públicas mantêm a mesma assinatura
da versão original para compatibilidade.

Roll  → servos angulares (canais 9, 14, 6, 1) compensam inclinação lateral.
Pitch → servos fêmur e tíbia compensam inclinação frontal via IK variando Z.

Montagem do IMU (PCB — parede lateral DIREITA):
  +X → FRENTE  (frente do robô)
  +Y → BAIXO   (ay = −1g quando nivelado)
  +Z → ESQUERDA

  Mapeamento:
    accel_up        = −ay
    accel_right     = −az   (Z aponta para esquerda, então direita = −Z)
    accel_forward   =  ax   (X aponta para frente)
    gyro_roll_rate  = −gx   (positivo = roll direita; sinal invertido em relação ao eixo físico)
    gyro_pitch_rate = −gz   (positivo = pitch nariz cima; sinal invertido em relação ao eixo físico)
"""

import math
import time

from smbus2 import SMBus  # type: ignore

from core.kinematics import ik_to_servo_angles

# ── Registros MPU6050 ─────────────────────────────────────────────────────────
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

# ── Limites dos servos angulares (roll) ───────────────────────────────────────
ANG_DIR_MIN = 70;  ANG_DIR_MAX = 120   # frente_dir e tras_esq
ANG_ESQ_MIN = 90;  ANG_ESQ_MAX = 135   # frente_esq e tras_dir

ANG_DIR_CENTER = (ANG_DIR_MIN + ANG_DIR_MAX) / 2   # 95°
ANG_ESQ_CENTER = (ANG_ESQ_MIN + ANG_ESQ_MAX) / 2   # 112.5°
ANG_DIR_RANGE  = (ANG_DIR_MAX - ANG_DIR_MIN) / 2   # 25°
ANG_ESQ_RANGE  = (ANG_ESQ_MAX - ANG_ESQ_MIN) / 2   # 22.5°

# ── Parâmetros de estabilização por pitch ─────────────────────────────────────
X_NOMINAL     =   0    # mm — posição X neutra
Z_NOMINAL     = -150   # mm — altura nominal
Z_PITCH_RANGE =  40    # mm — faixa máxima de ajuste por pitch

# ── Faixas máximas para normalização ──────────────────────────────────────────
ROLL_MAX_DEG  = 30.0
PITCH_MAX_DEG = 30.0

# Frequência do loop de controle (~20 Hz)
LOOP_DELAY = 0.05

# ── Compensação de centro de massa ───────────────────────────────────────────────────
# CoM medido a 48 mm ao longo do eixo +Z do IMU (= ESQUERDA do robô na
# montagem atual). Para equilibrar, o corpo deve manter um lean para DIREITA
# (roll negativo na convenção: roll+ = lean esquerda).
#
# Ângulo-alvo: lean_alvo = -atan(COM_OFFSET_Z_MM / |Z_NOMINAL|)
#   = -atan(48 / 150) ≈ -17.7°
#
# Se a correção for na direção errada, inverta o sinal de COM_OFFSET_Z_MM.
COM_OFFSET_Z_MM = 48.0   # mm — deslocamento do CoM no eixo +Z do IMU (ESQUERDA)
_COM_ROLL_TARGET_DEG = -math.degrees(math.atan2(COM_OFFSET_Z_MM, abs(Z_NOMINAL)))


# ── Utilitários I2C ───────────────────────────────────────────────────────────

def _read_word(bus: SMBus, addr: int, reg: int) -> int:
    """Lê um inteiro de 16 bits com sinal do MPU6050."""
    high  = bus.read_byte_data(addr, reg)
    low   = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


# ── Filtro de Kalman ──────────────────────────────────────────────────────────

class _KalmanFilter:
    """Filtro de Kalman 1-D para fusão acelerômetro + giroscópio."""

    def __init__(self) -> None:
        self.Q_angle   = 0.001
        self.Q_bias    = 0.003
        self.R_measure = 0.10   # Aumentado (0.03 → 0.10): reduz ruído na nova montagem
        self.angle     = 0.0
        self.bias      = 0.0
        self.P         = [[0.0, 0.0], [0.0, 0.0]]

    def get_angle(self, new_angle: float, new_rate: float, dt: float) -> float:
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

        p00, p01 = self.P[0][0], self.P[0][1]
        self.P[0][0] -= K[0] * p00
        self.P[0][1] -= K[0] * p01
        self.P[1][0] -= K[1] * p00
        self.P[1][1] -= K[1] * p01

        return self.angle


# ── Helpers internos ──────────────────────────────────────────────────────────

def _read_imu(bus: SMBus) -> tuple[float, float, float, float, float]:
    """Lê acelerômetro (ax, ay, az) e giroscópio (gx, gz) do MPU6050."""
    ax = _read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H)     / 16384.0
    ay = _read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 2) / 16384.0
    az = _read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4) / 16384.0
    gx = _read_word(bus, MPU6050_ADDR, GYRO_XOUT_H)      / 131.0
    gz = _read_word(bus, MPU6050_ADDR, GYRO_XOUT_H + 4)  / 131.0
    return ax, ay, az, gx, gz


def _apply_roll_correction(robot_leg, corr_roll: float) -> None:
    """Aplica a correção de roll nos 4 servos angulares."""
    robot_leg.frente_angular_dir.angle = _clamp(
        ANG_DIR_CENTER + corr_roll * ANG_DIR_RANGE, ANG_DIR_MIN, ANG_DIR_MAX)
    robot_leg.frente_angular_esq.angle = _clamp(
        ANG_ESQ_CENTER + corr_roll * ANG_ESQ_RANGE, ANG_ESQ_MIN, ANG_ESQ_MAX)
    robot_leg.tras_angular_dir.angle   = _clamp(
        ANG_ESQ_CENTER - corr_roll * ANG_ESQ_RANGE, ANG_ESQ_MIN, ANG_ESQ_MAX)
    robot_leg.tras_angular_esq.angle   = _clamp(
        ANG_DIR_CENTER - corr_roll * ANG_DIR_RANGE, ANG_DIR_MIN, ANG_DIR_MAX)


# ── Funções públicas ──────────────────────────────────────────────────────────

def stabilize(robot_leg, stop_event) -> None:
    """
    Loop de estabilização completa (roll + pitch) nas 4 pernas.

    Roll  → ajusta servos angulares.
    Pitch → ajusta fêmur e tíbia via IK variando Z.
    """
    kalman_roll  = _KalmanFilter()
    kalman_pitch = _KalmanFilter()

    with SMBus(1) as bus:
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
        time.sleep(0.1)
        print("MPU6050 inicializado para estabilização das 4 pernas.\n")

        # Seed dos filtros de Kalman com leitura inicial
        ax, ay, az, gx, gz = _read_imu(bus)
        accel_up      = -ay
        accel_right   = -az
        accel_forward =  ax
        kalman_roll.angle  = math.degrees(math.atan2(accel_right, accel_up))
        kalman_pitch.angle = math.degrees(math.atan2(accel_forward, accel_up))

        # Rampa suave para posição inicial
        N_RAMP     = 40
        RAMP_DELAY = 0.025

        femur_dir_alvo, tibia_dir_alvo = ik_to_servo_angles(X_NOMINAL, Z_NOMINAL, mirror=False)
        femur_esq_alvo, tibia_esq_alvo = ik_to_servo_angles(X_NOMINAL, Z_NOMINAL, mirror=True)

        # Lê ângulos atuais (fallback para centro/alvo se None)
        cur = {
            "f_dir_ang": robot_leg.frente_angular_dir.angle or ANG_DIR_CENTER,
            "f_esq_ang": robot_leg.frente_angular_esq.angle or ANG_ESQ_CENTER,
            "t_dir_ang": robot_leg.tras_angular_dir.angle   or ANG_ESQ_CENTER,
            "t_esq_ang": robot_leg.tras_angular_esq.angle   or ANG_DIR_CENTER,
            "f_dir_fem": robot_leg.frente_femur_dir.angle   or femur_dir_alvo,
            "f_dir_tib": robot_leg.frente_tibia_dir.angle   or tibia_dir_alvo,
            "f_esq_fem": robot_leg.frente_femur_esq.angle   or femur_esq_alvo,
            "f_esq_tib": robot_leg.frente_tibia_esq.angle   or tibia_esq_alvo,
            "t_dir_fem": robot_leg.tras_femur_dir.angle     or femur_dir_alvo,
            "t_dir_tib": robot_leg.tras_tibia_dir.angle     or tibia_dir_alvo,
            "t_esq_fem": robot_leg.tras_femur_esq.angle     or femur_esq_alvo,
            "t_esq_tib": robot_leg.tras_tibia_esq.angle     or tibia_esq_alvo,
        }

        for step in range(1, N_RAMP + 1):
            if stop_event.is_set():
                return
            t = step / N_RAMP
            s = t * t * t * (t * (t * 6.0 - 15.0) + 10.0)

            robot_leg.frente_angular_dir.angle = cur["f_dir_ang"] + (ANG_DIR_CENTER - cur["f_dir_ang"]) * s
            robot_leg.frente_angular_esq.angle = cur["f_esq_ang"] + (ANG_ESQ_CENTER - cur["f_esq_ang"]) * s
            robot_leg.tras_angular_dir.angle   = cur["t_dir_ang"] + (ANG_ESQ_CENTER - cur["t_dir_ang"]) * s
            robot_leg.tras_angular_esq.angle   = cur["t_esq_ang"] + (ANG_DIR_CENTER - cur["t_esq_ang"]) * s

            robot_leg.frente_femur_dir.angle   = cur["f_dir_fem"] + (femur_dir_alvo - cur["f_dir_fem"]) * s
            robot_leg.frente_tibia_dir.angle   = cur["f_dir_tib"] + (tibia_dir_alvo - cur["f_dir_tib"]) * s
            robot_leg.frente_femur_esq.angle   = cur["f_esq_fem"] + (femur_esq_alvo - cur["f_esq_fem"]) * s
            robot_leg.frente_tibia_esq.angle   = cur["f_esq_tib"] + (tibia_esq_alvo - cur["f_esq_tib"]) * s
            robot_leg.tras_femur_dir.angle     = cur["t_dir_fem"] + (femur_dir_alvo  - cur["t_dir_fem"]) * s
            robot_leg.tras_tibia_dir.angle     = cur["t_dir_tib"] + (tibia_dir_alvo  - cur["t_dir_tib"]) * s
            robot_leg.tras_femur_esq.angle     = cur["t_esq_fem"] + (femur_esq_alvo  - cur["t_esq_fem"]) * s
            robot_leg.tras_tibia_esq.angle     = cur["t_esq_tib"] + (tibia_esq_alvo  - cur["t_esq_tib"]) * s
            time.sleep(RAMP_DELAY)

        print("Estabilização ativa (4 pernas — roll + pitch). Ctrl+C para parar.\n")
        timer = time.time()

        while not stop_event.is_set():
            ax, ay, az, gx, gz = _read_imu(bus)
            accel_up      = -ay
            accel_right   = -az
            accel_forward =  ax

            now   = time.time()
            dt    = now - timer
            timer = now

            # Roll → angulares
            roll_acc  = math.degrees(math.atan2(accel_right, accel_up))
            roll      = kalman_roll.get_angle(roll_acc, -gx, dt)
            corr_roll = -_clamp((roll - _COM_ROLL_TARGET_DEG) / ROLL_MAX_DEG, -1.0, 1.0)
            _apply_roll_correction(robot_leg, corr_roll)

            # Pitch → fêmur e tíbia via IK
            pitch_acc  = math.degrees(math.atan2(accel_forward, accel_up))
            pitch      = kalman_pitch.get_angle(pitch_acc, -gz, dt)
            pitch_norm = _clamp(pitch / PITCH_MAX_DEG, -1.0, 1.0)

            z_frente = _clamp(Z_NOMINAL + pitch_norm * Z_PITCH_RANGE, -(220 - 1), -80)
            z_tras   = _clamp(Z_NOMINAL - pitch_norm * Z_PITCH_RANGE, -(220 - 1), -80)

            f_dir_fem, f_dir_tib = ik_to_servo_angles(X_NOMINAL, z_frente, mirror=False)
            f_esq_fem, f_esq_tib = ik_to_servo_angles(X_NOMINAL, z_frente, mirror=True)
            t_dir_fem, t_dir_tib = ik_to_servo_angles(X_NOMINAL, z_tras,   mirror=False)
            t_esq_fem, t_esq_tib = ik_to_servo_angles(X_NOMINAL, z_tras,   mirror=True)

            robot_leg.frente_femur_dir.angle = f_dir_fem
            robot_leg.frente_tibia_dir.angle = f_dir_tib
            robot_leg.frente_femur_esq.angle = f_esq_fem
            robot_leg.frente_tibia_esq.angle = f_esq_tib
            robot_leg.tras_femur_dir.angle   = t_dir_fem
            robot_leg.tras_tibia_dir.angle   = t_dir_tib
            robot_leg.tras_femur_esq.angle   = t_esq_fem
            robot_leg.tras_tibia_esq.angle   = t_esq_tib

            print(
                f"Roll={roll:+6.1f}° Pitch={pitch:+6.1f}°"
                f" | Ang FD={robot_leg.frente_angular_dir.angle:5.1f}°"
                f" FE={robot_leg.frente_angular_esq.angle:5.1f}°"
                f" TD={robot_leg.tras_angular_dir.angle:5.1f}°"
                f" TE={robot_leg.tras_angular_esq.angle:5.1f}°",
                end="\r",
            )

            time.sleep(LOOP_DELAY)

    print()  # nova linha ao sair


def stabilize_angular(stop_event, robot_leg) -> None:
    """
    Corrige roll em tempo real ajustando apenas os servos angulares (hip abduction).
    Projetado para rodar em paralelo com as threads de locomoção (use_angular=False).
    """
    kalman = _KalmanFilter()
    try:
        with SMBus(1) as bus:
            bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
            time.sleep(0.1)

            ax, ay, az, gx, gz = _read_imu(bus)
            kalman.angle = math.degrees(math.atan2(-az, -ay))

            timer = time.time()
            print("[stabilize_angular] Ativo — corrigindo roll durante locomoção.")

            while not stop_event.is_set():
                _, ay, az, gx, _ = _read_imu(bus)
                now   = time.time()
                dt    = now - timer
                timer = now

                roll_acc  = math.degrees(math.atan2(-az, -ay))
                roll      = kalman.get_angle(roll_acc, -gx, dt)
                corr_roll = -_clamp((roll - _COM_ROLL_TARGET_DEG) / ROLL_MAX_DEG, -1.0, 1.0)
                _apply_roll_correction(robot_leg, corr_roll)

                time.sleep(LOOP_DELAY)
    except Exception as exc:
        print(f"[stabilize_angular] Erro: {exc}")


def stabilize_full_walking(stop_event, robot_leg, shared_state: dict, sync_barrier=None) -> None:
    """
    Corrige roll e pitch em tempo real durante a locomoção.

    Roll  → servos angulares (hip abduction) diretamente.
    Pitch → atualiza shared_state["z_pitch_frente"/"z_pitch_tras"] para as
            threads de locomoção.

    O referencial "reto" é capturado com o robô em modo sleep e passado via
    shared_state["imu_roll_offset"] e shared_state["imu_pitch_offset"].
    """
    kalman_roll  = _KalmanFilter()
    kalman_pitch = _KalmanFilter()
    try:
        with SMBus(1) as bus:
            bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
            time.sleep(0.1)

            roll_offset  = shared_state.get("imu_roll_offset",  0.0)
            pitch_offset = shared_state.get("imu_pitch_offset", 0.0)

            if sync_barrier is not None:
                try:
                    sync_barrier.wait(timeout=5)
                except Exception:
                    pass

            ax, ay, az, gx, gz = _read_imu(bus)
            kalman_roll.angle  = math.degrees(math.atan2(-az,  -ay)) - roll_offset
            kalman_pitch.angle = math.degrees(math.atan2( ax,  -ay)) - pitch_offset
            kalman_roll.bias   = -gx
            kalman_pitch.bias  = -gz

            timer = time.time()
            print("[stabilize_full_walking] Ativo — corrigindo roll + pitch durante locomoção.")

            while not stop_event.is_set():
                ax, ay, az, gx, gz = _read_imu(bus)
                now   = time.time()
                dt    = now - timer
                timer = now

                roll_acc  = math.degrees(math.atan2(-az,  -ay)) - roll_offset
                roll      = kalman_roll.get_angle(roll_acc, -gx, dt)
                corr_roll = -_clamp((roll - _COM_ROLL_TARGET_DEG) / ROLL_MAX_DEG, -1.0, 1.0)
                _apply_roll_correction(robot_leg, corr_roll)

                pitch_acc  = math.degrees(math.atan2( ax,  -ay)) - pitch_offset
                pitch      = kalman_pitch.get_angle(pitch_acc, -gz, dt)
                pitch_norm = _clamp(pitch / PITCH_MAX_DEG, -1.0, 1.0)

                shared_state["z_pitch_frente"] = _clamp( pitch_norm * Z_PITCH_RANGE, -Z_PITCH_RANGE, Z_PITCH_RANGE)
                shared_state["z_pitch_tras"]   = _clamp(-pitch_norm * Z_PITCH_RANGE, -Z_PITCH_RANGE, Z_PITCH_RANGE)

                time.sleep(LOOP_DELAY)
    except Exception as exc:
        print(f"[stabilize_full_walking] Erro: {exc}")
    finally:
        shared_state["z_pitch_frente"] = 0.0
        shared_state["z_pitch_tras"]   = 0.0
