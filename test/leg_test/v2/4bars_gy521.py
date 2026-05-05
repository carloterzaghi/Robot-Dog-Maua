"""
4bars_gy521.py — Ciclo de passada (4 barras v2) com estabilização via GY-521 (MPU-6050).

NOVO MECANISMO v2 (paralelogramo):
  - Servo 0 (ombro) : link superior — eleva/abaixa a perna.
  - Servo 4 (joelho): link inferior — avança/recua a perna.

Combina o ciclo de passada de 4bars.py (v2) com as leituras do acelerômetro e
giroscópio do GY-521 para manter o chassi nivelado de forma contínua.

Lógica de estabilização (controle proporcional) — adaptada ao paralelogramo:
  - Pitch (inclinação frontal/traseira): desloca o ponto médio do joelho,
    fazendo a perna compensar a inclinação no sentido oposto.
      pitch > 0 (frente caindo) → joelho recua mais → empurra o chassi de volta
      pitch < 0 (traseira caindo) → joelho avança mais → idem
  - Roll (inclinação lateral): ajusta o OMBRO_CHAO efetivo, alterando a altura
    do passo para compensar o desequilíbrio lateral.

Filtragem: filtro de Kalman 1-D independente para pitch e roll.

Threads:
  - thread_imu      : lê o sensor a ~50 Hz, calcula ângulos e correções.
  - thread_movimento: executa o ciclo de passada com as correções aplicadas.

Hardware: Raspberry Pi + PCA9685 (ServoKit, I2C) + GY-521/MPU-6050 (I2C, addr 0x68).
"""

import math
import time
import threading
from smbus2 import SMBus          # type: ignore
from adafruit_servokit import ServoKit  # type: ignore

# ── Controlador de servos (PCA9685, 16 canais, I2C) ───────────────────────────
kit = ServoKit(channels=16)

servo_ombro  = kit.servo[0]  # Servo 0: ombro — elevação (link superior)
servo_joelho = kit.servo[4]  # Servo 4: joelho — extensão (link inferior)

# ── Ângulos de referência (graus) ─────────────────────────────────────────────
# OMBRO (Servo 0): controla a altura da perna
OMBRO_CENTRO  = 90   # Posição central/neutra do ombro
OMBRO_CHAO    = 90   # Pé tocando o chão
OMBRO_ELEVADO = 60   # Perna no ponto mais alto

# JOELHO (Servo 4): controla o avanço/recuo da perna
JOELHO_CENTRO = 90   # Posição central/neutra do joelho
JOELHO_FRENTE = 30   # Perna totalmente avançada
JOELHO_TRAS   = 115  # Perna totalmente recuada

# ── Parâmetros de movimento ───────────────────────────────────────────────────
STEP_DELAY       = 0.01   # Intervalo (s) entre cada passo do ciclo de passada
HOLD_DELAY       = 0.5    # Tempo (s) de pausa ao retornar à posição inicial
PASSOS_POR_CICLO = 100    # Resolução do ciclo — mais passos = mais suave/lento

# ── MPU-6050 / GY-521 — registradores I2C ────────────────────────────────────
MPU6050_ADDR = 0x68   # Endereço I2C padrão do MPU-6050 (AD0 = GND)
PWR_MGMT_1   = 0x6B   # Registrador de gerenciamento de energia
ACCEL_XOUT_H = 0x3B   # Primeiro byte do acelerômetro (X alto)
GYRO_XOUT_H  = 0x43   # Primeiro byte do giroscópio  (X alto)

# ── Filtro de Kalman — parâmetros de ruído ───────────────────────────────────
KALMAN_Q = 0.001
KALMAN_R = 0.03

# ── Ganhos de estabilização ───────────────────────────────────────────────────
KP_PITCH = 1.2    # Ganho proporcional: pitch (°) → correção no joelho (°)
KP_ROLL  = 0.8    # Ganho proporcional: roll  (°) → correção no ombro (°)
CORR_MAX = 25.0   # Correção máxima permitida em qualquer eixo (graus)

# ── Filtro de Kalman (1-D por eixo) ──────────────────────────────────────────

class KalmanAngle:
    """
    Filtro de Kalman unidimensional para estimativa de ângulo a partir de IMU.

    Modelo de estado:
        θ_k = θ_{k-1} + ω * dt   (integração do giroscópio)

    Etapas a cada iteração:
      1. Predict — propaga o estado com o giroscópio e aumenta a incerteza (P).
      2. Update  — corrige com o ângulo do acelerômetro usando o ganho de Kalman.

    Parâmetros de ruído (ajuste conforme o hardware):
        Q (KALMAN_Q): variância do processo — quanto o modelo de integração erra.
        R (KALMAN_R): variância da medição  — quanto o acelerômetro é ruidoso.
    """

    def __init__(self, q: float = KALMAN_Q, r: float = KALMAN_R) -> None:
        self.q     = q
        self.r     = r
        self.angle = 0.0  # estimativa atual do ângulo (graus)
        self.p     = 1.0  # covariância do erro de estimativa

    def update(self, accel_angle: float, gyro_rate: float, dt: float) -> float:
        """
        Executa uma iteração do filtro de Kalman.

        Parâmetros:
            accel_angle: ângulo calculado pelo acelerômetro (graus) — medição.
            gyro_rate  : velocidade angular do giroscópio (°/s)    — entrada.
            dt         : intervalo de tempo desde a última chamada (s).

        Retorna:
            Estimativa filtrada do ângulo (graus).
        """
        # Etapa de predição
        angle_pred = self.angle + gyro_rate * dt
        p_pred = self.p + self.q

        # Etapa de atualização
        k = p_pred / (p_pred + self.r)
        self.angle = angle_pred + k * (accel_angle - angle_pred)
        self.p = (1.0 - k) * p_pred

        return self.angle


# ── Estado compartilhado entre as threads ─────────────────────────────────────
# pitch_corr: offset aplicado ao ponto médio do joelho para compensar inclinação frontal.
# roll_corr : offset aplicado ao OMBRO_CHAO para compensar inclinação lateral.
pitch_corr = 0.0
roll_corr  = 0.0
lock = threading.Lock()


# ── Funções do MPU-6050 ───────────────────────────────────────────────────────

def read_word(bus: SMBus, addr: int, reg: int) -> int:
    """
    Lê dois bytes consecutivos do registrador 'reg' e retorna valor com sinal.

    O MPU-6050 armazena cada medição em dois bytes (complemento de 2).

    Parâmetros:
        bus : instância aberta do SMBus.
        addr: endereço I2C do dispositivo.
        reg : endereço do registrador (byte alto).
    """
    high  = bus.read_byte_data(addr, reg)
    low   = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 65536
    return value


def compute_pitch(ax: float, ay: float, az: float) -> float:
    """
    Calcula o ângulo de pitch (inclinação frontal/traseira) a partir do acelerômetro.
    pitch > 0 → frente caindo; pitch < 0 → traseira caindo. Faixa: ±90°.
    """
    return math.degrees(math.atan2(-ax, math.sqrt(ay ** 2 + az ** 2)))


def compute_roll(ax: float, ay: float, az: float) -> float:
    """
    Calcula o ângulo de roll (inclinação lateral) a partir do acelerômetro.
    roll > 0 → lado direito caindo; roll < 0 → lado esquerdo caindo. Faixa: ±90°.
    """
    return math.degrees(math.atan2(ay, math.sqrt(ax ** 2 + az ** 2)))


# ── Funções de movimento ──────────────────────────────────────────────────────

def mover_suave(servo, angulo_inicio, angulo_fim, delay=STEP_DELAY):
    """
    Move um servo de angulo_inicio até angulo_fim grau a grau.

    Parâmetros:
        servo        : instância do servo (adafruit_servokit.Servo).
        angulo_inicio: ângulo de partida em graus (int ou float).
        angulo_fim   : ângulo de destino em graus (int ou float).
        delay        : intervalo em segundos entre cada passo (padrão STEP_DELAY).
    """
    angulo_inicio = int(angulo_inicio)
    angulo_fim    = int(angulo_fim)
    passo = 1 if angulo_fim > angulo_inicio else -1
    for angulo in range(angulo_inicio, angulo_fim + passo, passo):
        servo.angle = angulo
        time.sleep(delay)


def posicao_inicial():
    """
    Leva a perna à posição de repouso: ombro no chão, joelho centralizado.
    Move servo_ombro e servo_joelho sequencialmente a partir de seus ângulos atuais.
    """
    print("Movendo para posição inicial...")
    inicio_ombro  = int(servo_ombro.angle) if servo_ombro.angle is not None else OMBRO_CENTRO
    inicio_joelho = int(servo_joelho.angle) if servo_joelho.angle is not None else JOELHO_CENTRO
    mover_suave(servo_ombro, inicio_ombro, OMBRO_CHAO)
    mover_suave(servo_joelho, inicio_joelho, JOELHO_CENTRO)
    time.sleep(HOLD_DELAY)


# ── Threads ───────────────────────────────────────────────────────────────────

def thread_imu():
    """
    Lê o GY-521 continuamente a ~50 Hz e atualiza as correções de estabilização.

    Aplica um filtro de Kalman independente para cada eixo (pitch e roll),
    usando o giroscópio como entrada do modelo e o acelerômetro como medição
    de correção.

    Na v2 (paralelogramo), as correções são mapeadas assim:
      - pitch_corr → offset no ponto médio do JOELHO (avanço/recuo)
      - roll_corr  → offset no OMBRO_CHAO (altura da perna)
    """
    global pitch_corr, roll_corr

    kf_pitch = KalmanAngle()
    kf_roll  = KalmanAngle()

    t_ant    = time.time()
    contador = 0

    with SMBus(1) as bus:
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)  # sai do modo sleep
        print("GY-521 inicializado (filtro de Kalman ativo).")

        while True:
            t_agora = time.time()
            dt      = max(t_agora - t_ant, 1e-6)
            t_ant   = t_agora

            # Acelerômetro (em g, escala ±2g → divisor 16384)
            ax = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H)     / 16384.0
            ay = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 2) / 16384.0
            az = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4) / 16384.0

            # Giroscópio (em °/s, escala ±250°/s → divisor 131)
            gyro_pitch = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H + 2) / 131.0
            gyro_roll  = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H)     / 131.0

            # Ângulos de referência do acelerômetro
            pitch_accel = compute_pitch(ax, ay, az)
            roll_accel  = compute_roll(ax, ay, az)

            # Filtro de Kalman
            pitch = kf_pitch.update(pitch_accel, gyro_pitch, dt)
            roll  = kf_roll.update(roll_accel,  gyro_roll,  dt)

            # Correção proporcional com limite máximo:
            #   pitch > 0 → frente caindo → joelho recua mais
            #   roll  > 0 → direita caindo → ombro ajusta altura
            pc = max(-CORR_MAX, min(CORR_MAX,  pitch * KP_PITCH))
            rc = max(-CORR_MAX, min(CORR_MAX, -roll  * KP_ROLL))

            with lock:
                pitch_corr = pc
                roll_corr  = rc

            contador += 1
            if contador >= 10:
                print(
                    f"Pitch: {pitch:+6.1f}°  Roll: {roll:+6.1f}°  |"
                    f"  corr_joelho: {pc:+5.1f}°  corr_ombro: {rc:+5.1f}°"
                )
                contador = 0

            time.sleep(0.02)  # ~50 Hz


def thread_movimento():
    """
    Executa o ciclo de passada v2 continuamente com correções de estabilização.

    Trajetória do paralelogramo com correções IMU:
        joelho(t) = joelho_medio_eff + amp_joelho * cos(t)
        ombro(t)  = ombro_chao_eff   - amp_ombro_eff * max(0, sin(t))

    A cada passo, lê pitch_corr e roll_corr (produzidos por thread_imu):
      - joelho_medio_eff = joelho_medio + pitch_corr → compensa pitch
      - ombro_chao_eff   = OMBRO_CHAO   + roll_corr  → compensa roll
    """
    # Parâmetros base da trajetória v2
    joelho_medio = (JOELHO_TRAS + JOELHO_FRENTE) / 2
    amp_joelho   = (JOELHO_TRAS - JOELHO_FRENTE) / 2
    amp_ombro    = OMBRO_CHAO - OMBRO_ELEVADO

    # Posiciona servo_joelho em JOELHO_TRAS antes de iniciar o ciclo
    inicio_joelho = int(servo_joelho.angle) if servo_joelho.angle is not None else JOELHO_CENTRO
    mover_suave(servo_joelho, inicio_joelho, JOELHO_TRAS)

    i = 0
    while True:
        t = 2 * math.pi * i / PASSOS_POR_CICLO

        with lock:
            jc = pitch_corr
            oc = roll_corr

        # Correção de pitch no joelho: desloca o ponto médio
        joelho_medio_eff = max(JOELHO_FRENTE + 5, min(JOELHO_TRAS - 5, joelho_medio + jc))

        # Correção de roll no ombro: ajusta a altura do piso
        ombro_chao_eff = max(OMBRO_ELEVADO + 10, min(170, OMBRO_CHAO + oc))
        amp_ombro_eff  = ombro_chao_eff - OMBRO_ELEVADO

        # Trajetória do paralelogramo v2 com correções:
        joelho_angle = joelho_medio_eff + amp_joelho * math.cos(t)
        ombro_angle  = ombro_chao_eff   - amp_ombro_eff * max(0.0, math.sin(t))

        # Clamp de segurança
        servo_joelho.angle = max(0, min(180, round(joelho_angle)))
        servo_ombro.angle  = max(0, min(180, round(ombro_angle)))
        time.sleep(STEP_DELAY)

        i = (i + 1) % PASSOS_POR_CICLO


# ── Inicialização e loop principal ────────────────────────────────────────────
print("Iniciando sistema de estabilização (v2 paralelogramo)...")
posicao_inicial()

# Threads daemon
t_imu = threading.Thread(target=thread_imu, daemon=True)
t_mov = threading.Thread(target=thread_movimento, daemon=True)

t_imu.start()
t_mov.start()

print("Sistema ativo. Ctrl+C para encerrar.")
print(f"Ganhos: KP_PITCH={KP_PITCH}  KP_ROLL={KP_ROLL}  CORR_MAX={CORR_MAX}°")
print("Ajuste KP_PITCH e KP_ROLL no topo do arquivo se a compensação for excessiva ou insuficiente.\n")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nCtrl+C detectado! Retornando à posição inicial...")
    posicao_inicial()
    print("Encerrado com segurança.")
