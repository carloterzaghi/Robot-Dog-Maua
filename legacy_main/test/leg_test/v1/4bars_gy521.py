"""
4bars_gy521.py — Ciclo de passada (4 barras) com estabilização via GY-521 (MPU-6050).

Combina o ciclo de passada de 4bars.py com as leituras do acelerômetro e
giroscópio do GY-521 para manter o chassi nivelado de forma contínua.

Lógica de estabilização (controle proporcional):
  - Pitch (inclinação frontal/traseira): desloca o ponto médio do eixo X,
    fazendo a perna compensar a inclinação no sentido oposto.
      pitch > 0 (frente caindo) → perna recua mais → empurra o chassi de volta
      pitch < 0 (traseira caindo) → perna avança mais → idem
  - Roll (inclinação lateral): ajusta o Y_GROUND efetivo, alterando a altura
    do passo para compensar o desequilíbrio lateral.

Filtragem (filtro de Kalman por eixo):
  Um filtro de Kalman 1-D independente para pitch e roll funde o giroscópio
  (entrada do modelo de estado) com o acelerômetro (medição de correção),
  eliminando simultaneamente o drift do giroscópio e o ruído do acelerômetro.
  Os parâmetros Q (ruído de processo) e R (ruído de medição) controlam o
  equilíbrio entre responsividade e suavidade — ajuste no topo do arquivo.

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

motor_x = kit.servo[4]  # Servo horizontal: controla avanço/recuo da perna
motor_y = kit.servo[0]  # Servo vertical:   controla elevação/abaixamento da perna

# ── Ângulos de referência (graus) ─────────────────────────────────────────────
# Ajuste estes valores conforme a montagem física da perna.
X_CENTER  = 90   # Posição central/neutra do motor_x
Y_GROUND  = 90   # Ângulo do motor_y com o pé tocando o chão
Y_LIFT    = 60   # Ângulo do motor_y com a perna no ponto mais alto
X_FORWARD = 30   # Ângulo do motor_x com a perna totalmente avançada
X_BACK    = 115  # Ângulo do motor_x com a perna totalmente recuada

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
# Q (ruído de processo): variância do modelo de integração do giroscópio.
#   Valor pequeno → confia mais no giroscópio; grande → reage mais ao acelerômetro.
KALMAN_Q = 0.001
# R (ruído de medição): variância do acelerômetro.
#   Valor pequeno → confia mais no acelerômetro; grande → suaviza mais.
KALMAN_R = 0.03

# ── Ganhos de estabilização ───────────────────────────────────────────────────
KP_PITCH = 1.2    # Ganho proporcional: pitch (°) → correção no eixo X (°)
KP_ROLL  = 0.8    # Ganho proporcional: roll  (°) → correção no eixo Y (°)
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

    Quanto menor Q em relação a R, mais o filtro confia no giroscópio (suave,
    mas sujeito a drift). Quanto maior Q, mais reage ao acelerômetro (responsivo,
    mas suscetível a vibrações).
    """

    def __init__(self, q: float = KALMAN_Q, r: float = KALMAN_R) -> None:
        self.q     = q    # variância do ruído de processo
        self.r     = r    # variância do ruído de medição
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
        # ── Etapa de predição ─────────────────────────────────────────────────
        # Integra a velocidade angular para prever o novo ângulo
        angle_pred = self.angle + gyro_rate * dt
        # Propaga a covariância do erro (incerteza cresce com o tempo)
        p_pred = self.p + self.q

        # ── Etapa de atualização ──────────────────────────────────────────────
        # Ganho de Kalman: pondera predição vs. medição conforme as incertezas
        k = p_pred / (p_pred + self.r)
        # Corrige a estimativa com a inovação (diferença entre medição e predição)
        self.angle = angle_pred + k * (accel_angle - angle_pred)
        # Atualiza a covariância do erro (reduz após incorporar a medição)
        self.p = (1.0 - k) * p_pred

        return self.angle


# ── Estado compartilhado entre as threads ─────────────────────────────────────
# pitch_corr: offset aplicado ao ponto médio de X para compensar inclinação frontal.
# roll_corr : offset aplicado ao Y_GROUND para compensar inclinação lateral.
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

    Convenção: pitch > 0 → frente caindo; pitch < 0 → traseira caindo.
    Faixa: −90° a +90°.
    """
    return math.degrees(math.atan2(-ax, math.sqrt(ay ** 2 + az ** 2)))


def compute_roll(ax: float, ay: float, az: float) -> float:
    """
    Calcula o ângulo de roll (inclinação lateral) a partir do acelerômetro.

    Convenção: roll > 0 → lado direito caindo; roll < 0 → lado esquerdo caindo.
    Faixa: −90° a +90°.
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
    Leva a perna à posição central de repouso (X_CENTER, Y_GROUND).

    Move motor_x e motor_y sequencialmente a partir de seus ângulos atuais,
    garantindo uma transição suave independentemente do estado anterior.
    """
    print("Movendo para posição inicial...")
    inicio_x = int(motor_x.angle) if motor_x.angle is not None else X_CENTER
    inicio_y = int(motor_y.angle) if motor_y.angle is not None else Y_GROUND
    mover_suave(motor_x, inicio_x, X_CENTER)
    mover_suave(motor_y, inicio_y, Y_GROUND)
    time.sleep(HOLD_DELAY)


# ── Threads ───────────────────────────────────────────────────────────────────

def thread_imu():
    """
    Lê o GY-521 continuamente a ~50 Hz e atualiza as correções de estabilização.

    Aplica um filtro de Kalman independente para cada eixo (pitch e roll),
    usando o giroscópio como entrada do modelo e o acelerômetro como medição
    de correção. Isso elimina o drift do giroscópio e o ruído do acelerômetro
    simultaneamente, produzindo estimativas mais precisas que o filtro complementar.

    As correções resultantes (pitch_corr, roll_corr) são gravadas na variável
    global protegida por lock para serem consumidas por thread_movimento.

    Imprime o estado atual a cada 10 leituras (~5 Hz) para não sobrecarregar o
    terminal.
    """
    global pitch_corr, roll_corr

    # Uma instância de KalmanAngle por eixo — estados completamente independentes
    kf_pitch = KalmanAngle()
    kf_roll  = KalmanAngle()

    t_ant    = time.time()
    contador = 0

    with SMBus(1) as bus:
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)  # sai do modo sleep
        print("GY-521 inicializado (filtro de Kalman ativo).")

        while True:
            t_agora = time.time()
            dt      = max(t_agora - t_ant, 1e-6)  # evita divisão por zero
            t_ant   = t_agora

            # Leitura do acelerômetro (em g, escala ±2g → divisor 16384)
            ax = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H)     / 16384.0
            ay = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 2) / 16384.0
            az = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4) / 16384.0

            # Leitura do giroscópio (em °/s, escala ±250°/s → divisor 131)
            # gyro_y integra o pitch; gyro_x integra o roll
            gyro_pitch = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H + 2) / 131.0
            gyro_roll  = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H)     / 131.0

            # Ângulos de referência do acelerômetro (medição para o Kalman)
            pitch_accel = compute_pitch(ax, ay, az)
            roll_accel  = compute_roll(ax, ay, az)

            # Filtro de Kalman: funde giroscópio (entrada) + acelerômetro (medição)
            pitch = kf_pitch.update(pitch_accel, gyro_pitch, dt)
            roll  = kf_roll.update(roll_accel,  gyro_roll,  dt)

            # Correção proporcional com limite máximo:
            #   pitch > 0 → frente caindo → correção positiva empurra perna para trás
            #   roll  > 0 → lado direito caindo → correção negativa sobe a perna
            pc = max(-CORR_MAX, min(CORR_MAX,  pitch * KP_PITCH))
            rc = max(-CORR_MAX, min(CORR_MAX, -roll  * KP_ROLL))

            with lock:
                pitch_corr = pc
                roll_corr  = rc

            # Imprime status a ~5 Hz para não travar o terminal
            contador += 1
            if contador >= 10:
                print(
                    f"Pitch: {pitch:+6.1f}°  Roll: {roll:+6.1f}°  |"
                    f"  corr_x: {pc:+5.1f}°  corr_y: {rc:+5.1f}°"
                )
                contador = 0

            time.sleep(0.02)  # ~50 Hz


def thread_movimento():
    """
    Executa o ciclo de passada continuamente com correções de estabilização.

    Usa exatamente a mesma trajetória de 4bars.py:
        x(t) = x_medio_eff + x_amp * cos(t)
        y(t) = y_ground_eff - y_amp_eff * max(0, sin(t))

    A cada passo, lê pitch_corr e roll_corr (produzidos por thread_imu) e as
    aplica como offsets:
      - x_medio_eff = x_medio + pitch_corr  → desloca o ponto de equilíbrio de X
      - y_ground_eff = Y_GROUND + roll_corr  → ajusta a altura do passo

    Ambos os offsets são limitados para não ultrapassar os ângulos físicos seguros.
    """
    # Parâmetros base da trajetória — idênticos ao ciclo_passada de 4bars.py
    x_medio = (X_BACK + X_FORWARD) / 2
    x_amp   = (X_BACK - X_FORWARD) / 2   # positivo pois X_BACK > X_FORWARD
    y_amp   = Y_GROUND - Y_LIFT           # amplitude total de elevação

    # Posiciona motor_x em X_BACK antes de iniciar o ciclo (igual ao 4bars.py)
    inicio_x = int(motor_x.angle) if motor_x.angle is not None else X_CENTER
    mover_suave(motor_x, inicio_x, X_BACK)

    i = 0  # índice do passo atual no ciclo
    while True:
        # t percorre [0, 2π) uniformemente ao longo do ciclo
        t = 2 * math.pi * i / PASSOS_POR_CICLO

        # Lê as correções atuais de forma segura
        with lock:
            xc = pitch_corr
            yc = roll_corr

        # Aplica correção ao ponto médio de X (compensação de pitch),
        # limitando para que os ângulos resultantes fiquem dentro do range físico
        x_medio_eff = max(X_FORWARD + 5, min(X_BACK - 5, x_medio + xc))

        # Aplica correção ao Y_GROUND (compensação de roll),
        # garantindo que a amplitude de elevação continue positiva
        y_ground_eff = max(Y_LIFT + 10, min(170, Y_GROUND + yc))
        y_amp_eff    = y_ground_eff - Y_LIFT  # amplitude relativa ao novo piso

        # Trajetória — mesma fórmula de 4bars.py:
        #   X cossenoidal: x_medio_eff ± x_amp
        #   Y meia senoide: eleva só durante t ∈ [0, π] (fase de avanço)
        x_angle = x_medio_eff + x_amp * math.cos(t)
        y_angle = y_ground_eff - y_amp_eff * max(0.0, math.sin(t))

        # Clamp final de segurança antes de enviar ao servo
        motor_x.angle = max(0, min(180, round(x_angle)))
        motor_y.angle = max(0, min(180, round(y_angle)))
        time.sleep(STEP_DELAY)

        i = (i + 1) % PASSOS_POR_CICLO


# ── Inicialização e loop principal ────────────────────────────────────────────
print("Iniciando sistema de estabilização...")
posicao_inicial()

# Threads daemon: encerram automaticamente quando o processo principal sair
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
