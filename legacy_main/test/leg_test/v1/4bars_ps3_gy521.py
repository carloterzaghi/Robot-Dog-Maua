"""
4bars_ps3_gy521.py — Controle da perna via PS3 com estabilização automática GY-521.

Combina os dois sistemas:
  - 4bars_ps3.py  : analógico esquerdo (↑/↓) controla direção e parada da passada.
  - 4bars_gy521.py: filtro de Kalman sobre o MPU-6050 calcula correções de pitch
                    e roll aplicadas a cada passo do ciclo.

Threads:
  - thread_imu     : lê GY-521 a ~50 Hz, aplica Kalman, publica pitch_corr/roll_corr.
  - loop_movimento : executa o ciclo de passada com direção do PS3 e correções do IMU.
  - Main           : lê eventos do controle PS3 e atualiza 'direcao'.

Lógica de estabilização (idêntica ao 4bars_gy521.py):
  - pitch_corr desloca x_medio → compensa inclinação frontal/traseira.
  - roll_corr  ajusta Y_GROUND → compensa inclinação lateral.

Hardware: Raspberry Pi + PCA9685 (ServoKit, I2C) + GY-521 (I2C, 0x68) + PS3 (BLE).
"""

import sys
import os
import math
import time
import threading

# Adiciona a raiz do projeto ao path para importar a lib do PS3
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from lib.connect_ps3_control import ensure_connected

from evdev import InputDevice, ecodes        # type: ignore
from smbus2 import SMBus                     # type: ignore
from adafruit_servokit import ServoKit       # type: ignore

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
STEP_DELAY       = 0.01  # Intervalo (s) entre cada passo do ciclo de passada
HOLD_DELAY       = 0.5   # Tempo (s) de pausa ao retornar à posição inicial
PASSOS_POR_CICLO = 100   # Resolução do ciclo — mais passos = mais suave/lento
JOY_THRESHOLD    = 30    # Zona morta do analógico: ignora variações menores que isso

# ── MPU-6050 / GY-521 — registradores I2C ────────────────────────────────────
MPU6050_ADDR = 0x68   # Endereço I2C padrão do MPU-6050 (AD0 = GND)
PWR_MGMT_1   = 0x6B   # Registrador de gerenciamento de energia
ACCEL_XOUT_H = 0x3B   # Primeiro byte do acelerômetro (X alto)
GYRO_XOUT_H  = 0x43   # Primeiro byte do giroscópio  (X alto)

# ── Filtro de Kalman — parâmetros de ruído ────────────────────────────────────
# Q: variância do processo (integração do giroscópio).
#   Pequeno → confia mais no giroscópio; grande → reage mais ao acelerômetro.
KALMAN_Q = 0.001
# R: variância da medição (acelerômetro).
#   Pequeno → confia mais no acelerômetro; grande → suaviza mais.
KALMAN_R = 0.03

# ── Ganhos de estabilização ───────────────────────────────────────────────────
KP_PITCH = 1.2    # Ganho proporcional: pitch (°) → correção no eixo X (°)
KP_ROLL  = 0.8    # Ganho proporcional: roll  (°) → correção no eixo Y (°)
CORR_MAX = 25.0   # Correção máxima permitida em qualquer eixo (graus)

# ── Filtro de Kalman 1-D ──────────────────────────────────────────────────────

class KalmanAngle:
    """
    Filtro de Kalman unidimensional para estimativa de ângulo a partir de IMU.

    Modelo de estado: θ_k = θ_{k-1} + ω * dt  (integração do giroscópio)

    Etapas:
      1. Predict — integra o giroscópio e aumenta a incerteza P.
      2. Update  — corrige com o ângulo do acelerômetro via ganho de Kalman.

    Parâmetros:
        q (KALMAN_Q): variância do processo — confiança no giroscópio.
        r (KALMAN_R): variância da medição  — confiança no acelerômetro.
    """

    def __init__(self, q: float = KALMAN_Q, r: float = KALMAN_R) -> None:
        self.q     = q
        self.r     = r
        self.angle = 0.0  # estimativa atual do ângulo (graus)
        self.p     = 1.0  # covariância do erro de estimativa

    def update(self, accel_angle: float, gyro_rate: float, dt: float) -> float:
        """
        Executa uma iteração do filtro.

        Parâmetros:
            accel_angle: ângulo do acelerômetro (graus) — medição.
            gyro_rate  : velocidade angular (°/s)       — entrada do modelo.
            dt         : intervalo desde a última chamada (s).

        Retorna:
            Estimativa filtrada do ângulo (graus).
        """
        # Predição: integra giroscópio e propaga incerteza
        angle_pred = self.angle + gyro_rate * dt
        p_pred     = self.p + self.q

        # Atualização: ganho de Kalman corrige com a medição do acelerômetro
        k          = p_pred / (p_pred + self.r)
        self.angle = angle_pred + k * (accel_angle - angle_pred)
        self.p     = (1.0 - k) * p_pred

        return self.angle


# ── Estado compartilhado entre as threads ─────────────────────────────────────
# direcao  : -1 = frente | 0 = parado | 1 = trás  (escrito pelo loop PS3)
# pitch_corr: offset de X calculado pelo IMU       (escrito por thread_imu)
# roll_corr : offset de Y calculado pelo IMU       (escrito por thread_imu)
direcao    = 0
pitch_corr = 0.0
roll_corr  = 0.0
lock       = threading.Lock()  # protege as três variáveis acima


# ── Funções do MPU-6050 ───────────────────────────────────────────────────────

def read_word(bus: SMBus, addr: int, reg: int) -> int:
    """
    Lê dois bytes consecutivos e retorna valor inteiro com sinal (complemento de 2).

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
    Calcula pitch (inclinação frontal/traseira) pelo acelerômetro.
    pitch > 0 → frente caindo; pitch < 0 → traseira caindo. Faixa: ±90°.
    """
    return math.degrees(math.atan2(-ax, math.sqrt(ay ** 2 + az ** 2)))


def compute_roll(ax: float, ay: float, az: float) -> float:
    """
    Calcula roll (inclinação lateral) pelo acelerômetro.
    roll > 0 → lado direito caindo; roll < 0 → lado esquerdo caindo. Faixa: ±90°.
    """
    return math.degrees(math.atan2(ay, math.sqrt(ax ** 2 + az ** 2)))


# ── Funções de movimento ──────────────────────────────────────────────────────

def mover_suave(servo, angulo_inicio, angulo_fim, delay=STEP_DELAY):
    """
    Move um servo grau a grau de angulo_inicio até angulo_fim.

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
    Move motor_x e motor_y sequencialmente a partir de seus ângulos atuais.
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
    Lê o GY-521 a ~50 Hz e publica as correções de estabilização.

    Aplica filtro de Kalman independente para pitch e roll, fundindo giroscópio
    (entrada do modelo) com acelerômetro (medição de correção). Grava pitch_corr
    e roll_corr no estado compartilhado, protegido por lock, para consumo por
    loop_movimento.

    Imprime status a ~5 Hz (a cada 10 leituras) para não sobrecarregar o terminal.
    """
    global pitch_corr, roll_corr

    kf_pitch = KalmanAngle()   # filtro independente para pitch
    kf_roll  = KalmanAngle()   # filtro independente para roll

    t_ant    = time.time()
    contador = 0

    with SMBus(1) as bus:
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)  # sai do modo sleep
        print("GY-521 inicializado (filtro de Kalman ativo).")

        while True:
            t_agora = time.time()
            dt      = max(t_agora - t_ant, 1e-6)  # evita divisão por zero
            t_ant   = t_agora

            # Acelerômetro (escala ±2g → divisor 16384 → resultado em g)
            ax = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H)     / 16384.0
            ay = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 2) / 16384.0
            az = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4) / 16384.0

            # Giroscópio (escala ±250°/s → divisor 131 → resultado em °/s)
            gyro_pitch = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H + 2) / 131.0
            gyro_roll  = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H)     / 131.0

            # Ângulos de referência do acelerômetro (medição para o Kalman)
            pitch_accel = compute_pitch(ax, ay, az)
            roll_accel  = compute_roll(ax, ay, az)

            # Estimativas filtradas via Kalman
            pitch = kf_pitch.update(pitch_accel, gyro_pitch, dt)
            roll  = kf_roll.update(roll_accel,  gyro_roll,  dt)

            # Correção proporcional com clamping:
            #   pitch > 0 → frente caindo → desloca x_medio para trás
            #   roll  > 0 → direita caindo → reduz Y_GROUND efetivo
            pc = max(-CORR_MAX, min(CORR_MAX,  pitch * KP_PITCH))
            rc = max(-CORR_MAX, min(CORR_MAX, -roll  * KP_ROLL))

            with lock:
                pitch_corr = pc
                roll_corr  = rc

            contador += 1
            if contador >= 10:
                print(
                    f"[IMU] Pitch: {pitch:+6.1f}°  Roll: {roll:+6.1f}°  |"
                    f"  corr_x: {pc:+5.1f}°  corr_y: {rc:+5.1f}°"
                )
                contador = 0

            time.sleep(0.02)  # ~50 Hz


def loop_movimento():
    """
    Thread de movimento: ciclo de passada com direção do PS3 e correções do IMU.

    A cada passo:
      1. Lê 'direcao' (PS3) e as correções IMU (pitch_corr, roll_corr) — thread-safe.
      2. Aplica as correções ao ponto médio de X e ao Y_GROUND efetivo.
      3. Calcula a trajetória (mesma fórmula de 4bars.py) e envia aos servos.

    Controle de direção (de 4bars_ps3.py):
      - direcao = -1 → i aumenta → ciclo para frente
      - direcao =  1 → i diminui → ciclo invertido (trás)
      - direcao =  0 → posicao_inicial() e aguarda novo comando

    Correções do IMU (de 4bars_gy521.py):
      - x_medio_eff  = x_medio  + pitch_corr  → compensa pitch
      - y_ground_eff = Y_GROUND + roll_corr   → compensa roll
    """
    global direcao

    # Parâmetros base da trajetória (idênticos ao ciclo_passada de 4bars.py)
    x_medio = (X_BACK + X_FORWARD) / 2
    x_amp   = (X_BACK - X_FORWARD) / 2   # positivo pois X_BACK > X_FORWARD
    y_amp   = Y_GROUND - Y_LIFT           # amplitude total de elevação

    i      = 0     # índice atual no ciclo (0 a PASSOS_POR_CICLO - 1)
    parado = True  # True enquanto o robô está parado / aguardando comando

    while True:
        with lock:
            dir_atual = direcao
            xc        = pitch_corr
            yc        = roll_corr

        if dir_atual == 0:
            # Analógico solto: retorna ao repouso uma única vez por parada
            if not parado:
                # posicao_inicial()
                i = 0
                parado = True

            # Equilíbrio estático: aplica correções do IMU quando parado
            x_bal = max(0, min(180, round(X_CENTER + xc)))
            y_bal = max(0, min(180, round(Y_GROUND + yc)))
            motor_x.angle = x_bal
            motor_y.angle = y_bal
            time.sleep(0.05)
            continue

        if parado:
            # Ao retomar: posiciona motor_x em X_BACK (igual ao ciclo_passada)
            inicio_x = int(motor_x.angle) if motor_x.angle is not None else X_CENTER
            mover_suave(motor_x, inicio_x, X_BACK)
            i = 0
            parado = False

        # t percorre [0, 2π) ao longo do ciclo
        t = 2 * math.pi * i / PASSOS_POR_CICLO

        # Aplica correção de pitch ao ponto médio de X (compensação frontal/traseira)
        x_medio_eff = max(X_FORWARD + 5, min(X_BACK - 5, x_medio + xc))

        # Aplica correção de roll ao Y_GROUND (compensação lateral)
        y_ground_eff = max(Y_LIFT + 10, min(170, Y_GROUND + yc))
        # Amplitude relativa recalculada para o novo piso
        y_amp_eff    = y_ground_eff - Y_LIFT

        # Trajetória — mesma fórmula de 4bars.py:
        #   X cossenoidal: x_medio_eff ± x_amp
        #   Y meia senoide: sobe só durante t ∈ [0, π] (fase de avanço)
        x_angle = x_medio_eff + x_amp * math.cos(t)
        y_angle = y_ground_eff - y_amp_eff * max(0.0, math.sin(t))

        # Clamp final de segurança antes de enviar ao servo
        motor_x.angle = max(0, min(180, round(x_angle)))
        motor_y.angle = max(0, min(180, round(y_angle)))
        time.sleep(STEP_DELAY)

        # Avança ou recua o índice conforme a direção:
        #   dir_atual = -1 → i aumenta (frente)
        #   dir_atual =  1 → i diminui (trás)
        i = (i - dir_atual) % PASSOS_POR_CICLO


# ── Conexão com o controle PS3 ────────────────────────────────────────────────
print("Conectando ao controle PS3...")
caminho = ensure_connected()
if not caminho:
    print("Não foi possível conectar ao controle PS3.")
    sys.exit(1)

gamepad = InputDevice(caminho)
print(f"Controle conectado: {gamepad.name} ({caminho})")

# Descobre os limites reais do eixo ABS_Y reportados pelo dispositivo
abs_info = gamepad.absinfo(ecodes.ABS_Y)
JOY_MIN  = abs_info.min
JOY_MAX  = abs_info.max
JOY_MID  = (JOY_MIN + JOY_MAX) // 2
print(f"Limites do ABS_Y: {JOY_MIN} – {JOY_MAX}  |  Centro: {JOY_MID}")

# ── Inicialização ─────────────────────────────────────────────────────────────
print("Iniciando sistema...")
posicao_inicial()

# Threads daemon: encerram automaticamente quando o processo principal sair
t_imu = threading.Thread(target=thread_imu,     daemon=True)
t_mov = threading.Thread(target=loop_movimento, daemon=True)

t_imu.start()
t_mov.start()

print("\nSistema ativo!")
print(f"Ganhos IMU : KP_PITCH={KP_PITCH}  KP_ROLL={KP_ROLL}  CORR_MAX={CORR_MAX}°")
print("Analógico  : ↑ = frente  |  ↓ = trás  |  Neutro = parar")
print("Ctrl+C para encerrar.\n")

# ── Loop principal: leitura contínua do controle PS3 ─────────────────────────
try:
    for event in gamepad.read_loop():
        # Processa apenas eventos do eixo analógico vertical esquerdo (ABS_Y)
        if event.type == ecodes.EV_ABS and event.code == ecodes.ABS_Y:
            valor = event.value

            if valor < JOY_MID - JOY_THRESHOLD:
                nova_dir = -1   # analógico para frente → perna avança
            elif valor > JOY_MID + JOY_THRESHOLD:
                nova_dir = 1    # analógico para trás → perna recua
            else:
                nova_dir = 0    # zona morta → parado

            with lock:
                direcao = nova_dir

            estado = {-1: "→ FRENTE", 0: "■ PARADO", 1: "← TRÁS"}
            print(f"[PS3] Joystick: {valor:4d}  |  {estado[nova_dir]}")

except KeyboardInterrupt:
    print("\nCtrl+C detectado! Retornando à posição inicial...")
    with lock:
        direcao = 0
    time.sleep(0.3)  # aguarda loop_movimento reconhecer a parada
    posicao_inicial()
    print("Encerrado com segurança.")
