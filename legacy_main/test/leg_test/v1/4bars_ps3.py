"""
4bars_ps3.py — Controle da perna (mecanismo 4 barras) via controle PS3.

O analógico esquerdo (eixo Y) controla a direção da passada em tempo real:
  - Analógico para frente (↑): ciclo de passada para frente contínuo.
  - Analógico para trás  (↓): ciclo de passada para trás contínuo.
  - Neutro / zona morta       : para e retorna à posição inicial.

O ciclo de movimento usa a mesma trajetória elíptica de 4bars.py:
    x(t) = x_medio + x_amp * cos(t)
    y(t) = y_medio + y_amp * cos(t)

A direção é controlada pelo sentido de incremento do parâmetro t:
  - Frente: t aumenta de 0 → 2π  (perna recua no chão, avança levantada)
  - Trás  : t diminui de 2π → 0  (ciclo invertido)

Hardware: Raspberry Pi + PCA9685 (via Adafruit ServoKit, I2C) + Controle PS3 (BLE).
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
from adafruit_servokit import ServoKit

# ── Inicialização do controlador de servos (PCA9685, 16 canais, I2C) ──────────
kit = ServoKit(channels=16)

motor_x = kit.servo[4]  # Servo horizontal: controla avanço/recuo da perna
motor_y = kit.servo[0]  # Servo vertical:   controla elevação/abaixamento da perna

# ── Ângulos de referência (graus) ─────────────────────────────────────────────
# Ajuste estes valores conforme a montagem física da perna.
X_CENTER  = 90   # Posição central/neutra do motor_x
Y_GROUND  = 90   # Ângulo do motor_y com o pé tocando o chão
Y_LIFT    = 60   # Ângulo do motor_y com a perna no ponto mais alto
X_FORWARD = 30   # Ângulo do motor_x com a perna totalmente avançada
X_BACK    = 125  # Ângulo do motor_x com a perna totalmente recuada

# ── Parâmetros de movimento ───────────────────────────────────────────────────
STEP_DELAY       = 0.01  # Intervalo (s) entre cada passo do ciclo elíptico
PASSOS_POR_CICLO = 100   # Resolução do ciclo — mais passos = mais suave/lento
JOY_THRESHOLD    = 30    # Zona morta do analógico: ignora variações menores

# ── Estado compartilhado entre a thread de leitura e a de movimento ───────────
# direcao:  -1 = frente  |  0 = parado  |  1 = trás
direcao = 0
lock    = threading.Lock()


# ── Funções de movimento ──────────────────────────────────────────────────────

def mover_suave(servo, angulo_inicio, angulo_fim, delay=STEP_DELAY):
    """
    Move um servo grau a grau de angulo_inicio até angulo_fim.

    Parâmetros:
        servo        : instância do servo (adafruit_servokit.Servo).
        angulo_inicio: ângulo de partida em graus (int ou float).
        angulo_fim   : ângulo de destino em graus (int ou float).
        delay        : intervalo em segundos entre cada passo.
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
    Move os dois servos sequencialmente a partir de seus ângulos atuais.
    """
    inicio_x = int(motor_x.angle) if motor_x.angle is not None else X_CENTER
    inicio_y = int(motor_y.angle) if motor_y.angle is not None else Y_GROUND
    mover_suave(motor_x, inicio_x, X_CENTER)
    mover_suave(motor_y, inicio_y, Y_GROUND)


def loop_movimento():
    """
    Thread de movimento: executa o ciclo contínuo da perna.

    Lê a variável global 'direcao' a cada passo e age conforme:
      - direcao == -1 : incrementa t → ciclo para frente
      - direcao ==  1 : decrementa t → ciclo para trás (sentido inverso)
      - direcao ==  0 : para e retorna à posição inicial

    Usa exatamente a mesma lógica de ciclo_passada em 4bars.py:
      - X: onda cossenoidal entre X_BACK e X_FORWARD
      - Y: meia senoide — levanta só durante a fase de avanço (t ∈ [0, π]),
           permanece no chão durante a fase de apoio (t ∈ [π, 2π])
      - Posicionamento inicial: apenas motor_x vai para X_BACK (mover_suave)

    O índice 'i' é mantido entre mudanças de direção (sem parada) para que
    inverter o analógico seja suave, sem saltos abruptos de posição.
    """
    global direcao

    # Parâmetros da trajetória — idênticos ao ciclo_passada de 4bars.py
    x_medio = (X_BACK + X_FORWARD) / 2
    x_amp   = (X_BACK - X_FORWARD) / 2  # positivo pois X_BACK > X_FORWARD
    y_amp   = Y_GROUND - Y_LIFT          # amplitude total de elevação

    i      = 0     # índice atual no ciclo (0 a PASSOS_POR_CICLO - 1)
    parado = True  # controla se o posicionamento inicial já foi feito

    while True:
        with lock:
            dir_atual = direcao

        if dir_atual == 0:
            # Analógico solto: retorna ao repouso (apenas uma vez por parada)
            if not parado:
                posicao_inicial()
                i = 0
                parado = True
            time.sleep(0.05)
            continue

        if parado:
            # Ao sair do repouso, posiciona apenas motor_x em X_BACK —
            # exatamente como ciclo_passada faz em 4bars.py.
            inicio_x = int(motor_x.angle) if motor_x.angle is not None else X_CENTER
            mover_suave(motor_x, inicio_x, X_BACK)
            i = 0
            parado = False

        # t é o parâmetro angular do ciclo, variando de 0 a 2π (exclusive).
        t = 2 * math.pi * i / PASSOS_POR_CICLO

        # Trajetória idêntica ao ciclo_passada de 4bars.py:
        #   X cossenoidal: X_BACK em t=0, X_FORWARD em t=π
        #   Y meia senoide: levanta só durante t ∈ [0, π] (fase de avanço)
        x_angle = x_medio + x_amp * math.cos(t)
        y_angle = Y_GROUND - y_amp * max(0.0, math.sin(t))

        # Arredonda para inteiro pois servo.angle exige valor discreto (graus).
        motor_x.angle = round(x_angle)
        motor_y.angle = round(y_angle)
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
print("Movendo para posição inicial...")
posicao_inicial()

# Inicia a thread de movimento como daemon para encerrar junto com o processo
t_movimento = threading.Thread(target=loop_movimento, daemon=True)
t_movimento.start()

print("Pronto! Analógico esquerdo ↑ = frente  |  ↓ = trás  |  Neutro = parar. Ctrl+C para sair.")

# ── Loop principal: leitura contínua do controle ──────────────────────────────
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
            print(f"Joystick: {valor:4d}  |  {estado[nova_dir]}")

except KeyboardInterrupt:
    print("\nCtrl+C detectado! Retornando à posição inicial...")
    with lock:
        direcao = 0
    time.sleep(0.3)  # Aguarda a thread de movimento reconhecer a parada
    posicao_inicial()
    print("Encerrado com segurança.")
