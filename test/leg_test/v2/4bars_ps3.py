"""
4bars_ps3.py — Controle da perna (mecanismo 4 barras v2) via controle PS3.

NOVO MECANISMO v2 (paralelogramo):
  - Servo 0 (ombro) : link superior — eleva/abaixa a perna.
  - Servo 4 (joelho): link inferior — avança/recua a perna.

O analógico esquerdo (eixo Y) controla a direção da passada em tempo real:
  - Analógico para frente (↑): ciclo de passada para frente contínuo.
  - Analógico para trás  (↓): ciclo de passada para trás contínuo.
  - Neutro / zona morta       : para e retorna à posição inicial.

Diferenças em relação à v1:
  Na v1, motor_x (servo 4) fazia avanço/recuo e motor_y (servo 0) fazia
  elevação de forma desacoplada.

  Na v2, a coordenação é diferente por causa do paralelogramo:
    - Fase de balanço: ombro SOBE + joelho AVANÇA (pé no ar).
    - Fase de apoio  : ombro no CHÃO + joelho RECUA (empurra o corpo).

  As equações são:
    joelho(t) = joelho_medio + amp_joelho * cos(t)
    ombro(t)  = OMBRO_CHAO   - amp_ombro  * max(0, sin(t))

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
JOELHO_TRAS   = 125  # Perna totalmente recuada

# ── Parâmetros de movimento ───────────────────────────────────────────────────
STEP_DELAY       = 0.01  # Intervalo (s) entre cada passo do ciclo
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
    Leva a perna à posição de repouso: ombro no chão, joelho centralizado.
    Move os dois servos sequencialmente a partir de seus ângulos atuais.
    """
    inicio_ombro  = int(servo_ombro.angle) if servo_ombro.angle is not None else OMBRO_CENTRO
    inicio_joelho = int(servo_joelho.angle) if servo_joelho.angle is not None else JOELHO_CENTRO
    mover_suave(servo_ombro, inicio_ombro, OMBRO_CHAO)
    mover_suave(servo_joelho, inicio_joelho, JOELHO_CENTRO)


def loop_movimento():
    """
    Thread de movimento: executa o ciclo contínuo da perna (v2 paralelogramo).

    Lê a variável global 'direcao' a cada passo e age conforme:
      - direcao == -1 : incrementa t → ciclo para frente
      - direcao ==  1 : decrementa t → ciclo para trás (sentido inverso)
      - direcao ==  0 : para e retorna à posição inicial

    Trajetória adaptada ao paralelogramo v2:
      - Joelho: onda cossenoidal entre JOELHO_TRAS e JOELHO_FRENTE
      - Ombro : meia senoide — levanta só durante fase de balanço (t ∈ [0, π]),
                mantém o pé no chão durante fase de apoio (t ∈ [π, 2π])

    O índice 'i' é mantido entre mudanças de direção para transição suave.
    """
    global direcao

    # Parâmetros da trajetória — adaptados ao paralelogramo v2
    joelho_medio = (JOELHO_TRAS + JOELHO_FRENTE) / 2
    amp_joelho   = (JOELHO_TRAS - JOELHO_FRENTE) / 2  # positivo
    amp_ombro    = OMBRO_CHAO - OMBRO_ELEVADO          # positivo

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
            # Ao sair do repouso, posiciona servo_joelho em JOELHO_TRAS —
            # ponto de início do ciclo (equivalente ao X_BACK da v1).
            inicio_joelho = int(servo_joelho.angle) if servo_joelho.angle is not None else JOELHO_CENTRO
            mover_suave(servo_joelho, inicio_joelho, JOELHO_TRAS)
            i = 0
            parado = False

        # t é o parâmetro angular do ciclo, variando de 0 a 2π (exclusive).
        t = 2 * math.pi * i / PASSOS_POR_CICLO

        # Trajetória do paralelogramo v2:
        #   Joelho cossenoidal: JOELHO_TRAS em t=0, JOELHO_FRENTE em t=π
        #   Ombro meia senoide: eleva só durante t ∈ [0, π] (fase de balanço)
        joelho_angle = joelho_medio + amp_joelho * math.cos(t)
        ombro_angle  = OMBRO_CHAO   - amp_ombro  * max(0.0, math.sin(t))

        # Arredonda para inteiro pois servo.angle exige valor discreto (graus).
        servo_joelho.angle = round(joelho_angle)
        servo_ombro.angle  = round(ombro_angle)
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
