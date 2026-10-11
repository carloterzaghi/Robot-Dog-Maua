"""
4bars.py — Teste do mecanismo de 4 barras para uma perna do Robot Dog Mauá.

Este script controla dois servos que compõem o mecanismo de 4 barras de uma
perna do robô, permitindo testar individualmente cada servo e executar um
ciclo de passada completo.

O ciclo de passada usa uma trajetória senoidal assimétrica:
  - Motor X (horizontal): oscila em cosseno entre X_BACK e X_FORWARD.
  - Motor Y (vertical)  : levanta suavemente (meia senoide positiva de sin(t))
                          apenas durante a fase de avanço (t ∈ [0, π]),
                          permanecendo no chão durante a fase de apoio (t ∈ [π, 2π]).

Como executar:
    python 4bars.py

Hardware: Raspberry Pi + PCA9685 (via Adafruit ServoKit, I2C).
"""

import math
import time
import threading
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

# ── Parâmetros de tempo ───────────────────────────────────────────────────────
STEP_DELAY = 0.01  # Intervalo (s) entre cada passo no movimento suave
HOLD_DELAY = 0.5   # Tempo (s) de pausa nos extremos após cada teste individual


# ── Funções utilitárias ───────────────────────────────────────────────────────

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
    angulo_fim = int(angulo_fim)
    passo = 1 if angulo_fim > angulo_inicio else -1
    for angulo in range(angulo_inicio, angulo_fim + passo, passo):
        servo.angle = angulo
        time.sleep(delay)


def posicao_inicial():
    """
    Leva a perna à posição central de repouso (X_CENTER, Y_GROUND).

    Move motor_x e motor_y sequencialmente a partir de seus ângulos atuais,
    garantindo uma transição suave independentemente do estado anterior.
    Aguarda HOLD_DELAY ao final para estabilizar a posição.
    """
    print("Movendo para posição inicial...")
    inicio_x = int(motor_x.angle) if motor_x.angle is not None else X_CENTER
    inicio_y = int(motor_y.angle) if motor_y.angle is not None else Y_GROUND
    mover_suave(motor_x, inicio_x, X_CENTER)
    mover_suave(motor_y, inicio_y, Y_GROUND)
    time.sleep(HOLD_DELAY)


def testar_motor_x():
    """
    Percorre o range completo do motor_x para verificar seu funcionamento.

    Sequência: X_CENTER → X_FORWARD → X_BACK → X_CENTER.
    Útil para validar os limites físicos e calibrar X_FORWARD/X_BACK.
    """
    print("\n--- Teste motor_x (horizontal) ---")
    print(f"Centro -> Avança ({X_FORWARD}°) -> Recua ({X_BACK}°) -> Centro")
    mover_suave(motor_x, X_CENTER, X_FORWARD)
    time.sleep(HOLD_DELAY)
    mover_suave(motor_x, X_FORWARD, X_BACK)
    time.sleep(HOLD_DELAY)
    mover_suave(motor_x, X_BACK, X_CENTER)
    time.sleep(HOLD_DELAY)
    print("motor_x OK")


def testar_motor_y():
    """
    Percorre o range completo do motor_y para verificar seu funcionamento.

    Sequência: Y_GROUND → Y_LIFT → Y_GROUND.
    Útil para validar os limites físicos e calibrar Y_LIFT.
    """
    print("\n--- Teste motor_y (vertical) ---")
    print(f"Chão ({Y_GROUND}°) -> Levanta ({Y_LIFT}°) -> Chão ({Y_GROUND}°)")
    mover_suave(motor_y, Y_GROUND, Y_LIFT)
    time.sleep(HOLD_DELAY)
    mover_suave(motor_y, Y_LIFT, Y_GROUND)
    time.sleep(HOLD_DELAY)
    print("motor_y OK")


def mover_juntos(servo_a, inicio_a, fim_a, servo_b, inicio_b, fim_b, delay=STEP_DELAY):
    """
    Move dois servos de forma simultânea usando threads paralelas.

    Cada servo executa mover_suave em sua própria thread; a função retorna
    somente quando ambas as threads terminam.

    Parâmetros:
        servo_a  : primeiro servo.
        inicio_a : ângulo inicial do servo_a.
        fim_a    : ângulo final do servo_a.
        servo_b  : segundo servo.
        inicio_b : ângulo inicial do servo_b.
        fim_b    : ângulo final do servo_b.
        delay    : intervalo em segundos entre cada passo (padrão STEP_DELAY).
    """
    t_a = threading.Thread(target=mover_suave, args=(servo_a, inicio_a, fim_a, delay))
    t_b = threading.Thread(target=mover_suave, args=(servo_b, inicio_b, fim_b, delay))
    t_a.start()
    t_b.start()
    t_a.join()
    t_b.join()


def ciclo_passada(repeticoes=3, passos_por_ciclo=100):
    """
    Executa um ciclo de passada fluido usando trajetória senoidal assimétrica.

    Equações de movimento:
        x(t) = x_medio + x_amp * cos(t)
        y(t) = Y_GROUND - y_amp * max(0, sin(t))

    O eixo X oscila continuamente em cosseno (simétrico). O eixo Y usa apenas
    a metade positiva do seno, de modo que a perna só se eleva durante a fase
    de avanço e permanece exatamente em Y_GROUND durante a fase de apoio:
        t ∈ [0, π]  → fase de balanço: perna avança e sobe
        t ∈ [π, 2π] → fase de apoio:   perna recua rente ao chão

    Antes do ciclo, apenas motor_x é reposicionado em X_BACK via mover_suave;
    motor_y já deve estar em Y_GROUND (posição de repouso).

    Parâmetros:
        repeticoes      : número de ciclos completos a executar (padrão 3).
        passos_por_ciclo: resolução do ciclo — mais passos = movimento mais
                          suave, porém mais lento (padrão 100).
    """
    print(f"\n--- Ciclo de passada fluido ({repeticoes} repetições) ---")

    # Ponto médio e amplitude da oscilação horizontal
    x_medio = (X_BACK + X_FORWARD) / 2
    x_amp   = (X_BACK - X_FORWARD) / 2  # positivo pois X_BACK > X_FORWARD
    # Amplitude total de elevação da perna
    y_amp   = Y_GROUND - Y_LIFT  # positivo pois Y_GROUND > Y_LIFT

    # Reposiciona apenas motor_x em X_BACK para iniciar o ciclo no ponto correto
    inicio_x = int(motor_x.angle) if motor_x.angle is not None else X_CENTER
    mover_suave(motor_x, inicio_x, X_BACK)

    for rep in range(repeticoes):
        print(f"  Passo {rep + 1}/{repeticoes}")
        for i in range(passos_por_ciclo):
            # t percorre [0, 2π) uniformemente ao longo do ciclo
            t = 2 * math.pi * i / passos_por_ciclo

            # Posição horizontal: X_BACK em t=0, X_FORWARD em t=π
            x_angle = x_medio + x_amp * math.cos(t)

            # Posição vertical: eleva só durante a fase de avanço (sin(t) > 0)
            # Em t ∈ [π, 2π], max retorna 0 e y_angle == Y_GROUND (pé no chão)
            y_angle = Y_GROUND - y_amp * max(0.0, math.sin(t))

            # Arredonda para inteiro pois servo.angle exige valor discreto (graus)
            motor_x.angle = round(x_angle)
            motor_y.angle = round(y_angle)
            time.sleep(STEP_DELAY)  # Intervalo entre passos para suavizar o movimento

    print("Ciclo concluído.")


# ── Menu interativo ──────────────────────────────────────────────────────────
# Executa posicao_inicial() logo ao iniciar para garantir um estado conhecido.
posicao_inicial()

opcao = ""
try:
    while opcao != "5":
        print("\n" + "=" * 40)
        print("Teste mecanismo 4 barras")
        print("1: Testar motor_x (horizontal)")
        print("2: Testar motor_y (vertical)")
        print("3: Testar ambos individualmente")
        print("4: Ciclo de passada")
        print("5: Sair")
        opcao = input("Opção: ").strip()

        if opcao == "1":
            posicao_inicial()
            testar_motor_x()
        elif opcao == "2":
            posicao_inicial()
            testar_motor_y()
        elif opcao == "3":
            posicao_inicial()
            testar_motor_x()
            posicao_inicial()
            testar_motor_y()
            posicao_inicial()
        elif opcao == "4":
            try:
                n = int(input("Quantas repetições? [padrão 3]: ").strip() or "3")
            except ValueError:
                n = 3
            posicao_inicial()
            ciclo_passada(n)
            posicao_inicial()
        elif opcao == "5":
            posicao_inicial()
            print("Encerrando...")
        else:
            print("Opção inválida.")
except KeyboardInterrupt:
    print("\nCtrl+C detectado! Executando última ação...")
    posicao_inicial()
    print("Encerrando com segurança.")