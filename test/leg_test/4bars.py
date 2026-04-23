"""
4bars.py — Teste do mecanismo de 4 barras para uma perna do Robot Dog Mauá.

Este script controla dois servos que formam o mecanismo de 4 barras de uma perna:
  - motor_x (canal 4): responsável pelo movimento horizontal (avanço/recuo).
  - motor_y (canal 0): responsável pelo movimento vertical (levanta/abaixa).

O ciclo de passada utiliza uma trajetória elíptica senoidal contínua, garantindo
que ambos os servos se movam de forma simultânea e fluida em todos os momentos.

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
HOLD_DELAY = 0.5   # Tempo (s) de pausa nos extremos após testes individuais


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
    Útil para validar limites físicos e calibrar X_FORWARD/X_BACK.
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
    Útil para validar limites físicos e calibrar Y_LIFT.
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
    Executa um ciclo de passada contínuo e fluido usando trajetória elíptica.

    A posição dos dois servos é calculada a cada passo por funções cossenoidais
    em fase, de modo que ambos se movam simultaneamente em todo momento,
    descrevendo uma elipse no espaço de trabalho da perna:

        x(t) = x_medio + x_amp * cos(t)
        y(t) = y_medio + y_amp * cos(t)

    Correspondência de fase:
        t = 0  → X = X_BACK,    Y = Y_GROUND  (pé recuado, no chão)
        t = π  → X = X_FORWARD, Y = Y_LIFT    (pé avançado, no alto)

    Parâmetros:
        repeticoes      : número de ciclos completos a executar (padrão 3).
        passos_por_ciclo: resolução angular do ciclo — mais passos geram
                          movimento mais suave, porém mais lento (padrão 100).
    """
    print(f"\n--- Ciclo de passada fluido ({repeticoes} repetições) ---")

    # Ponto médio e amplitude da oscilação horizontal
    x_medio = (X_BACK + X_FORWARD) / 2
    x_amp   = (X_BACK - X_FORWARD) / 2   # positivo pois X_BACK > X_FORWARD

    # Ponto médio e amplitude da oscilação vertical
    y_medio = (Y_GROUND + Y_LIFT) / 2
    y_amp   = (Y_GROUND - Y_LIFT) / 2    # positivo pois Y_GROUND > Y_LIFT

    # Leva a perna ao ponto inicial do ciclo (X_BACK, Y_GROUND) suavemente
    inicio_x = int(motor_x.angle) if motor_x.angle is not None else X_CENTER
    inicio_y = int(motor_y.angle) if motor_y.angle is not None else Y_GROUND
    mover_juntos(motor_x, inicio_x, X_BACK, motor_y, inicio_y, Y_GROUND)

    for rep in range(repeticoes):
        print(f"  Passo {rep + 1}/{repeticoes}")
        for i in range(passos_por_ciclo):
            # t é o parâmetro angular do ciclo, variando de 0 a 2π (exclusive).
            # Cada incremento de i representa um "fatia" igual do ciclo completo.
            t = 2 * math.pi * i / passos_por_ciclo

            # Trajetória elíptica: x e y compartilham o mesmo cos(t), por isso
            # atingem seus extremos exatamente ao mesmo tempo:
            #   t=0 → (X_BACK,    Y_GROUND)  — pé recuado, apoiado no chão
            #   t=π → (X_FORWARD, Y_LIFT)    — pé avançado, no ponto mais alto
            # Como cos(t) é contínuo e periódico, não há descontinuidade entre
            # o fim de uma repetição e o início da próxima.
            x_angle = x_medio + x_amp * math.cos(t)
            y_angle = y_medio + y_amp * math.cos(t)

            # Arredonda para inteiro pois servo.angle exige valor discreto (graus).
            motor_x.angle = round(x_angle)
            motor_y.angle = round(y_angle)
            time.sleep(STEP_DELAY)  # Aguarda antes do próximo passo para suavizar o movimento

    print("Ciclo concluído.")


# ── Menu ──────────────────────────────────────────────────────────────────────
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