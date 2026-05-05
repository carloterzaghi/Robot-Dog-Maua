"""
4bars.py — Teste do mecanismo de 4 barras v2 para uma perna do Robot Dog Mauá.

NOVO MECANISMO (v2) — conforme a foto de referência:
  ┌──────────────────────────────────────────────────────────────────────┐
  │  Servo 0 (ombro) : link superior do paralelogramo.                 │
  │                    Gira para cima/baixo → eleva/abaixa a perna.    │
  │  Servo 4 (joelho): link inferior/traseiro do paralelogramo.        │
  │                    Gira frente/trás → estende/recolhe a perna.     │
  └──────────────────────────────────────────────────────────────────────┘

Diferenças em relação à v1:
  Na v1, motor_x (servo 4) controlava diretamente o avanço/recuo e motor_y
  (servo 0) controlava a elevação, de forma basicamente desacoplada.

  Na v2, o mecanismo de paralelogramo acopla as duas articulações:
    - O ombro (servo 0) define a altura da perna. Ao levantar, o paralelogramo
      mantém a parte inferior da perna orientada de forma paralela.
    - O joelho (servo 4) define a extensão horizontal. Empurrar para frente
      avança o pé; puxar para trás recua o pé.

  Isso resulta em um ciclo de passada com coordenação diferente:
    - Fase de balanço (swing): ombro SOBE + joelho AVANÇA simultaneamente.
    - Fase de apoio (stance) : ombro no CHÃO + joelho RECUA (empurra o corpo).

Ciclo de passada (trajetória adaptada ao paralelogramo):
  O ciclo usa t ∈ [0, 2π) com coordenação entre ombro e joelho:
    ombro(t) = OMBRO_CHAO - amp_ombro * max(0, sin(t))
      → Eleva apenas durante t ∈ [0, π] (fase de balanço).
    joelho(t) = joelho_medio + amp_joelho * cos(t)
      → Oscila continuamente: avança durante o balanço, recua durante o apoio.

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

servo_ombro  = kit.servo[0]  # Servo 0: ombro — elevação (link superior do paralelogramo)
servo_joelho = kit.servo[4]  # Servo 4: joelho — extensão (link inferior/traseiro)

# ── Ângulos de referência (graus) ─────────────────────────────────────────────
# Ajuste estes valores conforme a montagem física da perna v2.
#
# OMBRO (Servo 0): controla a altura da perna
#   OMBRO_CHAO    : ângulo com o pé no chão (posição de apoio)
#   OMBRO_ELEVADO : ângulo com a perna totalmente levantada (posição de balanço)
#   OMBRO_CENTRO  : ângulo neutro/central do ombro
OMBRO_CENTRO  = 90   # Posição central/neutra do ombro
OMBRO_CHAO    = 90   # Pé tocando o chão (aumentar = perna mais baixa)
OMBRO_ELEVADO = 60   # Perna no ponto mais alto (diminuir = mais alto)

# JOELHO (Servo 4): controla o avanço/recuo da perna
#   JOELHO_FRENTE : ângulo com a perna totalmente avançada
#   JOELHO_TRAS   : ângulo com a perna totalmente recuada
#   JOELHO_CENTRO : ângulo neutro/central do joelho
JOELHO_CENTRO = 90   # Posição central/neutra do joelho
JOELHO_FRENTE = 30   # Perna totalmente avançada
JOELHO_TRAS   = 125  # Perna totalmente recuada

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
    Leva a perna à posição de repouso: ombro no chão, joelho centralizado.

    Move servo_ombro e servo_joelho sequencialmente a partir de seus ângulos
    atuais, garantindo uma transição suave independentemente do estado anterior.
    Aguarda HOLD_DELAY ao final para estabilizar a posição.
    """
    print("Movendo para posição inicial...")
    inicio_ombro  = int(servo_ombro.angle) if servo_ombro.angle is not None else OMBRO_CENTRO
    inicio_joelho = int(servo_joelho.angle) if servo_joelho.angle is not None else JOELHO_CENTRO
    mover_suave(servo_ombro, inicio_ombro, OMBRO_CHAO)
    mover_suave(servo_joelho, inicio_joelho, JOELHO_CENTRO)
    time.sleep(HOLD_DELAY)


def testar_ombro():
    """
    Percorre o range completo do servo_ombro para verificar seu funcionamento.

    Sequência: OMBRO_CHAO → OMBRO_ELEVADO → OMBRO_CHAO.
    Útil para validar os limites físicos e calibrar OMBRO_ELEVADO.
    """
    print("\n--- Teste servo_ombro (elevação, servo 0) ---")
    print(f"Chão ({OMBRO_CHAO}°) -> Elevado ({OMBRO_ELEVADO}°) -> Chão ({OMBRO_CHAO}°)")
    mover_suave(servo_ombro, OMBRO_CHAO, OMBRO_ELEVADO)
    time.sleep(HOLD_DELAY)
    mover_suave(servo_ombro, OMBRO_ELEVADO, OMBRO_CHAO)
    time.sleep(HOLD_DELAY)
    print("servo_ombro OK")


def testar_joelho():
    """
    Percorre o range completo do servo_joelho para verificar seu funcionamento.

    Sequência: JOELHO_CENTRO → JOELHO_FRENTE → JOELHO_TRAS → JOELHO_CENTRO.
    Útil para validar os limites físicos e calibrar JOELHO_FRENTE/JOELHO_TRAS.
    """
    print("\n--- Teste servo_joelho (extensão, servo 4) ---")
    print(f"Centro ({JOELHO_CENTRO}°) -> Frente ({JOELHO_FRENTE}°) -> Trás ({JOELHO_TRAS}°) -> Centro")
    mover_suave(servo_joelho, JOELHO_CENTRO, JOELHO_FRENTE)
    time.sleep(HOLD_DELAY)
    mover_suave(servo_joelho, JOELHO_FRENTE, JOELHO_TRAS)
    time.sleep(HOLD_DELAY)
    mover_suave(servo_joelho, JOELHO_TRAS, JOELHO_CENTRO)
    time.sleep(HOLD_DELAY)
    print("servo_joelho OK")


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
    Executa um ciclo de passada fluido adaptado ao mecanismo de paralelogramo v2.

    Equações de movimento (coordenação ombro + joelho):
        joelho(t) = joelho_medio + amp_joelho * cos(t)
        ombro(t)  = OMBRO_CHAO  - amp_ombro  * max(0, sin(t))

    Funcionamento do paralelogramo:
      - O joelho (servo 4) oscila continuamente em cosseno, controlando a
        posição horizontal do pé. Começa em JOELHO_TRAS (t=0) e vai até
        JOELHO_FRENTE (t=π).
      - O ombro (servo 0) usa meia senoide (apenas a parte positiva do sin(t)):
        levanta a perna durante t ∈ [0, π] (fase de balanço/swing) e mantém
        o pé no chão durante t ∈ [π, 2π] (fase de apoio/stance).

    Resultado cinemático:
      - Fase de balanço [0, π]: perna sobe e avança (ombro eleva + joelho vai
        para frente) — pé livre no ar.
      - Fase de apoio [π, 2π]: pé no chão enquanto o joelho recua — empurra
        o corpo para frente.

    Antes do ciclo, servo_joelho é reposicionado em JOELHO_TRAS via mover_suave;
    servo_ombro já deve estar em OMBRO_CHAO (posição de repouso).

    Parâmetros:
        repeticoes      : número de ciclos completos a executar (padrão 3).
        passos_por_ciclo: resolução do ciclo — mais passos = movimento mais
                          suave, porém mais lento (padrão 100).
    """
    print(f"\n--- Ciclo de passada v2 — paralelogramo ({repeticoes} repetições) ---")

    # Ponto médio e amplitude da oscilação do joelho (horizontal)
    joelho_medio = (JOELHO_TRAS + JOELHO_FRENTE) / 2
    amp_joelho   = (JOELHO_TRAS - JOELHO_FRENTE) / 2  # positivo pois JOELHO_TRAS > JOELHO_FRENTE

    # Amplitude de elevação do ombro (vertical)
    amp_ombro = OMBRO_CHAO - OMBRO_ELEVADO  # positivo pois OMBRO_CHAO > OMBRO_ELEVADO

    # Reposiciona servo_joelho em JOELHO_TRAS para iniciar o ciclo corretamente
    inicio_joelho = int(servo_joelho.angle) if servo_joelho.angle is not None else JOELHO_CENTRO
    mover_suave(servo_joelho, inicio_joelho, JOELHO_TRAS)

    for rep in range(repeticoes):
        print(f"  Passo {rep + 1}/{repeticoes}")
        for i in range(passos_por_ciclo):
            # t percorre [0, 2π) uniformemente ao longo do ciclo
            t = 2 * math.pi * i / passos_por_ciclo

            # Posição do joelho: JOELHO_TRAS em t=0, JOELHO_FRENTE em t=π
            joelho_angle = joelho_medio + amp_joelho * math.cos(t)

            # Posição do ombro: eleva só durante a fase de balanço (sin(t) > 0)
            # Em t ∈ [π, 2π], max retorna 0 e ombro_angle == OMBRO_CHAO (pé no chão)
            ombro_angle = OMBRO_CHAO - amp_ombro * max(0.0, math.sin(t))

            # Arredonda para inteiro pois servo.angle exige valor discreto (graus)
            servo_joelho.angle = round(joelho_angle)
            servo_ombro.angle  = round(ombro_angle)
            time.sleep(STEP_DELAY)

    print("Ciclo concluído.")


# ── Menu interativo ──────────────────────────────────────────────────────────
# Executa posicao_inicial() logo ao iniciar para garantir um estado conhecido.
posicao_inicial()

opcao = ""
try:
    while opcao != "5":
        print("\n" + "=" * 40)
        print("Teste mecanismo 4 barras v2 (paralelogramo)")
        print("1: Testar servo_ombro (elevação, servo 0)")
        print("2: Testar servo_joelho (extensão, servo 4)")
        print("3: Testar ambos individualmente")
        print("4: Ciclo de passada")
        print("5: Sair")
        opcao = input("Opção: ").strip()

        if opcao == "1":
            posicao_inicial()
            testar_ombro()
        elif opcao == "2":
            posicao_inicial()
            testar_joelho()
        elif opcao == "3":
            posicao_inicial()
            testar_ombro()
            posicao_inicial()
            testar_joelho()
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
