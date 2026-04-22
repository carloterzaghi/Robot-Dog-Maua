import time
import threading
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

motor_x = kit.servo[4]  # Movimento horizontal (avanço/recuo da perna)
motor_y = kit.servo[0]  # Movimento vertical (levanta/abaixa a perna)

# Ângulos de referência (ajuste conforme a montagem física)
X_CENTER = 95   # Posição central do motor_x
Y_GROUND = 180   # Posição com perna no chão (motor_y)
Y_LIFT   = 150   # Posição com perna levantada (motor_y)
X_FORWARD = 60  # Perna avançada (motor_x)
X_BACK    = 120 # Perna recuada (motor_x)

STEP_DELAY = 0.01  # Delay entre cada grau no movimento suave (segundos)
HOLD_DELAY = 0.5   # Tempo parado nos extremos (segundos)


def mover_suave(servo, angulo_inicio, angulo_fim, delay=STEP_DELAY):
    """Move o servo suavemente de um ângulo ao outro."""
    angulo_inicio = int(angulo_inicio)
    angulo_fim = int(angulo_fim)
    passo = 1 if angulo_fim > angulo_inicio else -1
    for angulo in range(angulo_inicio, angulo_fim + passo, passo):
        servo.angle = angulo
        time.sleep(delay)


def posicao_inicial():
    """Coloca a perna na posição central de repouso."""
    print("Movendo para posição inicial...")
    inicio_x = int(motor_x.angle) if motor_x.angle is not None else X_CENTER
    inicio_y = int(motor_y.angle) if motor_y.angle is not None else Y_GROUND
    mover_suave(motor_x, inicio_x, X_CENTER)
    mover_suave(motor_y, inicio_y, Y_GROUND)
    time.sleep(HOLD_DELAY)


def testar_motor_x():
    """Testa o range completo do motor_x (avanço/recuo)."""
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
    """Testa o range completo do motor_y (levanta/abaixa)."""
    print("\n--- Teste motor_y (vertical) ---")
    print(f"Chão ({Y_GROUND}°) -> Levanta ({Y_LIFT}°) -> Chão ({Y_GROUND}°)")
    mover_suave(motor_y, Y_GROUND, Y_LIFT)
    time.sleep(HOLD_DELAY)
    mover_suave(motor_y, Y_LIFT, Y_GROUND)
    time.sleep(HOLD_DELAY)
    print("motor_y OK")


def mover_juntos(servo_a, inicio_a, fim_a, servo_b, inicio_b, fim_b, delay=STEP_DELAY):
    """Move dois servos simultaneamente usando threads."""
    t_a = threading.Thread(target=mover_suave, args=(servo_a, inicio_a, fim_a, delay))
    t_b = threading.Thread(target=mover_suave, args=(servo_b, inicio_b, fim_b, delay))
    t_a.start()
    t_b.start()
    t_a.join()
    t_b.join()


def ciclo_passada(repeticoes=3):
    """
    Executa um ciclo de passada completo:
      1. Levanta + Avança  (Y: chão -> levantado, X: recuado -> avançado) — simultâneo
      2. Abaixa + Recua    (Y: levantado -> chão, X: avançado -> recuado) — simultâneo (fase de apoio)
    """
    print(f"\n--- Ciclo de passada ({repeticoes} repetições) ---")
    mover_suave(motor_x, X_CENTER, X_BACK)
    time.sleep(HOLD_DELAY)

    for i in range(repeticoes):
        print(f"  Passo {i + 1}/{repeticoes}")
        # Fase de balanço: levanta e avança juntos
        mover_juntos(motor_y, Y_GROUND, Y_LIFT, motor_x, X_BACK, X_FORWARD)
        # Fase de apoio: abaixa e recua juntos
        mover_juntos(motor_y, Y_LIFT, Y_GROUND, motor_x, X_FORWARD, X_BACK)

    mover_suave(motor_x, X_BACK, X_CENTER)
    print("Ciclo concluído.")


# ── Menu ──────────────────────────────────────────────────────────────────────
posicao_inicial()

opcao = ""
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

