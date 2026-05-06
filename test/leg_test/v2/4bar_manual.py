"""
4bar_manual.py — Controle manual dos servos (v2: novo mecanismo de 4 barras).

Novo mecanismo (conforme foto):
  - Servo 0 (topo)   : ombro — controla a elevação da perna inteira.
  - Servo 4 (lateral) : joelho — controla o avanço/recuo (extensão da perna).

Na v1, motor_x (servo 4) fazia avanço/recuo e motor_y (servo 0) fazia elevação.
Na v2, a cinemática muda:
  - Servo 0 (ombro) : rotação do link superior → sobe/desce a perna.
  - Servo 4 (joelho): rotação do link inferior → empurra/puxa a perna (frente/trás).

O mecanismo de paralelogramo faz com que a parte inferior da perna mantenha
orientação aproximadamente constante enquanto o ombro sobe/desce.

Hardware: Raspberry Pi + PCA9685 (via Adafruit ServoKit, I2C).
"""

from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

servo_ombro  = kit.servo[3]  # Servo 0: ombro (elevação — link superior do paralelogramo)
servo_joelho = kit.servo[7]  # Servo 4: joelho (extensão — link inferior/traseiro)

# MG966R: range 0-180°
ANGULO_MIN = 0
ANGULO_MAX = 180

print("Controle manual MG966R (v2 — novo mecanismo 4 barras)")
print("Servo 0 = ombro (elevação)  |  Servo 4 = joelho (extensão)")
print("Comandos: '0 <angulo>', '4 <angulo>', 'q' para sair")
print(f"Range válido: {ANGULO_MIN} a {ANGULO_MAX} graus\n")

while True:
    entrada = input(">> ").strip().lower()

    if entrada == "q":
        print("Encerrando.")
        break

    partes = entrada.split()
    if len(partes) != 2 or partes[0] not in ("0", "4"):
        print("Formato inválido. Use: 0 <angulo> ou 4 <angulo>")
        continue

    try:
        angulo = float(partes[1])
    except ValueError:
        print("Ângulo inválido. Digite um número.")
        continue

    if not (ANGULO_MIN <= angulo <= ANGULO_MAX):
        print(f"Ângulo fora do range ({ANGULO_MIN}-{ANGULO_MAX}°).")
        continue

    if partes[0] == "0":
        servo_ombro.angle = angulo
        print(f"servo_ombro (0) -> {angulo}°")
    else:
        servo_joelho.angle = angulo
        print(f"servo_joelho (4) -> {angulo}°")
