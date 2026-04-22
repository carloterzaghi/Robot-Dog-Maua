from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

motor_x = kit.servo[4]  # Movimento horizontal (avanço/recuo da perna)
motor_y = kit.servo[0]  # Movimento vertical (levanta/abaixa a perna)

# MG966R: range 0-180°
ANGULO_MIN = 0
ANGULO_MAX = 180

print("Controle manual MG966R")
print("Comandos: 'x <angulo>', 'y <angulo>', 'q' para sair")
print(f"Range válido: {ANGULO_MIN} a {ANGULO_MAX} graus\n")

while True:
    entrada = input(">> ").strip().lower()

    if entrada == "q":
        print("Encerrando.")
        break

    partes = entrada.split()
    if len(partes) != 2 or partes[0] not in ("x", "y"):
        print("Formato inválido. Use: x <angulo> ou y <angulo>")
        continue

    try:
        angulo = float(partes[1])
    except ValueError:
        print("Ângulo inválido. Digite um número.")
        continue

    if not (ANGULO_MIN <= angulo <= ANGULO_MAX):
        print(f"Ângulo fora do range ({ANGULO_MIN}-{ANGULO_MAX}°).")
        continue

    if partes[0] == "x":
        motor_x.angle = angulo
        print(f"motor_x -> {angulo}°")
    else:
        motor_y.angle = angulo
        print(f"motor_y -> {angulo}°")


