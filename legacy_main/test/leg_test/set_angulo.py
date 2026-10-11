from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

SERVOS_VALIDOS = list(range(0, 16))

# MG966R: range 0-180°
ANGULO_MIN = 0
ANGULO_MAX = 180

print("Controle manual MG966R")
print("Comandos: '<servo> <angulo>', 'q' para sair")
print(f"Servos válidos: {SERVOS_VALIDOS}")
print(f"Range válido: {ANGULO_MIN} a {ANGULO_MAX} graus\n")

while True:
    entrada = input(">> ").strip().lower()

    if entrada == "q":
        print("Encerrando.")
        break

    partes = entrada.split()
    if len(partes) != 2:
        print("Formato inválido. Use: <servo> <angulo>")
        continue

    try:
        servo_id = int(partes[0])
        angulo = float(partes[1])
    except ValueError:
        print("Valores inválidos. Servo deve ser inteiro e ângulo um número.")
        continue

    if servo_id not in SERVOS_VALIDOS:
        print(f"Servo inválido. Use um de: {SERVOS_VALIDOS}")
        continue

    if not (ANGULO_MIN <= angulo <= ANGULO_MAX):
        print(f"Ângulo fora do range ({ANGULO_MIN}-{ANGULO_MAX}°).")
        continue

    kit.servo[servo_id].angle = angulo
    print(f"servo[{servo_id}] -> {angulo}°")


