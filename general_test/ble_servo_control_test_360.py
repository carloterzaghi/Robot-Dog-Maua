import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))
from lib.connect_ps3_control import ensure_connected

from evdev import InputDevice, categorize, ecodes # type: ignore
from adafruit_servokit import ServoKit

# 1. Configuração do Atuador (PCA9685)
print("Iniciando conexão com o PCA9685...")
kit = ServoKit(channels=16)

# 2. Configuração do Controle PS3
caminho = ensure_connected()
if not caminho:
    print("Não foi possível conectar ao controle PS3.")
    sys.exit(1)

gamepad = InputDevice(caminho)
print(f"Controle conectado: {gamepad.name} ({caminho})")

# Descobre os limites reais dos dois eixos
abs_info_x = gamepad.absinfo(ecodes.ABS_X)
abs_info_y = gamepad.absinfo(ecodes.ABS_Y)
X_MIN, X_MAX = abs_info_x.min, abs_info_x.max
Y_MIN, Y_MAX = abs_info_y.min, abs_info_y.max
print(f"Limites ABS_X: {X_MIN} – {X_MAX}")
print(f"Limites ABS_Y: {Y_MIN} – {Y_MAX}")

# Estado atual do joystick (começa no centro)
joy_x = (X_MIN + X_MAX) / 2
joy_y = (Y_MIN + Y_MAX) / 2

ZONA_MORTA = 0.1  # ignora movimentos menores que 10% para evitar drift

def normalizar(valor, minimo, maximo):
    """Converte o valor para -1.0 (mín) até 1.0 (máx), com 0.0 no centro."""
    meio = (minimo + maximo) / 2
    metade = (maximo - minimo) / 2
    return max(-1.0, min(1.0, (valor - meio) / metade))

print("Mova o analógico esquerdo para controlar o servo 360°! Ctrl+C para sair.")
print("↑ Cima = gira para frente | ↓ Baixo = gira para trás | Centro = parado")

try:
    for event in gamepad.read_loop():
        if event.type != ecodes.EV_ABS:
            continue

        if event.code == ecodes.ABS_X:
            joy_x = event.value
        elif event.code == ecodes.ABS_Y:
            joy_y = event.value
        else:
            continue

        # Normaliza os dois eixos para -1.0 a 1.0
        nx = normalizar(joy_x, X_MIN, X_MAX)
        ny = normalizar(joy_y, Y_MIN, Y_MAX)

        # Calcula a magnitude do vetor do joystick
        magnitude = math.sqrt(nx**2 + ny**2)

        if magnitude < ZONA_MORTA:
            # Joystick no centro: para o servo
            kit.continuous_servo[0].throttle = 0
            print("Joystick: ● centro  → Servo parado")
        else:
            # Calcula o ângulo do joystick (0° = direita, 90° = cima)
            angulo = math.degrees(math.atan2(-ny, nx))
            if angulo < 0:
                angulo += 360

            # Determina a direção principal
            if 45 <= angulo < 135:
                direcao = "↑ CIMA"
            elif 135 <= angulo < 225:
                direcao = "← ESQUERDA"
            elif 225 <= angulo < 315:
                direcao = "↓ BAIXO"
            else:
                direcao = "→ DIREITA"

            # Throttle: componente Y controla frente/trás, magnitude controla velocidade
            # -ny: joystick para cima (ny negativo) → throttle positivo (frente)
            throttle = -ny * min(1.0, magnitude)
            kit.continuous_servo[0].throttle = throttle
            print(f"Joystick: {direcao} ({angulo:.0f}°) | Throttle: {throttle:+.2f}")

except KeyboardInterrupt:
    kit.continuous_servo[0].throttle = 0
    print("\nSistema encerrado.")