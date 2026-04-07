import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))
from lib.connect_ps3_control import ensure_connected

from evdev import InputDevice, categorize, ecodes
from adafruit_servokit import ServoKit # [cite: 544]

# 1. Configuração do Atuador (PCA9685)
print("Iniciando conexão com o PCA9685...")
kit = ServoKit(channels=16) # [cite: 544, 545]

# 2. Configuração do Controle PS3
caminho = ensure_connected()
if not caminho:
    print("Não foi possível conectar ao controle PS3.")
    sys.exit(1)

gamepad = InputDevice(caminho)
print(f"Controle conectado: {gamepad.name} ({caminho})")

# Função para converter a escala do joystick (0-255) para graus do servo (0-180)
def map_valor(valor_atual, min_entrada, max_entrada, min_saida, max_saida):
    return (valor_atual - min_entrada) * (max_saida - min_saida) / (max_entrada - min_entrada) + min_saida

print("Mova o analógico esquerdo para cima/baixo para mover o motor! Ctrl+C para sair.")

try:
    for event in gamepad.read_loop():
        # Captura apenas movimentos analógicos
        if event.type == ecodes.EV_ABS:
            # ABS_Y é o eixo vertical do analógico esquerdo
            if event.code == ecodes.ABS_Y:
                valor_joystick = event.value
                
                # Mapeia de 0-255 para 0-180 graus
                angulo = map_valor(valor_joystick, 0, 255, 0, 180)
                angulo = int(angulo) # Arredonda para número inteiro
                
                # Envia o ângulo para o servo no canal 0
                kit.servo[0].angle = angulo # [cite: 556, 559]
                print(f"Joystick: {valor_joystick} -> Ângulo do Motor: {angulo}°")
                
except KeyboardInterrupt:
    print("\nSistema encerrado.")