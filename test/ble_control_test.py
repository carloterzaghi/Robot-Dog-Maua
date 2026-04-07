import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))
from connect_ps3_control import ensure_connected

from evdev import InputDevice, ecodes

caminho = ensure_connected()
if not caminho:
    print("Não foi possível conectar ao controle PS3.")
    sys.exit(1)

gamepad = InputDevice(caminho)

print(f"Conectado com sucesso ao: {gamepad.name} ({caminho})")
print("Mova os analógicos! Pressione Ctrl+C para sair.")

try:
    for event in gamepad.read_loop():
        # Filtra apenas os eventos de eixos analógicos (EV_ABS)
        if event.type == ecodes.EV_ABS:
            # Imprime o código do eixo e o valor (geralmente varia de 0 a 255)
            print(f"Código do Eixo: {event.code} | Valor: {event.value}")
except KeyboardInterrupt:
    print("\nTeste de controle encerrado.")