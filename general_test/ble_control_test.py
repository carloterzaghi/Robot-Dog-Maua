import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))
from lib.connect_ps3_control import ensure_connected

from evdev import InputDevice, ecodes, categorize # type: ignore

caminho = ensure_connected()
if not caminho:
    print("Não foi possível conectar ao controle PS3.")
    sys.exit(1)

gamepad = InputDevice(caminho)

print(f"Conectado com sucesso ao: {gamepad.name} ({caminho})")
print()

# Mostra os limites reais de cada eixo analógico disponível
print("=== Eixos disponíveis e seus limites ===")
capabilities = gamepad.capabilities()
if ecodes.EV_ABS in capabilities:
    for codigo, absinfo in capabilities[ecodes.EV_ABS]:
        nome_eixo = ecodes.ABS.get(codigo, f"ABS_{codigo}")
        print(f"  {nome_eixo} (código {codigo}): min={absinfo.min}, max={absinfo.max}, flat={absinfo.flat}, fuzz={absinfo.fuzz}")
print()

print("Mova os analógicos e aperte os botões! Pressione Ctrl+C para sair.")
print("=" * 50)

try:
    for event in gamepad.read_loop():
        if event.type == ecodes.EV_ABS:
            nome_eixo = ecodes.ABS.get(event.code, f"ABS_{event.code}")
            print(f"[EIXO]   {nome_eixo} (código {event.code}) | Valor: {event.value}")
        elif event.type == ecodes.EV_KEY:
            nome_botao = ecodes.BTN.get(event.code, ecodes.KEY.get(event.code, f"KEY_{event.code}"))
            estado = "PRESSIONADO" if event.value == 1 else "SOLTO"
            print(f"[BOTÃO]  {nome_botao} (código {event.code}) | {estado}")
except KeyboardInterrupt:
    print("\nTeste de controle encerrado.")