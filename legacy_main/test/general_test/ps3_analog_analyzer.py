"""
ps3_analog_analyzer.py
─────────────────────────────────────────────────────────────────────────────
Ferramenta de diagnóstico para analisar as respostas do controle PS3.
Mostra em tempo real os valores brutos e normalizados dos eixos analógicos.

Uso:
    python ps3_analog_analyzer.py

Pressione Ctrl+C para sair.
"""

import sys
import os
import time

# Adiciona a pasta lib ao path para o connect_ps3_control
_HERE = os.path.dirname(os.path.abspath(__file__))
_LIB  = os.path.abspath(os.path.join(_HERE, "..", "..", "lib"))
if _LIB not in sys.path:
    sys.path.insert(0, _LIB)

try:
    from connect_ps3_control import ensure_connected  # type: ignore
    from evdev import InputDevice, ecodes, categorize  # type: ignore
except ImportError as e:
    print(f"Erro ao importar bibliotecas: {e}")
    print("Certifique-se de que evdev está instalado: pip install evdev")
    sys.exit(1)

DEADZONE = 0.20

def deadzone(val, dz=DEADZONE):
    if abs(val) < dz:
        return 0.0
    sign = 1.0 if val > 0 else -1.0
    return sign * (abs(val) - dz) / (1.0 - dz)

def normalize(raw_value):
    # O driver sixad reporta o centro como 0 (faixa de aprox -128 a +127).
    return raw_value / 128.0

# Mapeamento de nomes dos eixos
AXIS_NAMES = {
    ecodes.ABS_X:  "ABS_X  (Esq  ←→)",
    ecodes.ABS_Y:  "ABS_Y  (Esq  ↑↓)",
    ecodes.ABS_RX: "ABS_RX (Dir  ←→)",
    ecodes.ABS_RY: "ABS_RY (Dir  ↑↓)",
    ecodes.ABS_Z:  "ABS_Z  (L2     )",
    ecodes.ABS_RZ: "ABS_RZ (R2     )",
}

# Mapeamento de nomes dos botões
BUTTON_NAMES = {
    ecodes.BTN_SOUTH:  "Cruz      (BTN_SOUTH)",
    ecodes.BTN_EAST:   "Círculo   (BTN_EAST) ",
    ecodes.BTN_NORTH:  "Triângulo (BTN_NORTH)",
    ecodes.BTN_WEST:   "Quadrado  (BTN_WEST) ",
    ecodes.BTN_START:  "Start     (BTN_START)",
    ecodes.BTN_SELECT: "Select    (BTN_SELECT)",
    ecodes.BTN_TL:     "L1        (BTN_TL)   ",
    ecodes.BTN_TR:     "R1        (BTN_TR)   ",
    ecodes.BTN_TL2:    "L2        (BTN_TL2)  ",
    ecodes.BTN_TR2:    "R2        (BTN_TR2)  ",
    ecodes.BTN_THUMBL: "L3        (BTN_THUMBL)",
    ecodes.BTN_THUMBR: "R3        (BTN_THUMBR)",
    ecodes.BTN_TOP:    "TOP       (BTN_TOP)  ",
    315:               "Start     (315)      ",
}

def bar(val, width=20):
    """Barra visual de -1.0 a +1.0."""
    center = width // 2
    filled = int(abs(val) * center)
    filled = min(filled, center)
    if val < 0:
        left  = " " * (center - filled)
        mid   = "█" * filled + "|"
        right = " " * center
    elif val > 0:
        left  = " " * center + "|"
        mid   = "█" * filled
        right = " " * (center - filled)
    else:
        left  = " " * center
        mid   = "|"
        right = " " * center
    return f"[{left}{mid}{right}]"

def main():
    print("Conectando ao controle PS3...")
    device_path = ensure_connected(timeout=20)
    if not device_path:
        print("Falha ao conectar o controle PS3.")
        sys.exit(1)

    try:
        gamepad = InputDevice(device_path)
        print(f"Controle conectado: {gamepad.name}  ({device_path})\n")
    except Exception as e:
        print(f"Erro ao acessar dispositivo: {e}")
        sys.exit(1)

    print("=" * 70)
    print("  ANALISADOR DE CONTROLE PS3")
    print("  Mova os analógicos e pressione botões. Ctrl+C para sair.")
    print("=" * 70)
    print(f"\n  {'Eixo/Botão':<28} {'Raw':>6}  {'Norm':>7}  {'DZ':>7}  Barra")
    print("  " + "-" * 66)

    print("  " + "-" * 66)

    try:
        gamepad.grab()
    except Exception:
        print("Aviso: não foi possível obter grab exclusivo.")

    try:
        for event in gamepad.read_loop():
            if event.type == ecodes.EV_SYN:
                continue

            ts = time.strftime("%H:%M:%S")

            # ── Eixos analógicos ─────────────────────────────────────────────
            if event.type == ecodes.EV_ABS:
                if event.code not in (ecodes.ABS_X, ecodes.ABS_Y):
                    continue
                
                raw = event.value
                axis_name = AXIS_NAMES.get(event.code, f"ABS_{event.code:<6}")

                norm = normalize(raw)
                dz   = deadzone(norm)
                vis  = bar(dz)
                forca = abs(dz)

                print(f"  [{ts}] {axis_name:<28} {raw:>6}  {norm:>+7.3f}  {dz:>+7.3f}  {vis}  (Força: {forca:.2f})")

    except KeyboardInterrupt:
        print("\n\nEncerrando analisador.")
    finally:
        try:
            gamepad.ungrab()
        except Exception:
            pass
        print("Desconectado.")

if __name__ == "__main__":
    main()