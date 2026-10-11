"""
mapear_controle.py — Testador completo de botões e analógicos de controle para Raspberry Pi.

Funcionalidades:
  1. Detecção automática de controles (USB, Dongle 2.4GHz ou Bluetooth).
  2. Modo Monitor Livre: Mostra na tela qualquer botão ou eixo acionado em tempo real.

"""

import os
import sys
import json
import time

try:
    from evdev import InputDevice, list_devices, ecodes, categorize  # type: ignore
except ImportError:
    print("\n[ERRO] A biblioteca 'evdev' não está instalada ou este sistema não é Linux.")
    print("Na Raspberry Pi, execute:")
    print("    pip install evdev")
    print("    sudo usermod -aG input $USER\n")
    sys.exit(1)


# ── Nomes amigáveis comuns para referência ────────────────────────────────────
NOMES_CONHECIDOS = {
    304: "Botão A / Sul (BTN_SOUTH)",
    305: "Botão B / Leste (BTN_EAST)",
    307: "Botão X / Oeste (BTN_NORTH / BTN_WEST)",
    308: "Botão Y / Norte (BTN_NORTH / BTN_WEST)",
    310: "Gatilho Superior Esq (LB / L1)",
    311: "Gatilho Superior Dir (RB / R1)",
    312: "Gatilho Inferior Esq (LT / L2)",
    313: "Gatilho Inferior Dir (RT / R2)",
    314: "Botão Select / Back / Share",
    315: "Botão Start / Menu / Options",
    316: "Botão Home / Logo / Xbox / PS",
    317: "Clique Analógico Esquerdo (L3 / THUMBL)",
    318: "Clique Analógico Direito (R3 / THUMBR)",
    # D-Pad analógico ou hat
    16: "D-Pad Horizontal (ABS_HAT0X)",
    17: "D-Pad Vertical (ABS_HAT0Y)",
    # Analógicos comuns
    0:  "Analógico Esquerdo - Eixo X (ABS_X)",
    1:  "Analógico Esquerdo - Eixo Y (ABS_Y)",
    2:  "Gatilho Analógico Z / LT (ABS_Z)",
    3:  "Analógico Direito - Eixo X (ABS_RX)",
    4:  "Analógico Direito - Eixo Y (ABS_RY)",
    5:  "Gatilho Analógico RZ / RT (ABS_RZ)",
}


NOME_CONTROLE_EXATO = "Guangzhou Chicken Run Network Technology Co., Ltd. Wireless Controller"


def conectar_controle() -> InputDevice | None:
    """Conecta diretamente e exclusivamente ao controle com o nome exato."""
    for path in list_devices():
        try:
            dev = InputDevice(path)
            if dev.name.strip() == NOME_CONTROLE_EXATO:
                print(f"\n[SUCESSO] Controle conectado:")
                print(f"  Dispositivo : {dev.name}")
                print(f"  Evento      : {dev.path}")
                return dev
        except Exception:
            continue

    print(f"\n[ERRO] Controle '{NOME_CONTROLE_EXATO}' não encontrado.")
    print("Dispositivos detectados no sistema:")
    for path in list_devices():
        try:
            print(f"  - {path} : {InputDevice(path).name}")
        except Exception:
            pass
    return None


def limpar_buffer(device: InputDevice):
    """Descarta todos os eventos que já estavam acumulados no buffer do kernel."""
    try:
        while device.read_one() is not None:
            pass
    except Exception:
        pass

# ── MODO 1: MONITOR LIVRE EM TEMPO REAL ───────────────────────────────────────
def modo_monitor_livre(device: InputDevice):
    """Exibe todos os botões e eixos acionados em tempo real."""
    print("\n" + "=" * 65)
    print("          MODO MONITOR LIVRE (LIVE SNIFFER)")
    print("=" * 65)
    print(" Pressione qualquer botão ou mova os analógicos do controle.")
    print(" Pressione [Ctrl + C] para sair.")
    print("=" * 65 + "\n")

    botoes_vistos = {}

    try:
        device.grab()
    except Exception:
        pass

    try:
        for event in device.read_loop():
            if event.type == ecodes.EV_SYN:
                continue

            ts = time.strftime("%H:%M:%S")

            # ── 1. Botões Digitais (EV_KEY) ──
            if event.type == ecodes.EV_KEY:
                estado = "PRESSIONADO" if event.value == 1 else "SOLTO" if event.value == 0 else "MANTIDO"

                try:
                    key_event = categorize(event)
                    keycode = key_event.keycode
                    keycode_str = " / ".join(keycode) if isinstance(keycode, list) else str(keycode)
                except Exception:
                    keycode_str = f"BTN_{event.code}"

                desc = NOMES_CONHECIDOS.get(event.code, keycode_str)

                if event.value == 1:
                    botoes_vistos[event.code] = desc

                print(f"[{ts}] [BOTÃO] Código: {event.code:<4} | Estado: {estado:<11} | Ref: {desc}")

            # ── 2. Analógicos e Gatilhos (EV_ABS) ──
            elif event.type == ecodes.EV_ABS:
                abs_name = ecodes.ABS.get(event.code, f"ABS_{event.code}")
                if isinstance(abs_name, list):
                    abs_name = abs_name[0]
                desc = NOMES_CONHECIDOS.get(event.code, str(abs_name))

                print(f"[{ts}] [EIXO ] Código: {event.code:<4} | Valor: {event.value:<7} | Ref: {desc}")

    except KeyboardInterrupt:
        print("\n\nMonitoramento finalizado.")
    finally:
        try:
            device.ungrab()
        except Exception:
            pass

    if botoes_vistos:
        print("\n" + "─" * 60)
        print(" Resumo dos botões identificados nesta sessão:")
        print("─" * 60)
        for code, desc in sorted(botoes_vistos.items()):
            print(f"   Código {code:<4} -> {desc}")
        print("─" * 60)



# ── MENU PRINCIPAL ────────────────────────────────────────────────────────────
def main():
    print("\n" + "=" * 65)
    print("    ROBOT-DOG-MAUA — MAPEADOR DE CONTROLE (Raspberry Pi)")
    print("=" * 65)

    device = conectar_controle()
    if not device:
        print("Encerrando.")
        return

    while True:
        print("\nEscolha o modo de operação:")
        print("  [1] Monitor Livre (Exibe qualquer botão ou eixo acionado)")
        print("  [2] Sair")
        opcao = input("\nOpção (1/2): ").strip()

        if opcao == "1":
            modo_monitor_livre(device)
        elif opcao in ("2", "sair", "exit", "q"):
            print("Saindo...")
            break
        else:
            print("Opção inválida! Escolha 1 ou 2.")



if __name__ == "__main__":
    main()
