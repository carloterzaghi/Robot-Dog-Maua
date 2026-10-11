"""
controle_servos.py — Envio simples de sinal para os servos (PCA9685).

Objetivo:
  Código direto e fácil de entender para inicializar a placa,
  selecionar a porta e enviar o sinal de ângulo ou desenergizar.
"""

import time

# ── 1. INICIALIZAÇÃO DA PLACA ──────────────────────────────────────────────────
# Conecta ao PCA9685 via I2C com 16 canais.

from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
print("Placa PCA9685 conectada com sucesso!")

# ── 2. DEFINIÇÃO DE TODAS AS PORTAS ────────────────────────────────────────────
# Mapeamento de cada um dos 16 canais (0 a 15) do robô
PORTAS = {
    0:  "Pata Frontal Direita  - Tíbia",
    1:  "Pata Frontal Direita  - Angular",
    2:  "Pata Frontal Direita  - Fêmur",
    3:  "Canal Livre 3",
    4:  "Canal Livre 4",
    5:  "Pata Frontal Esquerda - Tíbia",
    6:  "Pata Frontal Esquerda - Angular",
    7:  "Pata Frontal Esquerda - Fêmur",
    8:  "Pata Traseira Esquerda - Fêmur",
    9:  "Pata Traseira Esquerda - Angular",
    10: "Pata Traseira Esquerda - Tíbia",
    11: "Canal Livre 11",
    12: "Canal Livre 12",
    13: "Pata Traseira Direita  - Fêmur",
    14: "Pata Traseira Direita  - Angular",
    15: "Pata Traseira Direita  - Tíbia",
}

for i in range(16):
    kit.servo[i].actuation_range = 180
    kit.servo[i].set_pulse_width_range(440, 2560)




# ── OFFSETS POR CANAL ──────────────────────────────────────────────────────────
# Offset aditivo em graus aplicado sobre o ângulo calculado pela IK.
# Derivado de servo_orientation_adjusts (Config.py):
#   0    → servo na orientação direta  (sem inversão)
#   -180 → servo invertido             (angulo_final = angulo + (-180), após clip)
#
#        canal : offset
OFFSETS: dict[int, float] = {
    0:   0.0,    # FR - Tíbia    (sem inversão)
    1:   0.0,    # FR - Angular  (sem inversão)
    2:   0.0,    # FR - Fêmur    (sem inversão)
    3:   0.0,    # Canal Livre
    4:   0.0,    # Canal Livre
    5: -180.0,   # FL - Tíbia    (invertido)
    6: -180.0,   # FL - Angular  (invertido)
    7: -180.0,   # FL - Fêmur    (invertido)
    8: -180.0,   # RL - Fêmur    (invertido)
    9:   0.0,   # RL - Angular  (invertido)
    10: -180.0,  # RL - Tíbia    (invertido)
    11:  0.0,    # Canal Livre
    12:  0.0,    # Canal Livre
    13:  0.0,    # RR - Fêmur    (sem inversão)
    14: -180.0,  # RR - Angular  (invertido)
    15:  0.0,    # RR - Tíbia    (sem inversão)
}



#IMPORTS CONFIG CONTROLE
from evdev import InputDevice, list_devices, ecodes, categorize 

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

# ── 3. ENVIO DE SINAIS ────────────────────────────────────────────────────────
def enviar_sinal(canal: int, angulo: float):
    """Envia o comando de ângulo (0° a 180°) para o servo da porta escolhida."""
    kit.servo[canal].angle = abs(-angulo-OFFSETS[canal])
    print(f"-> Sinal enviado: kit.servo[{canal}].angle = {abs(-angulo-OFFSETS[canal])}° ({PORTAS[canal]})")


# ── 4. LOOP PRINCIPAL ──────────────────────────────────────────────────────────
def main():
    print("\n" + "=" * 50)
    print("       CONTROLE SIMPLES DE SERVOS (PCA9685)")
    print("=" * 50)
    for canal, desc in PORTAS.items():
        print(f"  Porta {canal:2d} -> {desc}")
    print("=" * 50)

    while True:
        try:
            print("\n[Comandos: digite 'sair' para encerrar")
            entrada_porta = input("Qual porta você quer controlar (0 a 15)? ").strip().lower()

            if entrada_porta in ["sair", "exit"]:
                break

            if entrada_porta in ["cal"]:
                for i in range(16):
                    enviar_sinal(i, 90)
                continue

            if not entrada_porta.isdigit() or not (0 <= int(entrada_porta) <= 15):
                print("Porta inválida! Digite um número de 0 a 15.")
                continue


            canal = int(entrada_porta)
            print(f"Porta selecionada: {canal} ({PORTAS[canal]})")

            entrada_valor = input("Digite o ângulo (0 a 180)").strip().lower()

            if entrada_valor in ["sair", "exit"]:
                break


            # Converte e envia o ângulo
            angulo = float(entrada_valor)
            if 0 <= angulo <= 180:
                enviar_sinal(canal, angulo)
            else:
                print("Ângulo fora do limite! Escolha um valor entre 0 e 180.")

        except ValueError:
            print("Entrada inválida! Digite um número válido")
        except KeyboardInterrupt:
            print("\nInterrompido via teclado.")
            break
        except Exception as e:
            print(f"Erro inesperado: {e}")
            break

    # Garante que ao encerrar o script os servos sejam liberados
    print("\nPrograma finalizado")


if __name__ == "__main__":
    main()
