"""
calibration.py — Modo de calibração interativo do Robot-Dog-Maua.

Move o robô para a posição default e permite ajustar o ângulo de cada servo
individualmente via terminal. Ao salvar, calcula os offsets (ângulo ajustado
− ângulo default) e os persiste em configs/calibration.json.

Uso (na Raspberry Pi, dentro do diretório main/):
    python calibration.py

Comandos disponíveis no modo interativo:
  <número> <ângulo>  — move o servo indicado para o ângulo (ex: 3 120)
  +<número> <delta>  — adiciona delta ao ângulo atual do servo (ex: +3 5)
  -<número> <delta>  — subtrai delta do ângulo atual do servo (ex: -3 5)
  resetar            — volta todos os servos para a posição default
  salvar             — calcula e salva os offsets em calibration.json
  sair               — encerra o modo de calibração
"""

import sys
import os

# Garante que os imports relativos funcionem mesmo rodando diretamente
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from adafruit_servokit import ServoKit  # type: ignore

from configs.robot_config import DEFAULT_ANGLES, SERVO_CHANNELS, SMOOTH_N_STEPS, SMOOTH_DELAY
from core.servo_manager import ServoManager


# Ordem de exibição dos servos na tabela
SERVO_NAMES = [
    "frente_femur_dir",
    "frente_angular_dir",
    "frente_tibia_dir",
    "frente_femur_esq",
    "frente_angular_esq",
    "frente_tibia_esq",
    "tras_femur_dir",
    "tras_angular_dir",
    "tras_tibia_dir",
    "tras_femur_esq",
    "tras_angular_esq",
    "tras_tibia_esq",
]


def _print_table(servo_mgr: ServoManager) -> None:
    """Exibe uma tabela formatada com o estado atual de todos os servos."""
    print(f"\n  {'#':>2}  {'Nome':<22} {'Atual':>7}  {'Default':>8}  {'Offset':>7}")
    print("  " + "─" * 52)
    for i, name in enumerate(SERVO_NAMES, start=1):
        servo = servo_mgr[name]
        angle = servo.angle
        angle_str   = f"{angle:.1f}°" if angle is not None else "   N/A"
        default_str = f"{DEFAULT_ANGLES[name]:.1f}°"
        offset_str  = f"{servo_mgr.offsets.get(name, 0.0):+.1f}°"
        print(f"  {i:>2}  {name:<22} {angle_str:>7}  {default_str:>8}  {offset_str:>7}")


def _move_to_default(servo_mgr: ServoManager) -> None:
    """Move suavemente todos os servos para a posição default."""
    servo_mgr.smooth_move(DEFAULT_ANGLES, n_steps=SMOOTH_N_STEPS, delay=SMOOTH_DELAY)


def run_calibration(servo_mgr: ServoManager) -> None:
    """
    Loop interativo de calibração.

    O usuário seleciona cada servo pelo número (1-12) e informa o ângulo desejado.
    Ao digitar 'salvar', os offsets são calculados como:
        offset[servo] = ângulo_atual − ângulo_default
    e persistidos em configs/calibration.json via ServoManager.save_calibration().
    """
    print("\n" + "═" * 56)
    print("  MODO DE CALIBRAÇÃO — Robot-Dog-Maua")
    print("═" * 56)
    print("Movendo para posição default...")
    _move_to_default(servo_mgr)

    print("\nComandos disponíveis:")
    print("  <nº> <ângulo>    — move o servo (ex: 3 120)")
    print("  +<nº> <delta>    — adiciona delta ao ângulo (ex: +3 5)")
    print("  -<nº> <delta>    — subtrai delta do ângulo (ex: -3 5)")
    print("  resetar          — volta todos para o default")
    print("  salvar           — calcula e salva os offsets")
    print("  sair             — encerra")

    while True:
        _print_table(servo_mgr)

        try:
            raw = input("\n> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nInterrompido. Encerrando modo de calibração.")
            break

        if not raw:
            continue

        cmd = raw.lower()

        # ── Sair ──────────────────────────────────────────────────────────────
        if cmd == "sair":
            print("Saindo do modo de calibração.")
            break

        # ── Resetar para default ───────────────────────────────────────────────
        if cmd == "resetar":
            print("Resetando para posição default...")
            _move_to_default(servo_mgr)
            continue

        # ── Salvar offsets ─────────────────────────────────────────────────────
        if cmd == "salvar":
            new_offsets: dict[str, float] = {}
            for name in SERVO_NAMES:
                current = servo_mgr[name].angle
                if current is None:
                    current = DEFAULT_ANGLES[name]
                new_offsets[name] = round(current - DEFAULT_ANGLES[name], 2)

            servo_mgr.offsets = new_offsets
            servo_mgr.save_calibration()

            print("\n  ✓  Offsets salvos em configs/calibration.json")
            print("\n  Offsets calculados:")
            for name, off in new_offsets.items():
                print(f"     {name:<22}  {off:+.2f}°")
            continue

        # ── Ajuste de servo: <nº> <ângulo> ou ±<nº> <delta> ─────────────────
        parts = raw.split()
        if len(parts) != 2:
            print("  ✗  Formato inválido. Exemplos: '3 120'  '+3 5'  '-3 10'")
            continue

        raw_idx, raw_val = parts
        relative = False
        sign = 1

        # Detecta prefixo + / -
        if raw_idx.startswith("+"):
            relative = True
            sign = 1
            raw_idx = raw_idx[1:]
        elif raw_idx.startswith("-"):
            relative = True
            sign = -1
            raw_idx = raw_idx[1:]

        try:
            idx = int(raw_idx)
            value = float(raw_val)
        except ValueError:
            print("  ✗  Número ou ângulo inválido.")
            continue

        if not (1 <= idx <= len(SERVO_NAMES)):
            print(f"  ✗  Número do servo deve ser entre 1 e {len(SERVO_NAMES)}.")
            continue

        name = SERVO_NAMES[idx - 1]
        servo = servo_mgr[name]

        if relative:
            current = servo.angle if servo.angle is not None else DEFAULT_ANGLES[name]
            new_angle = current + sign * value
        else:
            new_angle = value

        if not (0 <= new_angle <= 180):
            print(f"  ✗  Ângulo {new_angle:.1f}° fora do intervalo [0, 180].")
            continue

        servo.angle = new_angle
        print(f"  ✓  {name} → {new_angle:.1f}°")


def main() -> None:
    """Ponto de entrada: inicializa o hardware e entra no modo de calibração."""
    kit = ServoKit(channels=16)
    servo_mgr = ServoManager(kit, channels=SERVO_CHANNELS)

    try:
        run_calibration(servo_mgr)
    except KeyboardInterrupt:
        # Ctrl+C fora do input() (ex: durante smooth_move) — encerra limpo
        print("\nInterrompido.")
    finally:
        # Sempre persiste, independente de como o programa terminou
        servo_mgr.save_calibration()
        servo_mgr.save_state()
        print("Calibração e estado salvos. Encerrando.")


if __name__ == "__main__":
    main()
