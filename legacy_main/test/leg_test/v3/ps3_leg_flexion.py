#!/usr/bin/env python3
"""
Controle da flexão das pernas usando o controle PS3.

Analógico esquerdo:
  Y (ABS_Y) → sobe/desce todas as pernas  (-1 = cima, +1 = baixo)
  X (ABS_X) → desloca frente/trás          (±30 mm em torno de X_FIXO)

Botões:
  Cruz (×)   BTN_SOUTH → posição de descanso (sleep)
  △          BTN_NORTH → posição padrão
  Start      BTN_START → sair

Uso:
    cd test/leg_test/v3
    python3 ps4_leg_flexion.py
"""

import sys
import os
import time
import math as m
import numpy as np

# ── Paths ────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(_ROOT, "lib"))   # lib/ps3_control.py
sys.path.insert(0, _HERE)                         # auxiliar_funcs/

from ps3_control import PS3Controller             # type: ignore
from connect_ps3_control import ensure_connected  # type: ignore
from adafruit_servokit import ServoKit            # type: ignore
from auxiliar_funcs.leg_flexion import _ik, MAX_Z, MAX_RADIUS, X_FIXO

# ── Faixa de Z ───────────────────────────────────────────────
Z_FUNDO = float(-np.sqrt(max(0.0, MAX_RADIUS**2 - X_FIXO**2)))
Z_TOPO  = float(MAX_Z)

# Deadzone central do analógico (normalizado)
DEADZONE = 0.10


# ── Inicialização dos servos ─────────────────────────────────
kit = ServoKit(channels=16)

frente_femur_dir   = kit.servo[1]
frente_angular_dir = kit.servo[2]
frente_tibia_dir   = kit.servo[3]
frente_femur_esq   = kit.servo[5]
frente_angular_esq = kit.servo[6]
frente_tibia_esq   = kit.servo[7]


def _deadzone(val: float, dz: float = DEADZONE) -> float:
    """Remove ruído de centro: retorna 0 se |val| < dz, senão reescala."""
    if abs(val) < dz:
        return 0.0
    sign = 1.0 if val > 0 else -1.0
    return sign * (abs(val) - dz) / (1.0 - dz)


def aplicar_ik(x: float, z: float):
    """Calcula a cinemática inversa e move os servos de fêmur e tíbia."""
    angulos = _ik(x, z)
    a_femur = angulos[0] * 180 / m.pi
    a_tibia = angulos[1] * 180 / m.pi - 8

    frente_femur_dir.angle = a_femur
    frente_tibia_dir.angle = a_tibia
    frente_femur_esq.angle = 180 - a_femur
    frente_tibia_esq.angle = 180 - a_tibia


def sleep_robot():
    frente_femur_dir.angle   = 43
    frente_angular_dir.angle = 100
    frente_tibia_dir.angle   = 71
    frente_femur_esq.angle   = 137
    frente_angular_esq.angle = 105
    frente_tibia_esq.angle   = 109


def posicao_padrao():
    frente_femur_dir.angle   = 90
    frente_angular_dir.angle = 100
    frente_tibia_dir.angle   = 90
    frente_femur_esq.angle   = 90
    frente_angular_esq.angle = 105
    frente_tibia_esq.angle   = 90


def smooth_sleep(n_steps: int = 60, delay: float = 0.02):
    """Move suavemente até a posição de descanso."""
    targets = {
        "femur_dir": 43, "angular_dir": 100, "tibia_dir": 71,
        "femur_esq": 137, "angular_esq": 105, "tibia_esq": 109,
    }
    servos = {
        "femur_dir":   frente_femur_dir,
        "angular_dir": frente_angular_dir,
        "tibia_dir":   frente_tibia_dir,
        "femur_esq":   frente_femur_esq,
        "angular_esq": frente_angular_esq,
        "tibia_esq":   frente_tibia_esq,
    }
    starts = {
        name: (srv.angle if srv.angle is not None else targets[name])
        for name, srv in servos.items()
    }
    for step in range(1, n_steps + 1):
        t = step / n_steps
        # Smootherstep (C²): aceleração zero nas extremidades, sem pico de corrente
        t_smooth = t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
        for name, srv in servos.items():
            srv.angle = starts[name] + (targets[name] - starts[name]) * t_smooth
        time.sleep(delay)


# ── Main ─────────────────────────────────────────────────────
def main():
    print("Iniciando robô na posição de descanso...")
    sleep_robot()
    time.sleep(0.5)

    # Garante que o controle PS3 está conectado via sixad
    print("Conectando controle PS3...")
    device_path = ensure_connected(timeout=30)
    if not device_path:
        print("Falha ao conectar controle PS3.")
        sys.exit(1)

    # Inicializa o controlador
    ctrl = PS3Controller(deadzone=6, normalize=True)
    if not ctrl.connect(timeout=5.0):
        print("Controle PS3 não encontrado após conexão sixad.")
        sys.exit(1)

    print(f"Conectado: {ctrl.device_name} ({ctrl.device_path})")
    print()
    print("  Analógico esq. Y  → sobe/desce as pernas")
    print("  Analógico esq. X  → frente/trás (±30 mm)")
    print("  Cruz (×)          → posição de descanso")
    print("  Triângulo (△)     → posição padrão")
    print("  Start             → sair")
    print()

    running = True

    def on_south(estado):
        if estado == 1:
            print("→ Posição de descanso")
            smooth_sleep()

    def on_north(estado):
        if estado == 1:
            print("→ Posição padrão")
            posicao_padrao()

    def on_start(estado):
        nonlocal running
        if estado == 1:
            running = False

    ctrl.on_button("BTN_SOUTH", on_south)
    ctrl.on_button("BTN_NORTH", on_north)
    ctrl.on_button("BTN_START", on_start)

    ctrl.start()  # loop de eventos em background

    try:
        while running:
            # Lê eixos com deadzone central
            eixo_y = _deadzone(float(ctrl.axis("ABS_Y")))   # -1=cima, +1=baixo
            eixo_x = _deadzone(float(ctrl.axis("ABS_X")))   # -1=esq,  +1=dir

            # Mapeia Y → Z:  -1 → Z_TOPO (pernas em cima)  |  +1 → Z_FUNDO (pernas embaixo)
            z = Z_TOPO + (Z_FUNDO - Z_TOPO) * (eixo_y * 0.5 + 0.5)

            # Mapeia X → deslocamento em X (±30 mm)
            x = X_FIXO + eixo_x * 30.0

            aplicar_ik(x, z)
            time.sleep(0.02)   # ~50 Hz

    except KeyboardInterrupt:
        print("\nInterrompido.")
    finally:
        ctrl.stop()
        print("Movendo para posição de descanso...")
        smooth_sleep()
        print("Pronto.")


if __name__ == "__main__":
    main()
