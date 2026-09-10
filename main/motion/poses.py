"""
motion/poses.py — Poses predefinidas e transições do robô.

Substitui os métodos smooth_flexion_start() e smooth_sleep_robot()
da classe RobotLeg de main.py, tornando as transições de pose
configuráveis via dicionários em robot_config.py.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from configs.robot_config import (
    POSE_SLEEP,
    POSE_SLEEP_STRUCT,
    POSE_STAND,
    SMOOTH_DELAY,
    SMOOTH_N_STEPS,
)

if TYPE_CHECKING:
    from core.servo_manager import ServoManager


def transition_to(
    servo_mgr: "ServoManager",
    pose: dict[str, float],
    n_steps: int = SMOOTH_N_STEPS,
    delay: float = SMOOTH_DELAY,
) -> None:
    """
    Transição suave para qualquer pose predefinida.

    Args:
        servo_mgr: instância de ServoManager.
        pose:      dicionário {nome_servo: ângulo_alvo}.
        n_steps:   número de passos da interpolação.
        delay:     tempo entre passos (s).
    """
    servo_mgr.smooth_move(pose, n_steps=n_steps, delay=delay)


def stand(servo_mgr: "ServoManager", n_steps: int = SMOOTH_N_STEPS, delay: float = SMOOTH_DELAY) -> None:
    """
    Levanta o robô para a posição de pé (posição de início de caminhada).

    Sequência em 2 etapas para reduzir pico de corrente:
      1. Fêmures e tíbias → posição de pé.
      2. Angulares → posição neutra de caminhada.
    """
    stand_struct = {k: v for k, v in POSE_STAND.items() if "angular" not in k}
    stand_ang    = {k: v for k, v in POSE_STAND.items() if "angular"     in k}

    servo_mgr.smooth_move(stand_struct, n_steps=n_steps, delay=delay)
    servo_mgr.smooth_move(stand_ang,    n_steps=n_steps, delay=delay)


def sleep(servo_mgr: "ServoManager", n_steps: int = SMOOTH_N_STEPS, delay: float = SMOOTH_DELAY) -> None:
    """
    Deita o robô para a posição de descanso (sleep).

    Sequência em 3 etapas:
      1. Fêmures e tíbias → posição intermediária (estrutura dobrada).
      2. Angulares → posição deitada.
      3. Fêmures e tíbias → posição final de sleep.
    """
    sleep_ang    = {k: v for k, v in POSE_SLEEP.items() if "angular"     in k}
    sleep_struct = {k: v for k, v in POSE_SLEEP.items() if "angular" not in k}

    # 1. Dobra estrutura primeiro (evita bater no chão com os angulares)
    servo_mgr.smooth_move(POSE_SLEEP_STRUCT, n_steps=n_steps, delay=delay)

    # 2. Angulares deitam
    servo_mgr.smooth_move(sleep_ang, n_steps=n_steps, delay=delay)

    # 3. Ajusta fêmures/tíbias para posição final de repouso
    servo_mgr.smooth_move(sleep_struct, n_steps=n_steps, delay=delay)
