"""
auxiliar_funcs/leg_test.py — Shim de backward-compatibility.

Mantém a API pública original (frente_dir, frente_esq, tras_dir, tras_esq)
para que os scripts em test/ continuem funcionando sem modificação.

A implementação real foi movida para core/leg.py (classe Leg).

NOTA: Para novos scripts, use core/leg.py diretamente.
"""

import threading
import time

import numpy as np

from configs.robot_config import LEG_CONFIG, N_RAMP, RAMP_DELAY, GAIT_PARAMS
from core.kinematics import cubic_bezier, ease, ik_to_servo_angles


def _run_leg(
    leg_name: str,
    # Servos da perna (passados por referência a partir de self)
    femur_srv,
    angular_srv,
    tibia_srv,
    # Parâmetros de configuração
    mirror: bool,
    phase: str,
    ang_min: float,
    ang_max: float,
    angular_fixed: float,
    gait: dict,
    turn_scale: float,
    # Argumentos originais da função
    stop_event: threading.Event,
    use_angular: bool,
    delay_before_descent: float,
    sync_barrier,
    shared_state: dict | None,
) -> None:
    """Lógica unificada de locomoção para qualquer perna."""

    z_apoio_base = gait["z_apoio"]
    z_swing      = gait["z_swing"]
    x_frente     = gait["x_frente"]
    x_atras      = gait["x_atras"]
    n_pontos     = gait["n_pontos"]
    delay        = gait["delay"]
    pitch_key    = f"z_pitch_{gait['group']}"  # "z_pitch_frente" ou "z_pitch_tras"


    # ── Rampa de inicialização ────────────────────────────────────────────────
    femur_alvo, tibia_alvo = ik_to_servo_angles(0, z_apoio_base, mirror=mirror)
    angular_alvo = ang_min if use_angular else angular_fixed

    femur_atual   = femur_srv.angle   if femur_srv.angle   is not None else femur_alvo
    tibia_atual   = tibia_srv.angle   if tibia_srv.angle   is not None else tibia_alvo
    angular_atual = angular_srv.angle if angular_srv.angle is not None else angular_alvo

    femur_ramp   = ease(femur_atual,   femur_alvo,   N_RAMP)
    tibia_ramp   = ease(tibia_atual,   tibia_alvo,   N_RAMP)
    angular_ramp = ease(angular_atual, angular_alvo, N_RAMP)

    if delay_before_descent > 0:
        time.sleep(delay_before_descent)

    for i in range(N_RAMP):
        if stop_event.is_set():
            return
        femur_srv.angle   = femur_ramp[i]
        tibia_srv.angle   = tibia_ramp[i]
        angular_srv.angle = angular_ramp[i]
        time.sleep(RAMP_DELAY)

    if sync_barrier is not None:
        try:
            sync_barrier.wait(timeout=5)
        except Exception:
            pass

    # ── Loop de marcha ────────────────────────────────────────────────────────
    def move(x: float, z: float) -> None:
        fd, td = ik_to_servo_angles(x, z, mirror=mirror)
        femur_srv.angle = fd
        tibia_srv.angle = td

    while not stop_event.is_set():
        if shared_state is not None:
            z_pitch = shared_state.get(pitch_key, 0.0)
            z_apoio = z_apoio_base + z_pitch
            speed   = shared_state.get("speed", 0)
            yaw     = shared_state.get("yaw", 0.0)

            if speed == 0 and abs(yaw) < 0.05:
                move(0, z_apoio)
                time.sleep(0.05)
                continue

            if abs(yaw) > abs(speed):
                if mirror:
                    direction = 1 if yaw > 0 else -1   # esquerda: avança ao girar direita
                else:
                    direction = -1 if yaw > 0 else 1   # direita: recua ao girar direita
                intensity = min(1.0, abs(yaw)) * turn_scale
            else:
                direction = shared_state.get("direction", 1)
                intensity = min(1.0, abs(speed))
        else:
            z_apoio   = z_apoio_base
            direction = 1
            intensity = 1.0

        x_f = x_frente * intensity
        x_b = x_atras  * intensity

        def swing():
            sx, sz = cubic_bezier(
                [x_b * direction, z_apoio],
                [x_b * direction, z_swing],
                [x_f * direction, z_swing],
                [x_f * direction, z_apoio],
                n=n_pontos,
            )
            ang_seq = np.linspace(ang_min, ang_max, n_pontos)
            for i in range(n_pontos):
                if stop_event.is_set():
                    return
                move(sx[i], sz[i])
                if use_angular:
                    angular_srv.angle = ang_seq[i]
                time.sleep(delay)

        def stance():
            ang_seq = np.linspace(ang_max, ang_min, n_pontos)
            for i, x in enumerate(np.linspace(x_f * direction, x_b * direction, n_pontos)):
                if stop_event.is_set():
                    return
                move(x, z_apoio)
                if use_angular:
                    angular_srv.angle = ang_seq[i]
                time.sleep(delay)

        if phase == "swing_first":
            swing()
            stance()
        else:
            stance()
            swing()


# ── API pública (backward-compat) ─────────────────────────────────────────────

def frente_dir(self, stop_event, use_angular=True, delay_before_descent=0.0,
               sync_barrier=None, shared_state=None):
    cfg  = LEG_CONFIG["frente_dir"]
    gait = {**GAIT_PARAMS["frente"], "group": "frente"}
    _run_leg("frente_dir",
             self.frente_femur_dir, self.frente_angular_dir, self.frente_tibia_dir,
             cfg["mirror"], cfg["phase"], cfg["ang_min"], cfg["ang_max"],
             cfg["angular_fixed_angle"], gait, cfg.get("turn_scale", 1.0),
             stop_event, use_angular, delay_before_descent, sync_barrier, shared_state)


def frente_esq(self, stop_event, use_angular=True, delay_before_descent=0.0,
               sync_barrier=None, shared_state=None):
    cfg  = LEG_CONFIG["frente_esq"]
    gait = {**GAIT_PARAMS["frente"], "group": "frente"}
    _run_leg("frente_esq",
             self.frente_femur_esq, self.frente_angular_esq, self.frente_tibia_esq,
             cfg["mirror"], cfg["phase"], cfg["ang_min"], cfg["ang_max"],
             cfg["angular_fixed_angle"], gait, cfg.get("turn_scale", 1.0),
             stop_event, use_angular, delay_before_descent, sync_barrier, shared_state)


def tras_dir(self, stop_event, use_angular=True, delay_before_descent=0.0,
             sync_barrier=None, shared_state=None):
    cfg  = LEG_CONFIG["tras_dir"]
    gait = {**GAIT_PARAMS["tras"], "group": "tras"}
    _run_leg("tras_dir",
             self.tras_femur_dir, self.tras_angular_dir, self.tras_tibia_dir,
             cfg["mirror"], cfg["phase"], cfg["ang_min"], cfg["ang_max"],
             cfg["angular_fixed_angle"], gait, cfg.get("turn_scale", 1.0),
             stop_event, use_angular, delay_before_descent, sync_barrier, shared_state)


def tras_esq(self, stop_event, use_angular=True, delay_before_descent=0.0,
             sync_barrier=None, shared_state=None):
    cfg  = LEG_CONFIG["tras_esq"]
    gait = {**GAIT_PARAMS["tras"], "group": "tras"}
    _run_leg("tras_esq",
             self.tras_femur_esq, self.tras_angular_esq, self.tras_tibia_esq,
             cfg["mirror"], cfg["phase"], cfg["ang_min"], cfg["ang_max"],
             cfg["angular_fixed_angle"], gait, cfg.get("turn_scale", 1.0),
             stop_event, use_angular, delay_before_descent, sync_barrier, shared_state)
