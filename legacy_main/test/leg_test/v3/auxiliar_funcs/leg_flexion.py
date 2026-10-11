import threading
import time
import numpy as np
from numpy.linalg import norm
import math as m

# ── Parâmetros ────────────────────────────────────────────────────────────────
UPPER_LEG  = 120
LOWER_LEG  = 130
MAX_RADIUS = 220
MAX_Z      = -20   # mm — posição sleep (topo da faixa de z)
X_FIXO     =  -9   # mm — x de descanso do robô
N_PONTOS   = 70
DELAY      = 0.025


def _ease(start, end, n):
    """Interpolação smootherstep (C²): aceleração zero nas extremidades, sem pico de corrente."""
    t = np.linspace(0, 1, n)
    t_smooth = t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
    return start + (end - start) * t_smooth


def _point_to_rad(p1, p2):
    theta = m.atan2(p2, p1)
    return (theta + 2 * m.pi) % (2 * m.pi)


def _angle_corrector(angles):
    angles[1] = angles[0] + angles[1] - 5/4 * m.pi
    angles[0] = angles[0] - m.pi / 2
    return angles


def _ik(x, z):
    if z > MAX_Z:
        z = MAX_Z
    len_B = norm([x, 0, z])
    if len_B > MAX_RADIUS:
        scale = MAX_RADIUS / len_B
        x *= scale
        z *= scale
        len_B = MAX_RADIUS
    arg_b2 = np.clip((UPPER_LEG**2 + len_B**2 - LOWER_LEG**2) / (2 * UPPER_LEG * len_B), -1, 1)
    arg_b3 = np.clip((UPPER_LEG**2 + LOWER_LEG**2 - len_B**2) / (2 * UPPER_LEG * LOWER_LEG), -1, 1)
    b_1 = _point_to_rad(x, z)
    b_2 = m.acos(arg_b2)
    b_3 = m.acos(arg_b3)
    theta_2 = b_1 - b_2
    theta_3 = m.pi - b_3
    return _angle_corrector([theta_2, theta_3])


def _sweep(move_fn, stop_event, sync_barrier=None, delay_before_descent=0.0):
    """
    Varre a perna ao longo do eixo Z com X fixo em X_FIXO.

    Em cada ciclo do loop:
      1. Aguarda delay_before_descent (ex: 1s para patas de trás, 0s para as da frente).
      2. Desce do topo ao fundo.
      3. Aguarda no sync_barrier (no fundo) até que todas as patas cheguem.
      4. Sobe do fundo ao topo todas juntas.
      5. Repete o ciclo.
    """
    z_fundo = -np.sqrt(max(0.0, MAX_RADIUS**2 - X_FIXO**2))
    z_topo  = MAX_Z

    while not stop_event.is_set():
        # ── 1. Atraso antes de descer (ex: 1s para patas de trás) ─────────────
        if delay_before_descent > 0:
            t_end = time.monotonic() + delay_before_descent
            while time.monotonic() < t_end:
                if stop_event.is_set(): return
                time.sleep(min(0.05, t_end - time.monotonic()))

        # ── 2. Desce: topo -> fundo ───────────────────────────────────────────
        for z in _ease(z_topo, z_fundo, N_PONTOS):
            if stop_event.is_set(): return
            move_fn(X_FIXO, z)
            time.sleep(DELAY)

        # ── 3. Ponto de sincronização no fundo ────────────────────────────────
        if sync_barrier is not None:
            try:
                sync_barrier.wait()
            except threading.BrokenBarrierError:
                return

        # ── 4. Sobe: fundo -> topo ────────────────────────────────────────────
        for z in _ease(z_fundo, z_topo, N_PONTOS):
            if stop_event.is_set(): return
            move_fn(X_FIXO, z)
            time.sleep(DELAY)


def frente_dir(self, stop_event, sync_barrier=None, delay_before_descent=0.0):
    def move(x, z):
        angulos = _ik(x, z)
        self.frente_femur_dir.angle = angulos[0] * 180 / m.pi
        self.frente_tibia_dir.angle = angulos[1] * 180 / m.pi - 8
    _sweep(move, stop_event, sync_barrier=sync_barrier, delay_before_descent=delay_before_descent)


def frente_esq(self, stop_event, sync_barrier=None, delay_before_descent=0.0):
    def move(x, z):
        angulos = _ik(x, z)
        self.frente_femur_esq.angle = 180 - angulos[0] * 180 / m.pi
        self.frente_tibia_esq.angle = 180 - (angulos[1] * 180 / m.pi - 8)
    _sweep(move, stop_event, sync_barrier=sync_barrier, delay_before_descent=delay_before_descent)


def tras_dir(self, stop_event, sync_barrier=None, delay_before_descent=0.0):
    def move(x, z):
        angulos = _ik(x, z)
        femur_angle = max(0.0, min(180.0, angulos[0] * 180 / m.pi))
        tibia_angle = max(0.0, min(180.0, angulos[1] * 180 / m.pi - 8))
        self.tras_femur_dir.angle = femur_angle
        self.tras_tibia_dir.angle = tibia_angle
    _sweep(move, stop_event, sync_barrier=sync_barrier, delay_before_descent=delay_before_descent)


def tras_esq(self, stop_event, sync_barrier=None, delay_before_descent=0.0):
    def move(x, z):
        angulos = _ik(x, z)
        # Ângulos espelhados — servo montado em direção oposta (lado esquerdo)
        self.tras_femur_esq.angle = 180 - angulos[0] * 180 / m.pi
        self.tras_tibia_esq.angle = 180 - (angulos[1] * 180 / m.pi - 8)
    _sweep(move, stop_event, sync_barrier=sync_barrier, delay_before_descent=delay_before_descent)
