"""
core/leg.py — Classe Leg: abstração de uma perna genérica do robô.

Substitui as 4 funções quase-idênticas de auxiliar_funcs/leg_test.py
(frente_dir, frente_esq, tras_dir, tras_esq — ~160 linhas cada, ~640 no total)
por uma única classe parametrizada por LEG_CONFIG (~80 linhas de lógica real).

A diferença entre as 4 pernas resume-se a:
  - Quais servos usar (nome do servo = prefixo da perna)
  - Se os ângulos são espelhados (mirror=True para lado esquerdo)
  - Qual grupo de parâmetros de marcha usar ("frente" ou "tras")
  - Se começa no swing ou no apoio (phase_offset)
  - Faixa do servo angular (ang_min, ang_max)
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

import numpy as np

from configs.robot_config import (
    GAIT_PARAMS,
    LEG_CONFIG,
    N_RAMP,
    RAMP_DELAY,
)
from core.kinematics import cubic_bezier, ease, ik_to_servo_angles

if TYPE_CHECKING:
    from core.servo_manager import ServoManager


class Leg:
    """
    Representa uma perna do robô (fêmur + angular + tíbia).

    A lógica de IK, rampa de inicialização e ciclo de marcha é
    implementada uma única vez aqui e parametrizada via `config`.

    Args:
        name:      identificador da perna (ex: "frente_dir").
        servo_mgr: instância de ServoManager que gerencia os servos.
        config:    dicionário de configuração (entrada de LEG_CONFIG).

    Exemplo::

        mgr = ServoManager(kit)
        leg = Leg("frente_dir", mgr, LEG_CONFIG["frente_dir"])
        leg.move_to(0, -210)
    """

    def __init__(
        self,
        name: str,
        servo_mgr: "ServoManager",
        config: dict | None = None,
    ) -> None:
        self.name = name
        self._mgr = servo_mgr
        cfg = config or LEG_CONFIG[name]

        self.mirror:   bool  = cfg["mirror"]
        self.group:    str   = cfg["group"]      # "frente" | "tras"
        self.phase:    str   = cfg["phase"]      # "swing_first" | "stance_first"
        self.ang_min:  float = cfg["ang_min"]
        self.ang_max:  float = cfg["ang_max"]
        self.angular_fixed_angle: float = cfg.get("angular_fixed_angle", 100.0)
        self.turn_scale: float = cfg.get("turn_scale", 1.0)

        # Overrides por perna (z_apoio, z_swing) — usa LEG_CONFIG se existir,
        # senão cai no GAIT_PARAMS do grupo.
        gait_defaults = GAIT_PARAMS[self.group]
        self.z_apoio:  float = cfg.get("z_apoio",  gait_defaults["z_apoio"])
        self.z_swing:  float = cfg.get("z_swing",  gait_defaults["z_swing"])
        self.x_frente: float = cfg.get("x_frente", gait_defaults["x_frente"])
        self.x_atras:  float = cfg.get("x_atras",  gait_defaults["x_atras"])
        self.n_pontos: int   = cfg.get("n_pontos", gait_defaults["n_pontos"])
        self.gait_delay: float = cfg.get("delay",  gait_defaults["delay"])

        # Atalhos para os 3 servos desta perna.
        # Convenção de chave no ServoManager: "{posição}_{articulação}_{lado}"
        # Exemplo: name="frente_dir" → "frente_femur_dir", "frente_angular_dir", ...
        _pos, _side = name.rsplit("_", 1)
        self._femur_name   = f"{_pos}_femur_{_side}"
        self._angular_name = f"{_pos}_angular_{_side}"
        self._tibia_name   = f"{_pos}_tibia_{_side}"
        self._femur   = servo_mgr[self._femur_name]
        self._angular = servo_mgr[self._angular_name]
        self._tibia   = servo_mgr[self._tibia_name]

    # ── Propriedades de leitura de ângulo atual ───────────────────────────────

    @property
    def femur_angle(self) -> float | None:
        return self._femur.angle

    @property
    def tibia_angle(self) -> float | None:
        return self._tibia.angle

    @property
    def angular_angle(self) -> float | None:
        return self._angular.angle

    # ── Movimentação via IK ───────────────────────────────────────────────────

    def move_to(self, x: float, z: float) -> None:
        """
        Move o pé para a posição (x, z) em mm via cinemática inversa.
        O espelhamento para pernas esquerdas é aplicado automaticamente.
        Os offsets de calibração são aplicados via ServoManager.set_angle().
        """
        femur_deg, tibia_deg = ik_to_servo_angles(x, z, mirror=self.mirror)
        self._mgr.set_angle(self._femur_name, femur_deg)
        self._mgr.set_angle(self._tibia_name, tibia_deg)

    # ── Rampa de inicialização ────────────────────────────────────────────────

    def ramp_to_start(
        self,
        stop_event: threading.Event,
        use_angular: bool = True,
        delay_before: float = 0.0,
    ) -> None:
        """
        Move suavemente a perna para a posição inicial de marcha (x=0, z=Z_APOIO)
        usando a curva Smootherstep. Substitui o bloco de ~40 linhas repetido 4×.

        Args:
            stop_event:   Event para abortar o movimento.
            use_angular:  se True, inclui servo angular na rampa.
            delay_before: tempo de espera antes de iniciar (s).
        """
        z_apoio = self.z_apoio

        # Calcula ângulos alvo via IK
        femur_alvo, tibia_alvo = ik_to_servo_angles(0, z_apoio, mirror=self.mirror)
        angular_alvo = self.ang_min if use_angular else self.angular_fixed_angle

        # Ângulos atuais (fallback para o alvo se ainda não inicializado)
        femur_atual   = self._femur.angle   if self._femur.angle   is not None else femur_alvo
        tibia_atual   = self._tibia.angle   if self._tibia.angle   is not None else tibia_alvo
        angular_atual = self._angular.angle if self._angular.angle is not None else angular_alvo

        # Interpola com Smootherstep
        femur_ramp   = ease(femur_atual,   femur_alvo,   N_RAMP)
        tibia_ramp   = ease(tibia_atual,   tibia_alvo,   N_RAMP)
        angular_ramp = ease(angular_atual, angular_alvo, N_RAMP)

        if delay_before > 0:
            time.sleep(delay_before)

        for i in range(N_RAMP):
            if stop_event.is_set():
                return
            self._mgr.set_angle(self._femur_name,   femur_ramp[i])
            self._mgr.set_angle(self._tibia_name,   tibia_ramp[i])
            self._mgr.set_angle(self._angular_name, angular_ramp[i])
            time.sleep(RAMP_DELAY)

    # ── Ciclo de marcha ───────────────────────────────────────────────────────

    def run_gait_loop(
        self,
        stop_event: threading.Event,
        shared_state: dict | None = None,
        use_angular: bool = True,
        sync_barrier: threading.Barrier | None = None,
        cycle_params_ref: dict | None = None,
        cycle_ready: threading.Event | None = None,
        compute_cycle_fn=None,
    ) -> None:
        """
        Executa o loop de marcha contínuo (swing + apoio) até stop_event ser setado.

        A ordem swing/apoio é determinada por ``self.phase``:
          - "swing_first":  swing → apoio  (frente_dir, tras_esq)
          - "stance_first": apoio → swing  (frente_esq, tras_dir)

        Os parâmetros de direction e intensity são calculados centralmente
        pelo GaitController (via ``compute_cycle_fn``) e compartilhados entre
        todas as pernas via ``cycle_params_ref``. A thread que chega primeiro
        à barreira (barrier_id == 0) atua como "líder" e calcula os params;
        as demais esperam em ``cycle_ready``.

        Isso garante que pernas diagonais SEMPRE recebam os mesmos parâmetros
        no mesmo ciclo, eliminando a dessincronização durante giros.

        sync_barrier: se fornecida, todas as pernas esperam umas pelas outras
        ao final de cada ciclo completo — sem isso a fase entre frente/trás
        vai lentamente se perdendo (jitter de I2C/GIL acumulado ciclo a ciclo).
        """
        z_apoio_base = self.z_apoio
        z_swing      = self.z_swing
        x_frente     = self.x_frente
        x_atras      = self.x_atras
        n_pontos     = self.n_pontos
        delay        = self.gait_delay

        # Chave do z_pitch no shared_state (frente ou tras)
        pitch_key = f"z_pitch_{self.group}"

        while not stop_event.is_set():
            # ── Sincronização + cálculo centralizado de params ────────────────
            if sync_barrier is not None:
                try:
                    barrier_id = sync_barrier.wait(timeout=2.0)
                except threading.BrokenBarrierError:
                    sync_barrier.reset()
                    continue
                except Exception:
                    continue

                if stop_event.is_set():
                    return

                # Thread líder (barrier_id == 0) calcula os params do ciclo
                if barrier_id == 0 and compute_cycle_fn is not None:
                    cycle_ready.clear()
                    params = compute_cycle_fn()
                    cycle_params_ref.clear()
                    cycle_params_ref.update(params)
                    cycle_ready.set()
                elif cycle_ready is not None:
                    # Demais threads: esperam a líder terminar o cálculo
                    cycle_ready.wait(timeout=2.0)

                if stop_event.is_set():
                    return

            # ── Lê parâmetros do ciclo ────────────────────────────────────────
            if cycle_params_ref and cycle_params_ref.get("idle", False):
                # Robô parado: mantém pé no centro e aguarda
                z_pitch = cycle_params_ref.get(pitch_key, 0.0)
                z_apoio = z_apoio_base + z_pitch
                self.move_to(0, z_apoio)
                time.sleep(0.05)
                continue

            if cycle_params_ref and not cycle_params_ref.get("idle", True):
                # Parâmetros calculados centralmente pelo GaitController
                direction = cycle_params_ref["directions"][self.name]
                intensity = cycle_params_ref["intensities"][self.name]
                z_pitch = cycle_params_ref.get(pitch_key, 0.0)
                z_apoio = z_apoio_base + z_pitch
            elif shared_state is not None:
                # Fallback: leitura direta do shared_state (sem GaitController)
                z_pitch = shared_state.get(pitch_key, 0.0)
                z_apoio = z_apoio_base + z_pitch
                direction = shared_state.get("direction", 1)
                intensity = min(1.0, abs(shared_state.get("speed", 1)))
            else:
                z_apoio   = z_apoio_base
                direction = 1
                intensity = 1.0

            x_f = x_frente * intensity
            x_b = x_atras  * intensity

            # Escala a altura do swing junto com a amplitude horizontal:
            # sem isso, um passo curto (giro com baixa intensidade) ainda
            # eleva o pé na altura máxima, parecendo um "coice" para cima.
            z_swing_eff = z_apoio + (z_swing - z_apoio) * intensity

            # ── Fases de marcha ───────────────────────────────────────────────
            # No código original, a direção do angular depende da ORDEM:
            #   1ª fase (qualquer) → min→max (ang_ascending=True)
            #   2ª fase           → max→min (ang_ascending=False)
            if self.phase == "swing_first":
                self._swing_phase(stop_event, x_f, x_b, z_apoio, z_swing_eff,
                                  n_pontos, delay, direction, use_angular,
                                  ang_ascending=True)
                self._stance_phase(stop_event, x_f, x_b, z_apoio,
                                   n_pontos, delay, direction, use_angular,
                                   ang_ascending=False)
            else:  # stance_first
                self._stance_phase(stop_event, x_f, x_b, z_apoio,
                                   n_pontos, delay, direction, use_angular,
                                   ang_ascending=True)
                self._swing_phase(stop_event, x_f, x_b, z_apoio, z_swing_eff,
                                  n_pontos, delay, direction, use_angular,
                                  ang_ascending=False)

    # ── Fases internas ────────────────────────────────────────────────────────

    def _swing_phase(
        self,
        stop_event: threading.Event,
        x_f: float,
        x_b: float,
        z_apoio: float,
        z_swing: float,
        n: int,
        delay: float,
        direction: int,
        use_angular: bool,
        ang_ascending: bool = True,
    ) -> None:
        """
        Fase de SWING: pé no ar, move de atrás para a frente via curva de Bézier cúbica.
        P0 = decolagem (chão, atrás) → P3 = pouso (chão, frente).
        """
        swing_x, swing_z = cubic_bezier(
            p0=[x_b * direction, z_apoio],
            p1=[x_b * direction, z_swing],
            p2=[x_f * direction, z_swing],
            p3=[x_f * direction, z_apoio],
            n=n,
        )
        if ang_ascending:
            ang_seq = np.linspace(self.ang_min, self.ang_max, n)
        else:
            ang_seq = np.linspace(self.ang_max, self.ang_min, n)

        for i in range(n):
            if stop_event.is_set():
                return
            self.move_to(swing_x[i], swing_z[i])
            if use_angular:
                self._mgr.set_angle(self._angular_name, ang_seq[i])
            time.sleep(delay)

    def _stance_phase(
        self,
        stop_event: threading.Event,
        x_f: float,
        x_b: float,
        z_apoio: float,
        n: int,
        delay: float,
        direction: int,
        use_angular: bool,
        ang_ascending: bool = True,
    ) -> None:
        """
        Fase de APOIO: pé no chão, empurra de frente para trás em linha reta.
        """
        if ang_ascending:
            ang_seq = np.linspace(self.ang_min, self.ang_max, n)
        else:
            ang_seq = np.linspace(self.ang_max, self.ang_min, n)

        for i, x in enumerate(np.linspace(x_f * direction, x_b * direction, n)):
            if stop_event.is_set():
                return
            self.move_to(x, z_apoio)
            if use_angular:
                self._mgr.set_angle(self._angular_name, ang_seq[i])
            time.sleep(delay)

