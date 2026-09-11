"""
motion/locomotion.py — Orquestrador de locomoção (GaitController).

Gerencia as threads de marcha das 4 pernas do robô, substituindo
a lógica de threading inline de gamesir_control_mode() em main.py.

A lógica de cada perna (swing, apoio, IK) vive em core/leg.py.
Aqui ficam apenas a orquestração das threads e a barreira de sincronização.

O cálculo de direction/intensity é centralizado aqui (não em cada Leg),
garantindo que pernas diagonais SEMPRE recebam os mesmos parâmetros
no mesmo ciclo — eliminando a dessincronização durante giros.
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

from configs.robot_config import LEG_CONFIG

if TYPE_CHECKING:
    from core.leg import Leg


# ── Grupos diagonais de marcha (trot) ─────────────────────────────────────────
# Pernas no mesmo grupo levantam (swing) ao mesmo tempo.
_DIAGONAL_GROUPS: dict[str, str] = {
    "frente_dir": "group_a",
    "tras_esq":   "group_a",
    "frente_esq": "group_b",
    "tras_dir":   "group_b",
}


class GaitController:
    """
    Gerencia a locomoção simultânea das 4 pernas via threads.

    Cada perna roda em uma thread daemon separada, sincronizadas
    por um threading.Barrier no início do ciclo de marcha.

    O cálculo de direction/intensity é feito centralmente por uma
    thread "líder" após a barreira, e compartilhado com todas as
    pernas via ``_cycle_params``.

    Uso::

        gait = GaitController(legs)
        gait.start(shared_state)
        # ... (robô andando) ...
        gait.stop()
    """

    # Delay pós-parada antes de executar transição de pose,
    # garantindo que as threads de locomoção terminaram de escrever nos servos.
    _POST_STOP_DELAY = 0.15

    # Variação máxima de stride_scale por ciclo de marcha.
    # Evita saltos bruscos ao trocar de andar para girar ou inverter sentido.
    _MAX_SCALE_STEP = 0.35

    def __init__(self, legs: dict[str, "Leg"]) -> None:
        """
        Args:
            legs: dicionário {nome_perna: instância Leg}.
                  Espera as chaves: "frente_dir", "frente_esq", "tras_dir", "tras_esq".
        """
        self._legs: dict[str, "Leg"] = legs
        self._stop_event = threading.Event()
        self._threads: list[threading.Thread] = []

        # Stride scale gerenciado por GRUPO DIAGONAL (não por perna individual).
        # Garante que pernas diagonais sempre tenham a mesma amplitude.
        self._stride_scales: dict[str, float] = {
            "group_a": 0.0,  # frente_dir + tras_esq
            "group_b": 0.0,  # frente_esq + tras_dir
        }

        # Parâmetros do ciclo atual, preenchidos pela thread líder.
        # Todas as threads lêem deste dict após a barreira.
        self._cycle_params: dict = {}
        self._cycle_ready = threading.Event()

    @property
    def is_running(self) -> bool:
        """True se as threads de locomoção estão ativas."""
        return any(t.is_alive() for t in self._threads)

    def start(self, shared_state: dict, use_angular: bool = False) -> None:
        """
        Inicia as threads de locomoção das 4 pernas.

        As pernas traseiras recebem um atraso de 0.5 s antes da descida
        inicial (delay_before_descent), mantendo a sincronização diagonal
        frente_dir↔tras_esq e frente_esq↔tras_dir.

        Args:
            shared_state: dicionário compartilhado com speed, direction, yaw, etc.
            use_angular:  se False, os angulares ficam fixos (para modo sem estabilização).
        """
        self._stop_event.clear()
        self._cycle_ready.clear()

        # Reseta stride scales ao iniciar uma nova sessão de locomoção
        for k in self._stride_scales:
            self._stride_scales[k] = 0.0

        # Barreira sincroniza as 4 pernas após a rampa de inicialização
        sync_barrier = threading.Barrier(len(self._legs))

        self._threads = []
        for name, leg in self._legs.items():
            delay_before = 0.5 if "tras" in name else 0.0

            t = threading.Thread(
                target=self._leg_worker,
                args=(leg, shared_state, delay_before, sync_barrier, use_angular),
                name=f"gait-{name}",
                daemon=True,
            )
            self._threads.append(t)
            t.start()

    def stop(self) -> None:
        """
        Para todas as threads de locomoção e aguarda a conclusão.

        Após retornar, é seguro chamar poses.stand() ou poses.sleep()
        sem risco de conflito de acesso aos servos.
        """
        self._stop_event.set()
        # Libera threads que possam estar esperando em _cycle_ready
        self._cycle_ready.set()
        for t in self._threads:
            t.join(timeout=3.0)
        time.sleep(self._POST_STOP_DELAY)
        self._threads = []

    # ── Cálculo centralizado dos parâmetros de ciclo ──────────────────────────

    def _compute_cycle_params(self, shared_state: dict) -> dict:
        """
        Calcula direction e intensity para CADA perna de forma atômica.

        Chamado uma única vez por ciclo pela thread líder (barrier_id == 0).
        O stride_scale é gerenciado por grupo diagonal, garantindo que
        pernas no mesmo grupo diagonal SEMPRE tenham a mesma amplitude.

        Returns:
            dict com:
              - "idle": True se robô parado
              - "directions": {nome_perna: direction} para cada perna
              - "intensities": {nome_perna: intensity} para cada perna
              - "z_pitch_frente": ajuste de pitch para grupo frente
              - "z_pitch_tras": ajuste de pitch para grupo tras
        """
        speed = shared_state.get("speed", 0)
        yaw   = shared_state.get("yaw", 0.0)

        # Robô parado
        if speed == 0 and abs(yaw) < 0.05:
            # Reseta stride scales quando parado
            for k in self._stride_scales:
                self._stride_scales[k] = 0.0
            return {
                "idle": True,
                "z_pitch_frente": shared_state.get("z_pitch_frente", 0.0),
                "z_pitch_tras":   shared_state.get("z_pitch_tras", 0.0),
            }

        # Determina direção e intensidade alvo POR PERNA
        directions_target: dict[str, int] = {}
        if abs(yaw) > abs(speed):
            # Modo giro: direção depende do lado da perna
            for name, leg in self._legs.items():
                if leg.mirror:
                    # Perna esquerda avança ao girar para a direita
                    directions_target[name] = 1 if yaw > 0 else -1
                else:
                    # Perna direita recua ao girar para a direita
                    directions_target[name] = -1 if yaw > 0 else 1
            target_intensity = min(1.0, abs(yaw))
        else:
            direction = shared_state.get("direction", 1)
            for name in self._legs:
                directions_target[name] = direction
            target_intensity = min(1.0, abs(speed))

        # Aplica rampa de stride_scale POR GRUPO DIAGONAL
        # (garante que pernas diagonais tenham EXATAMENTE a mesma amplitude)
        directions_final: dict[str, int] = {}
        intensities_final: dict[str, float] = {}

        for group_name in ("group_a", "group_b"):
            # Pega uma perna representativa do grupo para determinar a direção
            # (no modo giro, as duas pernas do grupo podem ter direções diferentes,
            #  então usamos a perna da frente como referência para o stride_scale)
            group_legs = [n for n, g in _DIAGONAL_GROUPS.items() if g == group_name]

            # Para o stride_scale, usamos a intensidade e a "direção predominante"
            # do grupo. No giro, cada perna tem sua própria direção, mas a
            # AMPLITUDE (intensity) deve ser idêntica.
            target_scale = target_intensity
            current = self._stride_scales[group_name]
            delta = target_scale - current
            delta = max(-self._MAX_SCALE_STEP, min(self._MAX_SCALE_STEP, delta))
            self._stride_scales[group_name] = current + delta

            group_intensity = self._stride_scales[group_name]

            for leg_name in group_legs:
                directions_final[leg_name] = directions_target[leg_name]
                intensities_final[leg_name] = group_intensity

        # Aplica turn_scale individual por perna
        for name, leg in self._legs.items():
            if abs(yaw) > abs(speed):
                intensities_final[name] *= leg.turn_scale

        return {
            "idle": False,
            "directions":    directions_final,
            "intensities":   intensities_final,
            "z_pitch_frente": shared_state.get("z_pitch_frente", 0.0),
            "z_pitch_tras":   shared_state.get("z_pitch_tras", 0.0),
        }

    # ── Worker interno ────────────────────────────────────────────────────────

    def _leg_worker(
        self,
        leg: "Leg",
        shared_state: dict,
        delay_before: float,
        sync_barrier: threading.Barrier,
        use_angular: bool,
    ) -> None:
        """Thread target: rampa de inicialização → barreira → loop de marcha."""
        # 1. Rampa suave da posição atual até a posição inicial de marcha
        leg.ramp_to_start(
            stop_event=self._stop_event,
            use_angular=use_angular,
            delay_before=delay_before,
        )

        if self._stop_event.is_set():
            return

        # 2. Barreira: aguarda todas as pernas chegarem ao ponto de partida
        try:
            sync_barrier.wait(timeout=5.0)
        except Exception:
            pass

        if self._stop_event.is_set():
            return

        # 3. Loop de marcha contínuo com cálculo centralizado de parâmetros.
        #    A thread líder (barrier_id == 0) calcula os params do ciclo,
        #    e as demais threads esperam em _cycle_ready.
        leg.run_gait_loop(
            stop_event=self._stop_event,
            shared_state=shared_state,
            use_angular=use_angular,
            sync_barrier=sync_barrier,
            cycle_params_ref=self._cycle_params,
            cycle_ready=self._cycle_ready,
            compute_cycle_fn=lambda: self._compute_cycle_params(shared_state),
        )
