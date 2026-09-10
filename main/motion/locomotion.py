"""
motion/locomotion.py — Orquestrador de locomoção (GaitController).

Gerencia as threads de marcha das 4 pernas do robô, substituindo
a lógica de threading inline de gamesir_control_mode() em main.py.

A lógica de cada perna (swing, apoio, IK) vive em core/leg.py.
Aqui ficam apenas a orquestração das threads e a barreira de sincronização.
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

from configs.robot_config import LEG_CONFIG

if TYPE_CHECKING:
    from core.leg import Leg


class GaitController:
    """
    Gerencia a locomoção simultânea das 4 pernas via threads.

    Cada perna roda em uma thread daemon separada, sincronizadas
    por um threading.Barrier no início do ciclo de marcha.

    Uso::

        gait = GaitController(legs)
        gait.start(shared_state)
        # ... (robô andando) ...
        gait.stop()
    """

    # Delay pós-parada antes de executar transição de pose,
    # garantindo que as threads de locomoção terminaram de escrever nos servos.
    _POST_STOP_DELAY = 0.15

    def __init__(self, legs: dict[str, "Leg"]) -> None:
        """
        Args:
            legs: dicionário {nome_perna: instância Leg}.
                  Espera as chaves: "frente_dir", "frente_esq", "tras_dir", "tras_esq".
        """
        self._legs: dict[str, "Leg"] = legs
        self._stop_event = threading.Event()
        self._threads: list[threading.Thread] = []

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
        for t in self._threads:
            t.join(timeout=3.0)
        time.sleep(self._POST_STOP_DELAY)
        self._threads = []

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

        # 3. Loop de marcha contínuo (re-sincroniza com as outras pernas a cada
        #    ciclo via sync_barrier, evitando deriva de fase entre frente/trás)
        leg.run_gait_loop(
            stop_event=self._stop_event,
            shared_state=shared_state,
            use_angular=use_angular,
            sync_barrier=sync_barrier,
        )
