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


# ── Grupos laterais (misturador diferencial) ──────────────────────────────────
# O giro é feito por velocidade diferencial (tank/skid steer) entre os lados:
# v_esq = frente + yaw,  v_dir = frente - yaw  (convenção: yaw > 0 = girar à direita)
#
# Cada lado pode ter uma direção independente. A alternância das pernas
# (qual perna levanta ou apoia em um dado momento) é definida pela `phase`
# e sincronizada através da barreira de threads, mantendo o trot intocado.
# Isso permite que, em um giro puro (yaw alto), um lado mova suas pernas
# para frente e o outro para trás simultaneamente (girando o robô no eixo).
#
# A ALTERNÂNCIA de fases do trot não depende deste agrupamento: ela é
# garantida pelo phase de cada perna (LEG_CONFIG) + barreira de sincronia.
_SIDE_GROUPS: dict[str, str] = {
    "frente_dir": "right",
    "tras_dir":   "right",
    "frente_esq": "left",
    "tras_esq":   "left",
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

        # Stride scale gerenciado por LADO (esquerdo/direito).
        # Representa a velocidade atual com sinal (-1.0 a 1.0) do lado.
        # A rampa suaviza acelerações e inversões de sentido automaticamente.
        self._stride_scales: dict[str, float] = {"left": 0.0, "right": 0.0}

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
        self._cycle_params.clear()  # não vazar params da sessão anterior

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

        O giro usa um misturador diferencial de VELOCIDADE (com sinal) por lado:

            v_esq = frente + yaw      v_dir = frente - yaw

        (convenção: yaw > 0 = girar à direita; frente = ±|speed|)

        Cada lado pode ter uma direção (frente/trás) independente, permitindo o
        giro em torno do próprio eixo (skid steer). A sincronia do trot (quais
        pernas apoiam ou levantam juntas) não é afetada por direções opostas
        entre lados, sendo garantida puramente pela barreira e fase inicial (phase)
        de cada perna configurada em `leg.py`.

        Inversões de sentido (frente ↔ trás ou inversões de giro) passam por
        desaceleração suave: a velocidade em `_stride_scales` passa de positiva
        para negativa gradualmente, garantindo que a troca de `direction`
        ocorra com amplitude ~0.

        Chamado uma única vez por ciclo pela thread líder (barrier_id == 0).

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
        if abs(yaw) < 0.05:
            yaw = 0.0

        # Robô parado
        if speed == 0 and yaw == 0.0:
            for k in self._stride_scales:
                self._stride_scales[k] = 0.0
            return {
                "idle": True,
                "z_pitch_frente": shared_state.get("z_pitch_frente", 0.0),
                "z_pitch_tras":   shared_state.get("z_pitch_tras", 0.0),
            }

        # Velocidade longitudinal com sinal (+ frente / − trás)
        fwd = shared_state.get("direction", 1) * min(1.0, abs(speed))

        # Misturador diferencial: velocidade com sinal de cada lado
        v_left  = max(-1.0, min(1.0, fwd + yaw))
        v_right = max(-1.0, min(1.0, fwd - yaw))

        # Agora permitimos que cada lado tenha sua própria direção (tank steer).
        # A velocidade de cada lado (com sinal) é o alvo.
        target_side_v = {
            "left":  v_left,
            "right": v_right,
        }

        # Fator de giro (0.0 = só frente/trás, 1.0 = só giro).
        # Interpola walk_scale ↔ turn_scale quando os comandos são combinados.
        _denom = abs(fwd) + abs(yaw)
        turn_factor = abs(yaw) / _denom if _denom > 0.0 else 0.0

        directions_final:  dict[str, int]   = {}
        intensities_final: dict[str, float] = {}

        for side in ("left", "right"):
            # Rampa suave de velocidade com sinal (-1.0 a 1.0)
            # Ao inverter a direção, a amplitude naturalmente passa por 0,
            # evitando o "tranco" de inversão abrupta em amplitude cheia.
            current = self._stride_scales[side]
            delta   = target_side_v[side] - current
            delta   = max(-self._MAX_SCALE_STEP, min(self._MAX_SCALE_STEP, delta))
            new_v   = current + delta
            
            self._stride_scales[side] = new_v

            # A direção e a intensidade (amplitude) são derivadas da velocidade atual
            side_dir = 1 if new_v >= 0 else -1
            side_intensity = abs(new_v)

            for leg_name in [n for n, s in _SIDE_GROUPS.items() if s == side]:
                leg   = self._legs[leg_name]
                scale = leg.walk_scale + (leg.turn_scale - leg.walk_scale) * turn_factor
                directions_final[leg_name]  = side_dir
                intensities_final[leg_name] = side_intensity * scale

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
