"""
core/servo_manager.py — Gerenciamento centralizado dos 12 servos.

Responsabilidades:
  - Inicializar servos a partir do ServoKit (adafruit_servokit).
  - Carregar/salvar calibração (calibration.json) e estado (servo_state.json).
  - Executar movimentação suave com Smootherstep (smooth_move).
  - Expor acesso individual a cada servo por nome.

Substitui os métodos _all_servos(), _load_state(), _save_state() e
_smooth_move() que estavam na classe monolítica RobotLeg de main.py,
eliminando ainda as 4 repetições do dicionário de 12 servos.
"""

from __future__ import annotations

import json
import os
import time
from typing import TYPE_CHECKING

import numpy as np

from configs.robot_config import (
    DEFAULT_ANGLES,
    SERVO_CHANNELS,
    SMOOTH_DELAY,
    SMOOTH_N_STEPS,
)

if TYPE_CHECKING:
    from adafruit_servokit import ServoKit  # type: ignore


# Caminhos dos arquivos de configuração (relativos ao diretório main/)
_HERE = os.path.dirname(os.path.abspath(__file__))
_CONFIGS_DIR = os.path.join(_HERE, "..", "configs")
_CALIB_FILE = os.path.join(_CONFIGS_DIR, "calibration.json")
_STATE_FILE = os.path.join(_CONFIGS_DIR, "servo_state.json")


class ServoManager:
    """
    Gerencia os 12 servos do robô quadrúpede.

    Uso::

        kit = ServoKit(channels=16)
        mgr = ServoManager(kit)
        mgr["frente_femur_dir"].angle = 90.0
        mgr.smooth_move({"frente_femur_dir": 60, "frente_tibia_dir": 45})
    """

    def __init__(self, kit: "ServoKit", channels: dict[str, int] = SERVO_CHANNELS) -> None:
        """
        Inicializa o gerenciador de servos.

        Args:
            kit:      instância do ServoKit (PCA9685, 16 canais).
            channels: mapeamento nome → canal PCA9685 (default: SERVO_CHANNELS).
        """
        # Mapeia nome → objeto servo
        self._servos: dict[str, object] = {
            name: kit.servo[ch] for name, ch in channels.items()
        }

        # Carrega offsets de calibração (ou zera se não existir)
        self.offsets: dict[str, float] = self._load_calibration()

        # Re-comanda cada servo para o último ângulo gravado,
        # evitando pico de corrente ao iniciar e dando ao ServoKit
        # um ponto de partida para as interpolações suaves.
        self._restore_state()

    # ── Acesso individual a servos ────────────────────────────────────────────

    def __getitem__(self, name: str):
        """Acessa o servo pelo nome: mgr['frente_femur_dir']."""
        return self._servos[name]

    def all_servos(self) -> dict[str, object]:
        """Retorna uma cópia do dict {nome: servo}."""
        return dict(self._servos)

    # ── Calibração ────────────────────────────────────────────────────────────

    def _load_calibration(self) -> dict[str, float]:
        """Lê calibration.json; retorna zeros se o arquivo não existir ou for inválido."""
        if os.path.exists(_CALIB_FILE):
            try:
                with open(_CALIB_FILE, "r") as f:
                    offsets = json.load(f)
                print(f"[Calibração] Offsets carregados de {_CALIB_FILE}")
                return offsets
            except Exception as exc:
                print(f"[Calibração] Falha ao carregar offsets: {exc} — usando zeros.")
        return {k: 0.0 for k in DEFAULT_ANGLES}

    def save_calibration(self) -> None:
        """Persiste os offsets atuais em calibration.json."""
        try:
            with open(_CALIB_FILE, "w") as f:
                json.dump(self.offsets, f, indent=4)
        except Exception as exc:
            print(f"[Calibração] Falha ao salvar: {exc}")

    # ── Estado ────────────────────────────────────────────────────────────────

    def _restore_state(self) -> None:
        """
        Lê servo_state.json e re-comanda cada servo para o último ângulo gravado.
        Como os servos já estão fisicamente nessa posição, nenhum movimento ocorre.
        O ServoKit passa a conhecer o ponto de partida para interpolações suaves.
        """
        if not os.path.exists(_STATE_FILE):
            return
        try:
            with open(_STATE_FILE, "r") as f:
                state = json.load(f)
            for name, servo in self._servos.items():
                angle = state.get(name)
                if angle is not None:
                    servo.angle = float(angle)
            print(f"[Estado] Posição anterior restaurada de {_STATE_FILE}")
        except Exception as exc:
            print(f"[Estado] Falha ao restaurar estado: {exc}")

    def save_state(self) -> None:
        """Grava o ângulo atual de todos os servos em servo_state.json."""
        state = {
            name: (srv.angle if srv.angle is not None else None)
            for name, srv in self._servos.items()
        }
        try:
            with open(_STATE_FILE, "w") as f:
                json.dump(state, f, indent=4)
        except Exception as exc:
            print(f"[Estado] Falha ao salvar estado: {exc}")

    # ── Movimentação suave ────────────────────────────────────────────────────

    def smooth_move(
        self,
        targets: dict[str, float],
        n_steps: int = SMOOTH_N_STEPS,
        delay: float = SMOOTH_DELAY,
    ) -> None:
        """
        Move os servos dos ângulos atuais até os alvos de forma gradual,
        usando Smootherstep (C²: 6t⁵ − 15t⁴ + 10t³) para eliminar picos
        de corrente no arranque e na parada.

        Apenas os servos presentes em `targets` são movidos.
        Os offsets de calibração são aplicados automaticamente.

        Args:
            targets:  {nome_servo: ângulo_alvo} — valores em graus.
            n_steps:  número de passos de interpolação.
            delay:    tempo de espera entre passos (segundos).
        """
        # Filtra apenas servos presentes nos alvos
        active = {
            name: srv
            for name, srv in self._servos.items()
            if name in targets
        }

        # Aplica offsets de calibração e limita ao intervalo [0°, 180°]
        adjusted = {
            name: max(0.0, min(180.0, targets[name] + self.offsets.get(name, 0.0)))
            for name in active
        }

        # Ângulo atual → usa alvo ajustado como fallback se ainda não foi definido
        starts = {
            name: (srv.angle if srv.angle is not None else adjusted[name])
            for name, srv in active.items()
        }

        for step in range(1, n_steps + 1):
            t = step / n_steps
            t_smooth = t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
            for name, srv in active.items():
                srv.angle = starts[name] + (adjusted[name] - starts[name]) * t_smooth
            time.sleep(delay)

        self.save_state()
