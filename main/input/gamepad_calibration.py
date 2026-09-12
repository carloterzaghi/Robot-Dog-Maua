"""
input/gamepad_calibration.py — Modo de calibração interativo via gamepad.

Implementa uma máquina de estados de 3 níveis ativada a partir do main.py
quando o usuário pressiona Y na tela inicial.

Ao entrar no modo config, todos os 12 servos vão para DEFAULT_ANGLES
(definidos em configs/robot_config.py), que é a posição de referência
usada para o cálculo de offsets.

Mapeamento de botões (Gamesir):
  [X]  BTN_WEST (308)  — Seleciona Frente Dir / Fêmur
  [Y]  BTN_NORTH (307) — Seleciona Frente Esq / Tíbia
  [B]  BTN_EAST (305)  — Seleciona Trás Dir   / Angular
  [A]  BTN_SOUTH (304) — Seleciona Trás Esq  / Confirmar-e-Voltar / Voltar-e-Salvar
  Analógico Esq. Y — Incrementa/decrementa o servo selecionado (+1° / -1°)
  START            — Sai do modo de calibração

Fluxo de estados:
  CONFIG_MENU (escolha de perna)
      └─→ JOINT_MENU (escolha de articulação ou Voltar-e-Salvar)
               └─→ ADJUSTING (analógico ajusta, A confirma e volta ao JOINT_MENU)
"""

from __future__ import annotations

import time
from enum import Enum, auto
from typing import TYPE_CHECKING

from configs.robot_config import DEFAULT_ANGLES

if TYPE_CHECKING:
    from core.servo_manager import ServoManager
    from input.gamepad import GamepadReader


# ── Constantes ────────────────────────────────────────────────────────────────

# Delay entre incrementos ao manter o analógico (segundos)
ANALOG_STEP_DELAY: float = 0.10   # 100 ms → mais preciso

# Valor mínimo do analógico para considerar como movimento intencional
ANALOG_THRESHOLD: float = 0.20

# Debounce de botão (segundos)
BUTTON_DEBOUNCE: float = 0.25

# Mapeamento perna → chaves dos servos no ServoManager
LEG_SERVO_KEYS: dict[str, dict[str, str]] = {
    "frente_dir": {
        "femur":   "frente_femur_dir",
        "tibia":   "frente_tibia_dir",
        "angular": "frente_angular_dir",
    },
    "frente_esq": {
        "femur":   "frente_femur_esq",
        "tibia":   "frente_tibia_esq",
        "angular": "frente_angular_esq",
    },
    "tras_dir": {
        "femur":   "tras_femur_dir",
        "tibia":   "tras_tibia_dir",
        "angular": "tras_angular_dir",
    },
    "tras_esq": {
        "femur":   "tras_femur_esq",
        "tibia":   "tras_tibia_esq",
        "angular": "tras_angular_esq",
    },
}


# ── Estados ───────────────────────────────────────────────────────────────────

class CalibState(Enum):
    CONFIG_MENU = auto()   # escolha de perna
    JOINT_MENU  = auto()   # escolha de articulação (ou Voltar)
    ADJUSTING   = auto()   # ajuste fino pelo analógico


# ── Classe principal ──────────────────────────────────────────────────────────

class GamepadCalibration:
    """
    Máquina de estados de calibração navegada pelo gamepad.

    Uso::

        calib = GamepadCalibration(servo_mgr, reader)
        calib.run()
    """

    def __init__(self, servo_mgr: "ServoManager", reader: "GamepadReader") -> None:
        self._mgr    = servo_mgr
        self._reader = reader

        # Estado atual da máquina
        self._state: CalibState = CalibState.CONFIG_MENU

        # Perna e articulação selecionadas
        self._current_leg:   str | None = None   # ex: "frente_dir"
        self._current_joint: str | None = None   # "femur" | "tibia" | "angular"

        # Ângulos iniciais registrados no momento da entrada no modo config,
        # usados para calcular o Δ no print final.
        self._initial_angles: dict[str, float] = {}

        # Valores em memória ajustados durante a sessão
        # (acumulados; só vão para o .json no Voltar do JOINT_MENU)
        self._session_angles: dict[str, float] = {}

        # Controle de tempo para debounce e step do analógico
        self._last_button_ts: float = 0.0
        self._last_analog_ts: float = 0.0

    # ── Ponto de entrada ──────────────────────────────────────────────────────

    def run(self) -> None:
        """Loop principal do modo de calibração."""
        ecodes = self._reader._ecodes

        self._enter_config_menu(initial_setup=True)

        try:
            for event in self._reader.events():
                if event.type == ecodes.EV_SYN:
                    continue

                # ── Botão ─────────────────────────────────────────────────────
                if event.type == ecodes.EV_KEY and event.value == 1:
                    code = event.code

                    # START sai do modo a qualquer momento
                    if code in self._reader.EXIT_BUTTONS:
                        print("\n[START] Saindo do modo de calibração...")
                        break

                    btn_tl = getattr(ecodes, "BTN_TL", 310)
                    btn_tr = getattr(ecodes, "BTN_TR", 311)
                    
                    # Ajuste fino com TL (decrementa) e TR (incrementa) sem debounce global
                    if code in (btn_tl, btn_tr):
                        if self._state == CalibState.ADJUSTING:
                            now = time.monotonic()
                            if now - self._last_analog_ts >= ANALOG_STEP_DELAY:
                                self._last_analog_ts = now
                                delta = -1.0 if code == btn_tl else 1.0
                                self._adjust_servo(delta)
                        continue

                    # Debounce
                    now = time.monotonic()
                    if now - self._last_button_ts < BUTTON_DEBOUNCE:
                        continue
                    self._last_button_ts = now

                    self._handle_button(code, ecodes)

        except KeyboardInterrupt:
            print("\nInterrompido.")
        finally:
            self._print_all_offsets()
            print("Modo de calibração encerrado.")

    # ── Máquina de estados ────────────────────────────────────────────────────

    def _handle_button(self, code: int, ecodes) -> None:
        """Despacha o botão pressionado para o handler do estado atual."""
        btn_a = getattr(ecodes, "BTN_SOUTH", 304)   # A
        btn_b = getattr(ecodes, "BTN_EAST", 305)    # B
        btn_x = getattr(ecodes, "BTN_WEST", 308)    # X
        btn_y = getattr(ecodes, "BTN_NORTH", 307)   # Y

        if self._state == CalibState.CONFIG_MENU:
            self._handle_config_menu(code, btn_x, btn_y, btn_b, btn_a)

        elif self._state == CalibState.JOINT_MENU:
            self._handle_joint_menu(code, btn_x, btn_y, btn_b, btn_a)

        elif self._state == CalibState.ADJUSTING:
            if code == btn_a:   # A → confirma ajuste e volta ao JOINT_MENU
                self._confirm_adjustment()

    # ── CONFIG_MENU ───────────────────────────────────────────────────────────

    def _enter_config_menu(self, initial_setup: bool = False) -> None:
        """Move todos os 12 servos para DEFAULT_ANGLES (se initial_setup) e exibe o menu."""
        if initial_setup:
            from motion import poses

            print("\n" + "═" * 56)
            print("  MODO DE CALIBRAÇÃO VIA GAMEPAD — Robot-Dog-Maua")
            print("═" * 56)

            print("Levantando o robô suavemente (POSE_STAND)...")
            poses.stand(self._mgr)

            print("Movendo servos para posição default de calibração (DEFAULT_ANGLES)...")
            poses.transition_to(self._mgr, DEFAULT_ANGLES)

            # Após mover para DEFAULT_ANGLES, limpa _session_angles para que
            # _get_servo_angle() retorne o ângulo real do servo (DEFAULT + offset)
            # em vez de valores antigos do servo_state.json.
            self._session_angles.clear()

            # Registra ângulos iniciais de todos os servos para cálculo do Δ
            self._initial_angles = self._snapshot_all_angles()

        self._state = CalibState.CONFIG_MENU
        self._print_config_menu()

    def _handle_config_menu(self, code, btn_x, btn_y, btn_b, btn_a) -> None:
        """Reage à escolha de perna."""
        mapping = {
            btn_x: "frente_dir",
            btn_y:  "frente_esq",
            btn_b:  "tras_dir",
            btn_a: "tras_esq",
        }
        leg = mapping.get(code)
        if leg is None:
            return

        self._current_leg = leg
        label = {
            "frente_dir": "Frente Direita",
            "frente_esq": "Frente Esquerda",
            "tras_dir":   "Trás Direita",
            "tras_esq":   "Trás Esquerda",
        }[leg]
        print(f"\n  Perna selecionada: {label}")
        self._enter_joint_menu()

    # ── JOINT_MENU ────────────────────────────────────────────────────────────

    def _enter_joint_menu(self) -> None:
        """Entra no menu de seleção de articulação."""
        self._state = CalibState.JOINT_MENU
        self._print_joint_menu()

    def _handle_joint_menu(self, code, btn_x, btn_y, btn_b, btn_a) -> None:
        """Reage à escolha de articulação ou ao Voltar-e-Salvar."""
        joint_map = {
            btn_x: "femur",
            btn_y:  "tibia",
            btn_b:  "angular",
        }

        if code in joint_map:
            self._current_joint = joint_map[code]
            label = self._current_joint.capitalize()
            servo_key = LEG_SERVO_KEYS[self._current_leg][self._current_joint]
            current_angle = self._get_servo_angle(servo_key)
            print(f"\n  Articulação: {label}  |  Servo: {servo_key}  |  Ângulo atual: {current_angle:.1f}°")
            print("  Use [TL]/[TR] (L1/R1) para ajustar o ângulo. Pressione [A] para confirmar.")
            self._state = CalibState.ADJUSTING

        elif code == btn_a:
            # A no JOINT_MENU = Voltar e Salvar
            self._save_and_print()
            self._enter_config_menu(initial_setup=False)   # volta ao menu de pernas

    # ── ADJUSTING ─────────────────────────────────────────────────────────────

    def _adjust_servo(self, delta: float) -> None:
        """Aplica +1° ou -1° ao servo da articulação selecionada."""
        if self._current_leg is None or self._current_joint is None:
            return

        servo_key = LEG_SERVO_KEYS[self._current_leg][self._current_joint]
        current   = self._get_servo_angle(servo_key)
        new_angle = max(0.0, min(180.0, current + delta))

        try:
            self._mgr[servo_key].angle = new_angle
            self._session_angles[servo_key] = new_angle
            sign = "+" if delta > 0 else ""
            # Atualiza a linha atual em vez de imprimir várias linhas
            msg = f"  {servo_key}  {sign}{delta:.0f}°  →  {new_angle:.1f}°"
            print(f"\r{msg.ljust(50)}", end="", flush=True)
        except Exception as exc:
            msg = f"  Erro ao mover {servo_key}: {exc}"
            print(f"\r{msg.ljust(50)}")

    def _confirm_adjustment(self) -> None:
        """A pressionado no ADJUSTING: salva em memória e volta ao JOINT_MENU."""
        servo_key = LEG_SERVO_KEYS[self._current_leg][self._current_joint]
        angle = self._get_servo_angle(servo_key)
        self._session_angles[servo_key] = angle
        print(f"\n  ✓ {servo_key} confirmado em {angle:.1f}°")
        self._current_joint = None
        self._enter_joint_menu()

    # ── Salvar ────────────────────────────────────────────────────────────────

    def _save_and_print(self) -> None:
        """
        Calcula offsets de calibração (ângulo ajustado − DEFAULT_ANGLE),
        persiste em calibration.json e exibe um resumo das alterações.
        """
        # ── Calcula e salva offsets em calibration.json ────────────────────
        for key, angle in self._session_angles.items():
            default = DEFAULT_ANGLES.get(key, 90.0)
            self._mgr.offsets[key] = angle - default

        self._mgr.save_calibration()

        # ── Print do diff ──────────────────────────────────────────────────
        changed = {
            k: v for k, v in self._session_angles.items()
            if abs(v - self._initial_angles.get(k, v)) >= 0.1
        }

        print("\n" + "═" * 60)
        print("  OFFSETS SALVOS — Robot-Dog-Maua")
        print("─" * 60)
        if changed:
            for name, new_val in sorted(changed.items()):
                old_val = self._initial_angles.get(name, new_val)
                default = DEFAULT_ANGLES.get(name, 90.0)
                offset  = self._mgr.offsets.get(name, 0.0)
                sign    = "+" if offset >= 0 else ""
                print(
                    f"  {name:<26}  {new_val:>6.1f}°"
                    f"  (default {default:>6.1f}°,  offset = {sign}{offset:.1f}°)"
                )
            print()
            print("  Offsets salvos em calibration.json:")
            print()
            for name in sorted(changed.keys()):
                offset = self._mgr.offsets.get(name, 0.0)
                print(f'    "{name}": {offset:.1f},')
        else:
            print("  Nenhuma alteração detectada.")
        print("═" * 60 + "\n")

        # Zera os ajustes da sessão após salvar
        self._session_angles.clear()
        # Atualiza snapshot inicial para refletir os valores salvos
        self._initial_angles = self._snapshot_all_angles()

    def _print_all_offsets(self) -> None:
        """Exibe todos os offsets configurados ao sair do modo de calibração."""
        has_offsets = any(abs(offset) >= 0.1 for offset in self._mgr.offsets.values())
        if not has_offsets:
            return

        print("\n" + "═" * 60)
        print("  RESUMO DE TODOS OS OFFSETS — Robot-Dog-Maua")
        print("─" * 60)
        
        for name in sorted(self._mgr.offsets.keys()):
            offset = self._mgr.offsets[name]
            if abs(offset) >= 0.1:
                default = DEFAULT_ANGLES.get(name, 90.0)
                current = default + offset
                sign = "+" if offset > 0 else ""
                print(
                    f"  {name:<26}  {current:>6.1f}°"
                    f"  (default {default:>6.1f}°,  offset = {sign}{offset:.1f}°)"
                )
                
        print()
        print("  Offsets atuais em calibration.json:")
        print()
        for name in sorted(self._mgr.offsets.keys()):
            offset = self._mgr.offsets[name]
            if abs(offset) >= 0.1:
                print(f'    "{name}": {offset:.1f},')
        print("═" * 60 + "\n")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _get_servo_angle(self, key: str) -> float:
        """Retorna o ângulo da sessão se disponível, senão DEFAULT + offset.

        Usa DEFAULT + offset em vez de ler servo.angle do hardware para
        evitar imprecisão de quantização PWM (ex: 109.7° em vez de 110.0°).
        """
        if key in self._session_angles:
            return self._session_angles[key]
        # Valor esperado = DEFAULT + offset de calibração
        return DEFAULT_ANGLES.get(key, 90.0) + self._mgr.offsets.get(key, 0.0)

    def _snapshot_all_angles(self) -> dict[str, float]:
        """Captura o ângulo atual de todos os servos conhecidos."""
        snapshot: dict[str, float] = {}
        for leg_keys in LEG_SERVO_KEYS.values():
            for key in leg_keys.values():
                snapshot[key] = self._get_servo_angle(key)
        return snapshot

    # ── Prints de menu ────────────────────────────────────────────────────────

    @staticmethod
    def _print_config_menu() -> None:
        print("\n  ┌─ SELECIONE A PERNA ──────────────────────────────┐")
        print("  │  [X]  Frente Direita                              │")
        print("  │  [Y]  Frente Esquerda                             │")
        print("  │  [B]  Trás Direita                                │")
        print("  │  [A]  Trás Esquerda                               │")
        print("  │  [START] Sair do modo de calibração               │")
        print("  └───────────────────────────────────────────────────┘")

    def _print_joint_menu(self) -> None:
        leg_label = {
            "frente_dir": "Frente Direita",
            "frente_esq": "Frente Esquerda",
            "tras_dir":   "Trás Direita",
            "tras_esq":   "Trás Esquerda",
        }.get(self._current_leg, self._current_leg)

        keys = LEG_SERVO_KEYS.get(self._current_leg, {})
        femur_angle   = self._get_servo_angle(keys.get("femur",   ""))
        tibia_angle   = self._get_servo_angle(keys.get("tibia",   ""))
        angular_angle = self._get_servo_angle(keys.get("angular", ""))

        print(f"\n  ┌─ {leg_label.upper()} — SELECIONE A ARTICULAÇÃO ─┐")
        print(f"  │  [X]  Fêmur    ({femur_angle:>6.1f}°)                    │")
        print(f"  │  [Y]  Tíbia    ({tibia_angle:>6.1f}°)                    │")
        print(f"  │  [B]  Angular  ({angular_angle:>6.1f}°)                    │")
        print(f"  │  [A]  Voltar e Salvar                             │")
        print(f"  └───────────────────────────────────────────────────┘")


# ── Função de conveniência ────────────────────────────────────────────────────

def run_gamepad_calibration(
    servo_mgr: "ServoManager",
    reader: "GamepadReader",
) -> None:
    """Inicializa e executa o modo de calibração via gamepad."""
    calib = GamepadCalibration(servo_mgr, reader)
    calib.run()
