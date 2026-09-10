"""
input/gamepad.py — Detecção e leitura de gamepad (evdev) + máquina de estados do robô.

Extrai as ~250 linhas de lógica de controle inline do método
gamesir_control_mode() de main.py, separando em duas classes:

  - GamepadReader: detecção do hardware e geração de eventos filtrados.
  - RobotController: máquina de estados (sleep ↔ standing) que
    reage aos eventos do gamepad e orquestra locomoção e poses.
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING

from motion import locomotion, poses

if TYPE_CHECKING:
    from core.leg import Leg
    from core.servo_manager import ServoManager
    from motion.locomotion import GaitController


# ── GamepadReader ─────────────────────────────────────────────────────────────

class GamepadReader:
    """
    Detecta e abre o primeiro gamepad disponível via evdev.

    Normaliza os eixos analógicos para [−1.0, 1.0] com zona morta de 15%.

    Uso::

        reader = GamepadReader()
        for event in reader.events():
            ...
    """

    DEAD_ZONE = 0.15  # zona morta dos analógicos (15%)

    def __init__(self) -> None:
        try:
            from evdev import InputDevice, ecodes, list_devices  # type: ignore
        except ImportError as exc:
            raise ImportError(
                f"evdev não encontrado: {exc}\n"
                "Instale com: pip install evdev"
            ) from exc

        self._ecodes = ecodes
        self._device = self._find_gamepad(list_devices, InputDevice)
        self._center_x, self._range_x = self._axis_info(ecodes.ABS_X)
        self._center_y, self._range_y = self._axis_info(ecodes.ABS_Y)

        # Whitelist de botões aceitos
        self.ALLOWED_BUTTONS: frozenset[int] = frozenset({
            ecodes.BTN_NORTH, ecodes.BTN_WEST, ecodes.BTN_TOP,
            getattr(ecodes, "BTN_Y", 308),
            getattr(ecodes, "BTN_X", 307),
            ecodes.BTN_START, 315,
        })
        self.ALLOWED_AXES: frozenset[int] = frozenset({ecodes.ABS_Y, ecodes.ABS_X})

        # Botões de ação (levantar/deitar)
        self.ACTION_BUTTONS: frozenset[int] = frozenset({
            ecodes.BTN_NORTH, ecodes.BTN_WEST, ecodes.BTN_TOP,
            getattr(ecodes, "BTN_Y", 308),
            getattr(ecodes, "BTN_X", 307),
        })

        # Botões de saída
        self.EXIT_BUTTONS: frozenset[int] = frozenset({ecodes.BTN_START, 315})

    @property
    def name(self) -> str:
        return self._device.name

    @property
    def path(self) -> str:
        return self._device.path

    def grab(self) -> None:
        """Grab exclusivo: impede que outros processos leiam o controle."""
        try:
            self._device.grab()
        except Exception:
            print("Aviso: não foi possível obter grab exclusivo do controle.")

    def ungrab(self) -> None:
        try:
            self._device.ungrab()
        except Exception:
            pass

    def events(self):
        """Gerador de eventos brutos do dispositivo."""
        return self._device.read_loop()

    def normalize_axis_y(self, raw: int) -> float:
        """Normaliza o eixo Y para [−1.0, 1.0]."""
        return (raw - self._center_y) / self._range_y

    def normalize_axis_x(self, raw: int) -> float:
        """Normaliza o eixo X para [−1.0, 1.0]."""
        return (raw - self._center_x) / self._range_x

    # ── Internos ──────────────────────────────────────────────────────────────

    @staticmethod
    def _find_gamepad(list_devices, InputDevice):
        from evdev import ecodes  # type: ignore
        devices = [InputDevice(p) for p in list_devices()]
        for dev in devices:
            cap = dev.capabilities()
            if ecodes.EV_KEY not in cap:
                continue
            keys = cap[ecodes.EV_KEY]
            name_lower = dev.name.lower()
            if (
                isinstance(keys, list)
                and any(k in keys for k in (ecodes.BTN_SOUTH, ecodes.BTN_A, ecodes.BTN_GAMEPAD))
            ) or any(kw in name_lower for kw in ("gamesir", "xbox", "gamepad")):
                return dev
        raise RuntimeError(
            "Nenhum gamepad encontrado. Verifique a conexão do controle."
        )

    def _axis_info(self, code: int) -> tuple[float, float]:
        """Retorna (center, range) para um eixo analógico."""
        from evdev import ecodes  # type: ignore
        abs_cap = self._device.capabilities().get(ecodes.EV_ABS, [])
        info = next((i for c, i in abs_cap if c == code), None)
        if info:
            center = (info.max + info.min) / 2.0
            rang   = (info.max - info.min) / 2.0
            return center, (rang if rang != 0 else 32767.0)
        return 0.0, 32767.0


# ── RobotController ───────────────────────────────────────────────────────────

class RobotController:
    """
    Máquina de estados do robô: sleep ↔ standing.

    Reage aos eventos do gamepad para:
      - Pressão curta do botão de ação → toggle stand/sleep.
      - Pressão longa (≥ 2 s) → toggle stand/sleep.
      - Analógico esquerdo Y → frente/trás.
      - Analógico esquerdo X → giro.
      - Botão START → encerra o modo.

    Uso::

        ctrl = RobotController(servo_mgr, legs)
        ctrl.run()
    """

    TOGGLE_COOLDOWN = 1.5  # s mínimos entre toggles consecutivos

    def __init__(
        self,
        servo_mgr: "ServoManager",
        legs: dict[str, "Leg"],
    ) -> None:
        self._mgr  = servo_mgr
        self._legs = legs
        self._gait = locomotion.GaitController(legs)

        self._is_standing    = False
        self._transitioning  = False
        self._last_toggle_ts = 0.0

        # Estado compartilhado entre threads (locomoção + estabilização)
        self.shared_state: dict = {
            "speed":            0,
            "direction":        1,
            "yaw":              0.0,
            "z_pitch_frente":   0.0,
            "z_pitch_tras":     0.0,
            "imu_roll_offset":  0.0,
            "imu_pitch_offset": 0.0,
        }

    # ── Ponto de entrada ──────────────────────────────────────────────────────

    def run(self) -> None:
        """Loop principal do modo Gamesir."""
        print("\nProcurando controle Gamesir...")
        try:
            reader = GamepadReader()
        except (ImportError, RuntimeError) as exc:
            print(f"Erro: {exc}")
            return

        ecodes = reader._ecodes
        print(f"Controle conectado: {reader.name} ({reader.path})")

        print("\n=== Modo Controle Gamesir ===")
        print(" -> Robô em modo SLEEP.")
        print(" -> Pressione [Botão Y] para LEVANTAR o robô.")
        print(" -> Use o Analógico Esquerdo (Cima) para andar para frente.")
        print(" -> Pressione [Botão Y] novamente para RETORNAR ao modo sleep.")
        print(" -> Pressione [START] ou Ctrl+C para SAIR.")

        poses.sleep(self._mgr)

        # Estado de botão de ação (pressão longa vs curta)
        action_pressed    = False
        action_press_time = 0.0

        reader.grab()
        try:
            for event in reader.events():

                # Ignora SYN
                if event.type == ecodes.EV_SYN:
                    continue

                # Checa pressão longa (≥ 2 s) do botão de ação
                if action_pressed and (time.time() - action_press_time >= 2.0):
                    action_pressed = False
                    self._toggle_stand_sleep()

                # ── Eventos de botão ──────────────────────────────────────────
                if event.type == ecodes.EV_KEY:
                    code = event.code
                    val  = event.value  # 1 = press, 0 = release, 2 = repeat

                    if code not in reader.ALLOWED_BUTTONS or val not in (0, 1):
                        continue

                    if code in reader.ACTION_BUTTONS:
                        if val == 1 and not action_pressed:
                            action_pressed    = True
                            action_press_time = time.time()
                            print("Segurando Botão de Ação... Aguarde 2 segundos.")
                        elif val == 0:
                            elapsed = time.time() - action_press_time
                            if action_pressed and elapsed < 2.0:
                                action_pressed = False
                                self._toggle_stand_sleep()
                            action_pressed = False

                    if val == 1 and code in reader.EXIT_BUTTONS:
                        print("\n[START Pressionado] Encerrando modo Gamesir...")
                        break

                # ── Eventos de eixo analógico ─────────────────────────────────
                elif event.type == ecodes.EV_ABS:
                    if not self._is_standing or self._transitioning:
                        continue
                    if event.code not in reader.ALLOWED_AXES:
                        continue

                    if event.code == ecodes.ABS_Y:
                        self._handle_axis_y(reader.normalize_axis_y(event.value))
                    elif event.code == ecodes.ABS_X:
                        self._handle_axis_x(reader.normalize_axis_x(event.value))

        except KeyboardInterrupt:
            print("\nInterrompido pelo usuário.")
        finally:
            self._gait.stop()
            reader.ungrab()
            print("Retornando robô ao modo sleep...")
            poses.sleep(self._mgr)
            print("Modo Gamesir finalizado.")

    # ── Handlers de eixo ─────────────────────────────────────────────────────

    def _handle_axis_y(self, axis_y: float) -> None:
        """Frente/trás via eixo Y do analógico esquerdo."""
        dz = GamepadReader.DEAD_ZONE
        current = self.shared_state.get("_walking_state", 0)

        if axis_y <= -dz:
            self.shared_state["speed"] = abs(axis_y)
            if current != 1:
                self.shared_state["direction"] = 1
                print(f"Andando para FRENTE... (Força: {abs(axis_y):.2f})")
                self.shared_state["_walking_state"] = 1
        elif axis_y >= dz:
            self.shared_state["speed"] = abs(axis_y)
            if current != -1:
                self.shared_state["direction"] = -1
                print(f"Andando para TRÁS... (Força: {abs(axis_y):.2f})")
                self.shared_state["_walking_state"] = -1
        else:
            self.shared_state["speed"] = 0
            if current != 0:
                print("Robô PARADO.")
                self.shared_state["_walking_state"] = 0

    def _handle_axis_x(self, axis_x: float) -> None:
        """Giro via eixo X do analógico esquerdo."""
        dz  = GamepadReader.DEAD_ZONE
        prev_yaw = self.shared_state["yaw"]
        self.shared_state["yaw"] = axis_x

        if abs(axis_x) > dz and abs(prev_yaw) <= dz:
            side = "DIREITA" if axis_x > 0 else "ESQUERDA"
            print(f"Girando para {side}... (Força: {abs(axis_x):.2f})")
        elif abs(axis_x) <= dz and abs(prev_yaw) > dz:
            print("Robô PARADO.")

    # ── Toggle stand/sleep ────────────────────────────────────────────────────

    def _toggle_stand_sleep(self) -> None:
        """Alterna entre posição de pé e sleep (com proteção contra double-click)."""
        now = time.time()
        if self._transitioning or (now - self._last_toggle_ts < self.TOGGLE_COOLDOWN):
            print("Aguarde a transição anterior finalizar.")
            return

        self._gait.stop()
        self.shared_state["speed"] = 0
        self.shared_state["yaw"]   = 0.0
        self._last_toggle_ts = now

        threading.Thread(target=self._run_toggle, daemon=True).start()

    def _run_toggle(self) -> None:
        self._transitioning = True
        try:
            if not self._is_standing:
                print("\n[Ação] Levantando robô...")
                poses.stand(self._mgr)
                self._gait.start(self.shared_state)
                self._is_standing = True
                print("Robô levantado e pronto para andar.")
            else:
                print("\n[Ação] Retornando para posição de descanso...")
                poses.sleep(self._mgr)
                self._is_standing = False
                print("Robô em repouso (sleep). Pressione Botão Y para levantar.")
        finally:
            self._transitioning = False
