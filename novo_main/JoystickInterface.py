from __future__ import annotations

import sys
import numpy as np
from evdev import InputDevice, list_devices, ecodes

from Command import Command
from Utilities import deadband, clipped_first_order_filter

NOME_CONTROLE_EXATO = "Guangzhou Chicken Run Network Technology Co., Ltd. Wireless Controller"


def conectar_controle() -> InputDevice | None:
    """Conecta diretamente e exclusivamente ao controle com o nome exato."""
    for path in list_devices():
        try:
            dev = InputDevice(path)
            if dev.name.strip() == NOME_CONTROLE_EXATO:
                print(f"\n[SUCESSO] Controle conectado:")
                print(f"  Dispositivo : {dev.name}")
                print(f"  Evento      : {dev.path}")
                return dev
        except Exception:
            continue

    print(f"\n[ERRO] Controle '{NOME_CONTROLE_EXATO}' não encontrado.")
    print("Dispositivos detectados no sistema:")
    for path in list_devices():
        try:
            print(f"  - {path} : {InputDevice(path).name}")
        except Exception:
            pass
    return None


class JoystickInterface:

    def __init__(self, config):
        self.config = config
        self.device = conectar_controle()

        if self.device is None:
            raise RuntimeError("Controle não encontrado.")

        # -------------------------------
        # Estado atual dos analógicos
        # -------------------------------
        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.ry = 0.0

        self.dpad_x = 0
        self.dpad_y = 0

        # -------------------------------
        # Eventos
        # -------------------------------
        self.activate_event = False
        self.trot_event = False
        self.hop_event = False

        # Valores anteriores
        self.previous_activate = 0
        self.previous_trot = 0
        self.previous_hop = 0

        # Descobre ranges reais dos analógicos
        self.axis_info = {}
        capabilities = self.device.capabilities(absinfo=True)
        if ecodes.EV_ABS in capabilities:
            for code, absinfo in capabilities[ecodes.EV_ABS]:
                self.axis_info[code] = absinfo

    def normalize_axis(self, code, value):
        """Converte o valor bruto do controle para aproximadamente -1 até +1."""
        if code not in self.axis_info:
            return 0.0

        info = self.axis_info[code]
        center = (info.max + info.min) / 2.0
        half_range = (info.max - info.min) / 2.0

        if half_range == 0:
            return 0.0

        normalized = (value - center) / half_range
        return float(np.clip(normalized, -1.0, 1.0))

    def read_events(self):
        """Lê todos os eventos disponíveis sem travar o programa."""
        self.activate_event = False
        self.trot_event = False
        self.hop_event = False

        try:
            for event in self.device.read():
                # =============================
                # BOTÕES
                # =============================
                if event.type == ecodes.EV_KEY:
                    # L1 (Ativar)
                    if event.code == 310:
                        if event.value == 1 and self.previous_activate == 0:
                            self.activate_event = True
                        self.previous_activate = event.value

                    # R1 (Trote)
                    elif event.code == 311:
                        if event.value == 1 and self.previous_trot == 0:
                            self.trot_event = True
                        self.previous_trot = event.value

                    # A (Pulo)
                    elif event.code == 304:
                        if event.value == 1 and self.previous_hop == 0:
                            self.hop_event = True
                        self.previous_hop = event.value

                # =============================
                # ANALÓGICOS E D-PAD
                # =============================
                elif event.type == ecodes.EV_ABS:
                    if event.code == 0:
                        self.lx = self.normalize_axis(0, event.value)
                    elif event.code == 1:
                        self.ly = self.normalize_axis(1, event.value)
                    elif event.code == 3:
                        self.rx = self.normalize_axis(3, event.value)
                    elif event.code == 4:
                        self.ry = self.normalize_axis(4, event.value)
                    elif event.code == 16:
                        self.dpad_x = event.value
                    elif event.code == 17:
                        self.dpad_y = event.value*-1

        except BlockingIOError:
            pass

    def get_command(self, state):
        self.read_events()

        command = Command(self.config)

        # =============================================
        # EVENTOS
        # =============================================
        command.activate_event = self.activate_event
        command.trot_event = self.trot_event
        command.hop_event = self.hop_event

        # =============================================
        # VELOCIDADE (com deadband de 8% contra drift)
        # =============================================
        dead_zone = 0.08
        x_vel = -deadband(self.ly, dead_zone) * self.config.max_x_velocity
        y_vel = -deadband(self.lx, dead_zone) * self.config.max_y_velocity
        command.horizontal_velocity = np.array([x_vel, y_vel])

        # =============================================
        # YAW (com deadband)
        # =============================================
        command.yaw_rate = -deadband(self.rx, dead_zone) * self.config.max_yaw_rate

        # =============================================
        # PITCH
        # =============================================
        pitch = -self.ry * self.config.max_pitch
        pitch = deadband(pitch, self.config.pitch_deadband)
        pitch_rate = clipped_first_order_filter(
            state.pitch,
            pitch,
            self.config.max_pitch_rate,
            self.config.pitch_time_constant,
        )
        command.pitch = state.pitch + 0.05 * pitch_rate

        # =============================================
        # ALTURA - DPAD VERTICAL
        # =============================================
        command.height = (
            state.height
            - 0.05 * self.config.z_speed * self.dpad_y
        )

        # =============================================
        # ROLL - DPAD HORIZONTAL MUDAR AQUI self.config.dt 0.05
        # =============================================
        command.roll = (
            state.roll
            - 0.05 * self.config.roll_speed * self.dpad_x
        )

        return command
