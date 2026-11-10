"""Asynchronous Gamepad Interface via Linux `evdev` Subsystem.

Provides non-blocking event polling for controller buttons and analog sticks,
axis normalization, deadband filtering, and mapping to robot velocity,
attitude, and behavioral command flags.
"""

from __future__ import annotations
import sys
from typing import TYPE_CHECKING, Any, Dict, Optional
import numpy as np
from evdev import InputDevice, ecodes, list_devices # type: ignore

from Command import Command
from Utilities import clipped_first_order_filter, deadband

if TYPE_CHECKING:
    from Config import Configuration
    from State import State

# Exact device identification string reported by GameSir T4 / compatible gamepad firmware
NOME_CONTROLE_EXATO = "Guangzhou Chicken Run Network Technology Co., Ltd. Wireless Controller"


def conectar_controle() -> InputDevice | None:
    """Searches `/dev/input/event*` devices for the controller with matching name.

    Returns:
        Connected `InputDevice` instance or None if not currently detected.
    """
    for path in list_devices():
        try:
            dev = InputDevice(path)
            if dev.name.strip() == NOME_CONTROLE_EXATO:
                print(f"\n[SUCCESS] Controller connected:")
                print(f"  Device : {dev.name}")
                print(f"  Event  : {dev.path}")
                return dev
        except Exception:
            continue

    print(f"\n[WARNING] Controller '{NOME_CONTROLE_EXATO}' not found.")
    print("Detected input devices on system:")
    for path in list_devices():
        try:
            print(f"  - {path} : {InputDevice(path).name}")
        except Exception:
            pass
    return None


class JoystickInterface:
    """Event polling manager and command dispatcher for the gamepad."""

    def __init__(self, config: Configuration) -> None:
        """Initializes the interface and discovers hardware axis bounds.

        Args:
            config: Configuration object containing velocity limits.

        Raises:
            RuntimeError: If the specified controller device is not detected.
        """
        self.config = config
        self.device: Optional[InputDevice] = conectar_controle()

        if self.device is None:
            raise RuntimeError("Controle não encontrado.")

        # Normalized joystick axes [-1.0 to +1.0]
        self.lx: float = 0.0
        self.ly: float = 0.0
        self.rx: float = 0.0
        self.ry: float = 0.0

        # Directional pad (D-Pad)
        self.dpad_x: int = 0
        self.dpad_y: int = 0

        # Discrete button event flags
        self.activate_event: bool = False
        self.trot_event: bool = False
        self.hop_event: bool = False
        self.A_button: bool = False
        self.X_button: bool = False
        self.L3_button: bool = False

        # Previous state tracking for rising-edge detection
        self.previous_activate: int = 0
        self.previous_trot: int = 0
        self.previous_hop: int = 0
        self.previous_sit: int = 0
        self.previous_X: int = 0
        self.previous_L3: int = 0

        # Discover native hardware calibration and limits
        self.axis_info: Dict[int, Any] = {}
        capabilities = self.device.capabilities(absinfo=True)
        if ecodes.EV_ABS in capabilities:
            for code, absinfo in capabilities[ecodes.EV_ABS]:
                self.axis_info[code] = absinfo

    def normalize_axis(self, code: int, value: int) -> float:
        """Normalizes a raw analog axis sensor value into [-1.0, +1.0].

        Args:
            code: evdev axis code (ABS_X, ABS_Y, etc.).
            value: Raw integer value returned by the kernel.

        Returns:
            Normalized float value clamped within [-1.0, 1.0].
        """
        if code not in self.axis_info:
            return 0.0

        info = self.axis_info[code]
        center = (info.max + info.min) / 2.0
        half_range = (info.max - info.min) / 2.0

        if half_range == 0:
            return 0.0

        normalized = (value - center) / half_range
        return float(np.clip(normalized, -1.0, 1.0))

    def read_events(self) -> None:
        """Reads all pending buffer events without blocking the main control loop."""
        self.activate_event = False
        self.trot_event = False
        self.hop_event = False
        self.A_button = False
        self.X_button = False
        self.L3_button = False

        try:
            for event in self.device.read():
                # Button key events (EV_KEY)
                if event.type == ecodes.EV_KEY:
                    # L1 (Activate/Deactivate - 310)
                    if event.code == 310:
                        if event.value == 1 and self.previous_activate == 0:
                            self.activate_event = True
                        self.previous_activate = event.value

                    # R1 (Trot Toggle - 311)
                    elif event.code == 311:
                        if event.value == 1 and self.previous_trot == 0:
                            self.trot_event = True
                        self.previous_trot = event.value

                    # A (Lay / Stand - 304)
                    elif event.code == 304:
                        if event.value == 1 and self.previous_sit == 0:
                            self.A_button = True
                        self.previous_sit = event.value

                    # X (90° Calibration - 308)
                    elif event.code == 308:
                        if event.value == 1 and self.previous_X == 0:
                            self.X_button = True
                        self.previous_X = event.value

                    # L3 (Left Stick Click - 317)
                    elif event.code == 317:
                        if event.value == 1 and self.previous_L3 == 0:
                            self.L3_button = True
                        self.previous_L3 = event.value

                # Analog stick and D-Pad events (EV_ABS)
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
                        self.dpad_y = event.value * -1

        except BlockingIOError:
            pass

    def get_command(self, state: State) -> Command:
        """Processes gamepad events and returns a validated target command vector.

        Applies an 8% deadband against potentiometer drift and computes rate-limited filters.

        Args:
            state: Current robot state.

        Returns:
            A `Command` instance containing target values for the next cycle.
        """
        self.read_events()
        command = Command(self.config)

        # Discrete events
        command.activate_event = self.activate_event
        command.trot_event = self.trot_event
        command.hop_event = self.hop_event
        command.A_button = self.A_button
        command.X_button = self.X_button
        command.L3_button = self.L3_button

        # Translational velocities with 8% deadband against stick drift
        dead_zone = 0.08
        x_vel = -deadband(self.ly, dead_zone) * self.config.max_x_velocity
        y_vel = -deadband(self.lx, dead_zone) * self.config.max_y_velocity
        command.horizontal_velocity = np.array([x_vel, y_vel])

        # Yaw angular rate
        command.yaw_rate = -deadband(self.rx, dead_zone) * self.config.max_yaw_rate

        # Pitch tilt smoothed via first-order filter
        pitch = self.ry * self.config.max_pitch
        pitch = deadband(pitch, self.config.pitch_deadband)
        pitch_rate = clipped_first_order_filter(
            state.pitch,
            pitch,
            self.config.max_pitch_rate,
            self.config.pitch_time_constant,
        )
        command.pitch = state.pitch + 0.05 * pitch_rate

        # Height fine adjustment via vertical D-Pad
        command.height = float(np.clip(
            state.height - 0.05 * self.config.z_speed * self.dpad_y,
            -0.23, -0.08
        ))

        # Roll fine adjustment via horizontal D-Pad
        command.roll = float(np.clip(
            state.roll - 0.05 * self.config.roll_speed * self.dpad_x,
            -0.20, 0.20
        ))

        return command
