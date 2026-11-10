# 🎮 Gamepad Configuration & Teleoperation (`evdev`)

> **Relevant Modules:** [`main/JoystickInterface.py`](../main/JoystickInterface.py), [`Testes Interface/mapear_controle.py`](../Testes%20Interface/mapear_controle.py)

---

## 1. How Gamepad Event Reading Works on Linux

On embedded Linux distributions (such as Raspberry Pi OS), gamepads connected via USB cables, 2.4 GHz wireless dongles, or Bluetooth are exposed by the Linux kernel as event character devices under `/dev/input/event*`.

The robot utilizes the Python **`evdev`** library, which interfaces with kernel event streams directly with minimal CPU overhead, avoiding heavy graphical dependencies like Pygame or SDL.

---

## 2. Bluetooth Pairing on Raspberry Pi

To pair a new Bluetooth gamepad via the terminal:

```bash
# Launch the BlueZ interactive utility
bluetoothctl

# Inside the bluetoothctl prompt:
agent on
default-agent
scan on

# Put the controller into pairing mode (fast-blinking LEDs)
# Locate the controller's MAC address (e.g., E4:17:D8:XX:XX:XX)
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
quit
```

Once paired and trusted (`trust`), the gamepad reconnects automatically whenever powered on within range of the Raspberry Pi.

---

## 3. Controller Device Identification

By default, [`main/JoystickInterface.py`](../main/JoystickInterface.py) matches the exact identification string advertised by the gamepad's firmware:

```python
NOME_CONTROLE_EXATO = "Guangzhou Chicken Run Network Technology Co., Ltd. Wireless Controller"
```

### Discovering Your Gamepad's Device Name:
If you are using a different gamepad (such as Xbox Wireless, PlayStation DualShock/DualSense, 8BitDo, or generic PC controllers), run the discovery tool:

```bash
sudo env/bin/python3 "Testes Interface/mapear_controle.py"
```

The script will list all detected input devices:
```text
Dispositivos detectados no sistema:
  - /dev/input/event0 : vc4-hdmi
  - /dev/input/event1 : Xbox Wireless Controller
  - /dev/input/event2 : Guangzhou Chicken Run Network Technology...
```

Copy the exact device string printed on the screen and update `NOME_CONTROLE_EXATO` in [`main/JoystickInterface.py`](../main/JoystickInterface.py).

---

## 4. Complete Button & Axis Layout

```
                    [ L1 / LB ]                   [ R1 / RB ]
                     (Activate)                   (Trot Gait)
                        │                             │
                   ┌────┴─────────────────────────────┴────┐
                   │                                       │
     [ D-PAD ]     │     (LX, LY)            (RX, RY)      │    [ A ] Lay / Stand
   Height / Roll   │  Translation Vx, Vy    Yaw / Pitch    │    [ X ] 90° Calibration
                   │                                       │
                   └───────────────────────────────────────┘
```

| Controller Input | `evdev` Event Code | Function on Robot | System Response |
|---|:---:|---|---|
| **L1 / LB Button** | `BTN_TL (310)` | Activate / Deactivate | Toggles between `DEACTIVATED` and `REST` (Smooth E-Stop) |
| **R1 / RB Button** | `BTN_TR (311)` | Trot Mode Toggle | Toggles between `REST` (standing) and `TROT` (locomotion) |
| **A Button** | `BTN_SOUTH (304)` | Lay / Stand Toggle | Smooth transition between standing and folded `lay_angles` |
| **X Button** | `BTN_WEST (308)` | Calibration Mode | Commands all 12 servomotors to the 90° neutral pose |
| **Left Stick Y ($LY$)** | `ABS_Y (1)` | Forward / Backward | Forward velocity $V_x$ (up to $\pm 0.4\text{ m/s}$) |
| **Left Stick X ($LX$)** | `ABS_X (0)` | Lateral Strafe | Lateral velocity $V_y$ (up to $\pm 0.3\text{ m/s}$) |
| **Right Stick X ($RX$)** | `ABS_RX (3)` | Heading Turn | Yaw rate $\omega_z$ (up to $\pm 2.0\text{ rad/s}$) |
| **Right Stick Y ($RY$)** | `ABS_RY (4)` | Longitudinal Tilt | Pitch angle (nose up/down up to $\pm 30^\circ$) |
| **D-Pad Vertical ($DPad_Y$)** | `ABS_HAT0Y (17)` | Height Adjustment | Body clearance height $Z$ between $-0.23\text{ m}$ and $-0.08\text{ m}$ |
| **D-Pad Horizontal ($DPad_X$)** | `ABS_HAT0X (16)` | Roll Adjustment | Lateral chassis roll tilt between $-0.20\text{ rad}$ and $+0.20\text{ rad}$ |

---

## 5. Stability Filters: Analog Deadband & Slew Rate

To prevent potentiometer drift or mechanical gimbal play from inducing unintended motion, the interface applies an **8% deadband filter** (`dead_zone = 0.08`):

```python
# In JoystickInterface.py:
dead_zone = 0.08
x_vel = -deadband(self.ly, dead_zone) * self.config.max_x_velocity
y_vel = -deadband(self.lx, dead_zone) * self.config.max_y_velocity
command.yaw_rate = -deadband(self.rx, dead_zone) * self.config.max_yaw_rate
```

Additionally, commanded pitch inclinations pass through a first-order slew rate filter (`clipped_first_order_filter`) in [`main/Utilities.py`](../main/Utilities.py), dampening sudden angular accelerations that could upset static equilibrium.
