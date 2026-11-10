# ⚡ Hardware, Pinout & Electrical Wiring

> **Core Components:** Raspberry Pi 4 Model B, Adafruit PCA9685 (16-channel 12-bit PWM driver), MPU6050 (GY-521 IMU), 12x Servomotors.

---

## 1. Power Distribution & Electrical Safety

Proper power delivery is the single most critical factor in quadruped robot stability. High-torque standard servomotors (such as DS3218, MG996R, or 20–35 kg·cm equivalents) typically draw:
- **Idle current:** $\approx 100\text{ mA}$ to $200\text{ mA}$ per servo.
- **Dynamic motion current:** $1.0\text{A}$ to $1.5\text{A}$ per servo.
- **Stall current:** up to $2.5\text{A}$ to $3.0\text{A}$ per servo.

With 12 servomotors operating simultaneously during dynamic trotting, aggregate current demand can spike to **$6\text{A}$ to $10\text{A}$ peak**.

```
                ┌─────────────────────────────────────────┐
                │ Dedicated Logic Supply (5V / 3A USB-C)  │
                └───────────────────┬─────────────────────┘
                                    │
                                    ▼
                          ┌───────────────────┐
                          │  Raspberry Pi 4   │
                          └─────────┬─────────┘
                                    │ I2C (3.3V, SDA, SCL, GND)
                                    ▼
┌────────────────────────┐       ┌─────────────────────┐
│ External Servo Supply  │──────►│  PCA9685 (Driver)   │
│ 5V - 6V (8A - 10A)     │ Green │   [COMMON GROUND!]  │
└────────────────────────┘ Term. └──────────┬──────────┘
                                            │
                                 12x PWM Cables (Signal, V+, GND)
                                            │
                                            ▼
                                     12x Servomotors
```

### Golden Rules of Electrical Setup:
1. **Never power servos from the Raspberry Pi 5V header:** The board's copper traces and internal regulators support at most $\approx 1.5\text{A}$ total. Powering servos directly from the Pi will cause brownout resets or permanent board damage.
2. **Common Ground is Mandatory:** The Raspberry Pi GND and the negative terminal of the external motor supply must both be firmly tied to the PCA9685 ground plane to ensure clean, referenced PWM logic signals.
3. **Decoupling Capacitors:** Installing a large low-ESR electrolytic capacitor ($1000\,\mu\text{F}$ to $4700\,\mu\text{F}$, rated for $10\text{V}$ or higher) across the PCA9685 green motor power terminal is strongly recommended to buffer inductive voltage spikes.

---

## 2. I2C Bus Pinout

The Raspberry Pi 4 communicates with both the PCA9685 PWM driver and the MPU6050 IMU over the primary hardware I2C bus (`/dev/i2c-1`):

| Raspberry Pi 4 Pin | Pin Name | PCA9685 Pin | MPU6050 Pin | Functional Role |
|:---:|:---:|:---:|:---:|---|
| **Pin 1** | `3V3 Power` | `VCC` | `VCC` | Logic power supply (3.3V) |
| **Pin 3** | `GPIO 2 (SDA)` | `SDA` | `SDA` | I2C bidirectional data line |
| **Pin 5** | `GPIO 3 (SCL)` | `SCL` | `SCL` | I2C clock line |
| **Pin 6 / 9 / 14** | `GND` | `GND` | `GND` | Shared zero-volt ground reference |

> ⚠️ **Note:** The `VCC` pin on the PCA9685 header powers only the internal logic IC and must receive **3.3V**. The green screw terminal block (`V+`) powers the motors and must receive the external **5V to 6V** high-current supply.

---

## 3. PCA9685 16-Channel Allocation Table

Servomotors must be connected to the standard 3-pin headers (Yellow/Signal, Red/V+, Brown or Black/GND) according to the mapping below, synchronized with [`main/Config.py`](../main/Config.py):

| PCA9685 Channel | Leg | Joint | Array Variable in `Config.py` |
|:---:|---|---|---|
| **0** | **FR** (Front Right) | **Tibia** (Lower) | `pins[2, 0] = 0` |
| **1** | **FR** (Front Right) | **Abduction** (Hip Roll) | `pins[0, 0] = 1` |
| **2** | **FR** (Front Right) | **Femur** (Upper) | `pins[1, 0] = 2` |
| **3** | *Unused / Spare* | — | — |
| **4** | *Unused / Spare* | — | — |
| **5** | **FL** (Front Left) | **Tibia** (Lower) | `pins[2, 1] = 5` |
| **6** | **FL** (Front Left) | **Abduction** (Hip Roll) | `pins[0, 1] = 6` |
| **7** | **FL** (Front Left) | **Femur** (Upper) | `pins[1, 1] = 7` |
| **8** | **RL** (Rear Left) | **Femur** (Upper) | `pins[1, 3] = 8` |
| **9** | **RL** (Rear Left) | **Abduction** (Hip Roll) | `pins[0, 3] = 9` |
| **10** | **RL** (Rear Left) | **Tibia** (Lower) | `pins[2, 3] = 10` |
| **11** | *Unused / Spare* | — | — |
| **12** | *Unused / Spare* | — | — |
| **13** | **RR** (Rear Right) | **Femur** (Upper) | `pins[1, 2] = 13` |
| **14** | **RR** (Rear Right) | **Abduction** (Hip Roll) | `pins[0, 2] = 14` |
| **15** | **RR** (Rear Right) | **Tibia** (Lower) | `pins[2, 2] = 15` |

### PWM Signal Timing (`HardwareInterface.py`)
- **Update Frequency:** $50\text{ Hz}$ ($T = 20\text{ ms}$).
- **Minimum Pulse Width ($0^\circ$):** $440\,\mu\text{s}$.
- **Maximum Pulse Width ($180^\circ$):** $2560\,\mu\text{s}$.
- **Angular Actuation Range:** $180^\circ$.

---

## 4. Hardware Verification & Diagnostics

Before running the main control loop, execute low-level diagnostics:

### 1. I2C Bus Address Scan
On the Raspberry Pi terminal, verify that both I2C peripherals are detected:
```bash
sudo i2cdetect -y 1
```
Expected output:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --  <-- PCA9685 (0x40)
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --  <-- MPU6050 (0x68)
70: 70 -- -- -- -- -- -- --                         <-- All-call PCA9685
```

### 2. Isolated Single-Servo Movement Test
To verify individual servo rotation safely without the kinematic loop, run the utility script in `Testes Interface/`:
```bash
sudo env/bin/python3 "Testes Interface/controle_servos.py"
```
This utility prompts for a specific channel (0–15) and target angle, allowing you to confirm motor health and wiring while the robot is suspended.
