# 🤝 Contribution Guidelines — Robot-Dog-Maua

Thank you for your interest in contributing to **Robot-Dog-Maua** or **MauAU**! This document outlines our engineering standards, coding conventions, testing procedures, and safety protocols to ensure the codebase remains robust, maintainable, and safe for all contributors.

---

## 1. Python Code Standards

### Formatting & Style (PEP 8)
- All Python code must adhere to [PEP 8](https://peps.python.org/pep-0008/) style guidelines.
- Indentation: **4 spaces** (never tabs).
- Maximum line length recommendation: **88 to 100 characters**.
- Naming conventions: `CamelCase` for classes, `snake_case` for methods and functions, and `UPPER_SNAKE_CASE` for constants.

### Static Type Annotations (*Type Hints*)
All public classes, methods, and functions must include type annotations for both parameters and return values:
```python
from typing import Tuple
import numpy as np
from numpy.typing import NDArray

def calculate_foot_position(
    leg_index: int, 
    joint_angles: NDArray[np.float64]
) -> NDArray[np.float64]:
    ...
```

### Docstring Standard (Google Style)
Always document the physical purpose of the function, parameter types with physical units, matrix shapes, and return values:
```python
def set_actuator_positions(
    self, 
    joint_angles: NDArray[np.float64], 
    use_servo_calibration: bool = True
) -> None:
    """Sends computed joint angles to servomotors via PCA9685.

    Args:
        joint_angles: A (3, 4) NumPy array of target angles in degrees [°].
            - Rows: [0]=Abduction, [1]=Femur, [2]=Tibia.
            - Columns: [0]=FR, [1]=FL, [2]=RR, [3]=RL.
        use_servo_calibration: If True, applies mechanical zero calibration offsets.

    Raises:
        ValueError: If the input matrix does not match shape (3, 4).
    """
```

---

## 2. Core Non-Negotiable Matrix Convention ($3 \times 4$)

Every mathematical, kinematic, and hardware module must adhere strictly to the project's $3 \times 4$ matrix layout:
- **Rows (Joints):** `0 = Abduction/Hip Roll`, `1 = Femur/Upper Leg`, `2 = Tibia/Lower Leg`.
- **Columns (Leg Index):** `0 = FR` (Front Right), `1 = FL` (Front Left), `2 = RR` (Rear Right), `3 = RL` (Rear Left).

Standard physical units (SI):
- **Lengths & Coordinates:** Meters ($m$) or Millimeters ($mm$) explicitly declared.
- **Internal Kinematic Angles:** Radians ($rad$).
- **Servo Output Angles:** Degrees ($^\circ$).
- **Translational Velocities:** Meters per second ($m/s$).
- **Rotational Velocities:** Radians per second ($rad/s$).

---

## 3. Bench Testing & Safety Verification Protocol

Servomotors under dynamic loads can strip gear teeth or burn out if commanded past physical joint limits:

1. **Static Compilation Check:**
   Before committing, verify that all Python files compile cleanly without syntax errors:
   ```bash
   python -m py_compile main/*.py
   ```
2. **Elevated Stand Verification:**
   Never place the robot on the ground immediately after modifying kinematics (`Kinematics.py`), offsets (`Config.py`), or gait controllers (`Controller.py`, `Gaits.py`).
   - Place the robot chassis on an elevated stand keeping **all 4 legs suspended in the air**.
   - Power logic first, connect the gamepad, and only then enable motor power.
   - Test static poses (`X` for 90° neutral calibration, `A` for laying down/standing up) before triggering dynamic trotting (`R1`).