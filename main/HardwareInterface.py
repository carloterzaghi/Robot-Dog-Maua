"""Direct hardware interface module for the Adafruit PCA9685 PWM driver.

Handles the transmission of PWM signals to the 12 servomotors, applies
mechanical calibration zero offsets, and computes smooth dynamic transitions
using a 5th-order Hermite polynomial (Smootherstep).
"""

from __future__ import annotations
import time
from typing import TYPE_CHECKING
import numpy as np
from numpy.typing import NDArray
from adafruit_servokit import ServoKit

from Config import Configuration, ServoParams

config = Configuration()


class HardwareInterface:
    """Low-level controller for PCA9685 I2C communication and PWM actuation."""

    def __init__(self) -> None:
        """Initializes the I2C connection to the PCA9685 and configures 16-channel PWM limits."""
        self.kit = ServoKit(channels=16)
        self.servo_params = ServoParams()
        self.current_angles: NDArray[np.float64] = np.zeros((3, 4))
        self.initialize_servo()

    def initialize_servo(self) -> None:
        """Configures 180° actuation range and pulse width limits (440 to 2560 µs) on all channels."""
        for i in range(16):
            self.kit.servo[i].actuation_range = 180
            self.kit.servo[i].set_pulse_width_range(440, 2560)

    def set_actuator_positions(
        self, 
        joint_angles: NDArray[np.float64], 
        use_servo_calibration: bool = True
    ) -> None:
        """Sends computed joint angles from a 3x4 matrix to the respective servos via I2C.

        Args:
            joint_angles: A (3, 4) matrix of target angles in degrees [°].
                - Rows: [0]=Abduction, [1]=Femur, [2]=Tibia.
                - Columns: [0]=FR, [1]=FL, [2]=RR, [3]=RL.
            use_servo_calibration: If True, subtracts mechanical assembly offsets
                defined in `config.physical_calibration_offsets`.
        """
        self.current_angles = np.copy(joint_angles)

        for row in range(3):          # Joint (0=Abduction, 1=Femur, 2=Tibia)
            for col in range(4):      # Leg (0=FR, 1=FL, 2=RR, 3=RL)
                canal = int(self.servo_params.pins[row, col])
                if use_servo_calibration:
                    angle_deg = joint_angles[row, col] - config.physical_calibration_offsets[row, col]
                else:
                    angle_deg = joint_angles[row, col]

                # Constrain the electrical command strictly within [0°, 180°]
                angle_deg = float(np.clip(angle_deg, 0.0, 180.0))
                self.kit.servo[canal].angle = angle_deg

    def smooth_transition(
        self, 
        target_angles: NDArray[np.float64], 
        n_steps: int = 25, 
        delay: float = 0.02
    ) -> None:
        """Executes a smooth transition from current posture to target posture.

        Uses the 5th-order Hermite polynomial (Smootherstep):
        s(t) = 6t^5 - 15t^4 + 10t^3, ensuring zero velocity and acceleration
        at the start and end points, preventing mechanical shock and servo chatter.

        Args:
            target_angles: A (3, 4) matrix of target angles in degrees [°].
            n_steps: Number of intermediate interpolation steps.
            delay: Wait time in seconds between successive steps.
        """
        start_angles = np.copy(self.current_angles)
        for step in range(1, n_steps + 1):
            t = step / n_steps
            # 5th-order Hermite polynomial (C² continuous)
            t_smooth = t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
            interpolated = start_angles + (target_angles - start_angles) * t_smooth
            self.set_actuator_positions(interpolated)
            time.sleep(delay)
