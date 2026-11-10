"""Main Control Loop — Robot-Dog-Maua.

Manages robot lifecycle, behavioral state machine transitions (DEACTIVATED,
REST, TROT), non-blocking gamepad polling at 100 Hz, and joint angle dispatch
to 12 servomotors via PCA9685.
"""

from __future__ import annotations
import math
import time
from typing import Optional
import numpy as np
from numpy.typing import NDArray

from Config import Configuration
from Controller import Controller
from HardwareInterface import HardwareInterface
from IMU import IMU
from JoystickInterface import JoystickInterface
from Kinematics import four_legs_inverse_kinematics
from State import BehaviorState, State

# ---------------------------------------------------------------------------
# Initialization of Parameters and Hardware Interfaces
# ---------------------------------------------------------------------------
use_imu: bool = False
imu: Optional[IMU] = IMU() if use_imu else None

config = Configuration()
state = State()
state.height = config.default_z_ref
hardware_interface = HardwareInterface()

joystick = JoystickInterface(config)
last_loop = time.time()

# Locomotion controller and kinematics planner
controller = Controller(
    config,
    four_legs_inverse_kinematics,
)

SMOOTH_N_STEPS: int = 60       # Smootherstep interpolation steps
SMOOTH_DELAY: float = 0.02     # Time delay between steps in seconds (total = 1.2s)

# Position robot in initial folded rest posture
hardware_interface.set_actuator_positions(config.lay_angles)


def calculate_default_angles() -> NDArray[np.float64]:
    """Calculates default joint angles for the nominal standing posture.

    Returns:
        A (3, 4) NumPy array of joint angles in degrees.
    """
    state.foot_locations = (
        config.default_stance
        + np.array([0, 0, config.default_z_ref])[:, np.newaxis]
    )
    state.joint_angles = four_legs_inverse_kinematics(
        state.foot_locations, config
    )
    return state.joint_angles


# Backward-compatible alias
calcualte_default_angles = calculate_default_angles

if use_imu and imu is not None:
    imu.calibrate(samples=300)

try:
    while True:
        # -------------------------------------------------------------------
        # Idle / Rest State (DEACTIVATED)
        # -------------------------------------------------------------------
        print("Waiting for L1 to activate robot. (Press A to lay/stand, X to calibrate)")
        while True:
            command = joystick.get_command(state)

            # Button X: Toggle mechanical 90° neutral calibration posture
            if command.X_button:
                if not state.is_calibration:
                    print("Calibration Mode (90° neutral pose active)")
                    hardware_interface.smooth_transition(
                        config.neutral_angle, n_steps=SMOOTH_N_STEPS, delay=SMOOTH_DELAY
                    )
                    state.is_calibration = True
                    state.is_laying = False
                else:
                    print("Returning from calibration to standing posture")
                    hardware_interface.smooth_transition(
                        calculate_default_angles(), n_steps=SMOOTH_N_STEPS, delay=SMOOTH_DELAY
                    )
                    state.is_calibration = False

            # Button A: Smooth transition between laying down and standing up
            if command.A_button:
                if not state.is_laying:
                    print("Laying down smoothly...")
                    hardware_interface.smooth_transition(
                        config.lay_angles, n_steps=SMOOTH_N_STEPS, delay=SMOOTH_DELAY
                    )
                    state.is_laying = True
                    state.is_calibration = False
                else:
                    print("Standing up smoothly...")
                    hardware_interface.smooth_transition(
                        calculate_default_angles(), n_steps=SMOOTH_N_STEPS, delay=SMOOTH_DELAY
                    )
                    state.is_laying = False

            # Button L1: Activate robot and advance to active control loop
            if command.activate_event == 1:
                if state.is_laying or state.is_calibration:
                    print("Robot was folded/calibrating. Standing up before activation...")
                    hardware_interface.smooth_transition(
                        calculate_default_angles(), n_steps=SMOOTH_N_STEPS, delay=SMOOTH_DELAY
                    )
                    state.is_laying = False
                    state.is_calibration = False

                state.behavior_state = BehaviorState.REST
                break
            time.sleep(0.1)

        print("Robot activated in REST mode. Press R1 to toggle Trot locomotion.")
        last_loop = time.time()

        # -------------------------------------------------------------------
        # Deterministic Real-Time Loop (100 Hz, dt = 0.01 s)
        # -------------------------------------------------------------------
        while True:
            now = time.time()
            if now - last_loop < config.dt:
                continue
            last_loop = time.time()

            # 1. Non-blocking polling of gamepad commands
            command = joystick.get_command(state)

            # 2. Read estimated orientation from MPU-6050 IMU
            quat_orientation = (
                imu.read_orientation()
                if (use_imu and imu is not None)
                else np.array([1, 0, 0, 0])
            )
            state.quat_orientation = quat_orientation

            # 3. Advance state machine and gait controller
            controller.run(state, command)

            # Button L1: Safety deactivation (Smooth E-Stop to current stance)
            if command.activate_event == 1:
                print("Deactivating robot (Returning to standby mode)...")
                hardware_interface.smooth_transition(state.joint_angles, n_steps=30, delay=0.04)
                break

            # 4. Dispatch computed angles to servomotors via PCA9685
            hardware_interface.set_actuator_positions(state.joint_angles)

except KeyboardInterrupt:
    print("\nExecution terminated by user (Ctrl + C).")

finally:
    if imu is not None:
        imu.close()
        print("IMU connection closed successfully.")
