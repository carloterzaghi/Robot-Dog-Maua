"""Global Configuration, Chassis Geometry, Gait Parameters, and Servo Mapping.

Serves as the single source of truth for physical chassis and link dimensions,
PCA9685 PWM channel assignments, gait timing parameters, velocity limits,
fundamental postures, and mechanical servo zero calibration offsets.
"""

from __future__ import annotations
import math as m
import numpy as np
from numpy.typing import NDArray


class ServoParams:
    """Hardware mapping of 12 servomotors to physical PCA9685 PWM channels."""

    def __init__(self) -> None:
        """Defines the pin array (3 joints x 4 legs).

        Matrix layout convention:
        - Rows: [0]=Abduction/Hip Roll, [1]=Femur, [2]=Tibia
        - Columns: [0]=FR, [1]=FL, [2]=RR, [3]=RL
        """
        self.pins: NDArray[np.int_] = np.array([
            [1, 6, 14, 9],   # Abduction (FR=1, FL=6, RR=14, RL=9)
            [2, 7, 13, 8],   # Femur     (FR=2, FL=7, RR=13, RL=8)
            [0, 5, 15, 10]   # Tibia     (FR=0, FL=5, RR=15, RL=10)
        ])


class Configuration:
    """Geometric, kinematic, and dynamic configuration parameters of the quadruped."""

    def __init__(self) -> None:
        # -------------------------------------------------------------------
        # 1. Angular Calibration and Fundamental Postures
        # -------------------------------------------------------------------
        # Neutral reference pose for mechanical zero alignment (all servos at 90°)
        self.neutral_angle: NDArray[np.float64] = np.array([
            [90.0, 90.0, 90.0, 90.0],
            [90.0, 90.0, 90.0, 90.0],
            [90.0, 90.0, 90.0, 90.0]
        ])

        # Servo rotation direction masks due to physical left/right symmetry mirroring
        self.servo_orientation_adjusts: NDArray[np.int_] = np.array([
            [0, 180, 180,   0],
            [0, 180,   0, 180],
            [0, 180,   0, 180]
        ])

        # Measured bench mechanical mounting offsets (degrees, unoriented)
        self.physical_calibration_offsets_unoriented: NDArray[np.float64] = np.array([
            [-5.50,  2.00,  6.00,  4.00],   # Abduction (FR, FL, RR, RL)
            [ 1.00,  1.50, -1.00, -1.00],   # Femur
            [ 0.83,  2.83,  4.83,  0.00]    # Tibia
        ])

        # Direction vector (+1 or -1) derived from orientation mask
        direction = np.where(self.servo_orientation_adjusts == 0, 1, -1)
        self.physical_calibration_offsets_oriented: NDArray[np.float64] = (
            self.physical_calibration_offsets_unoriented * direction
        )
        self.physical_calibration_offsets: NDArray[np.float64] = self.physical_calibration_offsets_oriented

        # Folded rest pose (laying down / lay)
        self.lay_angles: NDArray[np.float64] = abs(self.servo_orientation_adjusts - np.array([
            [90.0, 90.0, 90.0, 90.0],   # Abduction
            [41.0, 41.0, 41.0, 41.0],   # Femur
            [58.0, 58.0, 58.0, 58.0]    # Tibia
        ]))

        # -------------------------------------------------------------------
        # 2. Teleoperation Velocity Bounds
        # -------------------------------------------------------------------
        self.max_x_velocity: float = 0.4                  # Max forward/backward linear speed [m/s]
        self.max_y_velocity: float = 0.3                  # Max lateral strafe speed [m/s]
        self.max_yaw_rate: float = 2.0                    # Max yaw turning rate [rad/s]
        self.max_pitch: float = 30.0 * np.pi / 180.0       # Max pitch inclination [rad]

        # -------------------------------------------------------------------
        # 3. Dynamic Time Constants and Movement Parameters
        # -------------------------------------------------------------------
        self.z_time_constant: float = 0.02
        self.z_speed: float = 0.06                        # Max height adjustment speed [m/s]
        self.pitch_deadband: float = 0.02
        self.pitch_time_constant: float = 0.25
        self.max_pitch_rate: float = 0.15
        self.roll_speed: float = 0.16                     # Max roll adjustment rate [rad/s]
        self.yaw_time_constant: float = 0.3
        self.max_stance_yaw: float = 1.2
        self.max_stance_yaw_rate: float = 2.0

        self.default_yaw_rate: float = 0.0
        self.height: float = -0.20                        # Nominal standing body height relative to feet [m]
        self.default_pitch: float = 0.0
        self.default_roll: float = 0.0

        # -------------------------------------------------------------------
        # 4. Ground Stance Parameters
        # -------------------------------------------------------------------
        self.delta_x: float = 0.126                       # Nominal foot X position relative to center [m]
        self.delta_y: float = 0.137                       # Nominal foot Y position relative to center [m]
        self.x_shift: float = 0.0
        self.default_z_ref: float = self.height           # Nominal standing Z reference [m]

        # -------------------------------------------------------------------
        # 5. Swing Trajectory Parameters
        # -------------------------------------------------------------------
        self.z_coeffs = None
        self.__z_clearance: float = 0.12                  # Maximum vertical ground clearance during swing [m]
        self.alpha: float = 0.5                           # Raibert translational heuristic ratio
        self.beta: float = 0.5                            # Raibert rotational heuristic ratio

        # -------------------------------------------------------------------
        # 6. Gait Timing and Phases (Trot Gait)
        # -------------------------------------------------------------------
        self.dt: float = 0.01                             # Synchronous loop time step (100 Hz = 0.01s)
        self.num_phases: int = 4
        self.contact_phases: NDArray[np.int_] = np.array([
            [1, 1, 1, 0],
            [1, 0, 1, 1],
            [1, 0, 1, 1],
            [1, 1, 1, 0]
        ])
        self.overlap_time: float = 0.10                   # 4-foot ground support duration [s]
        self.swing_time: float = 0.15                     # Single diagonal swing flight duration [s]

        # -------------------------------------------------------------------
        # 7. Structural Chassis Dimensions & Link Lengths (meters)
        # -------------------------------------------------------------------
        self.LEG_FRONT_X: float = 0.1145                  # Front-back distance from center to front leg axis [m]
        self.LEG_REAR_X: float = 0.1375                   # Front-back distance from center to rear leg axis [m]
        self.LEG_LR: float = 0.097                        # Lateral distance from center to abduction plane [m]
        self.LEG_L1: float = 0.130                        # Femur (Upper Leg) joint length [m]
        self.LEG_L2: float = 0.120                        # Tibia (Lower Leg) joint length [m]
        self.ABDUCTION_OFFSET: float = 0.04               # Abduction lateral offset [m]
        self.FOOT_RADIUS: float = 0.014                   # Spherical foot tip radius [m]

        # Pantographic auxiliary linkage lengths (m)
        self.a: float = 0.035
        self.b: float = 0.03253
        self.c: float = 0.035
        self.d: float = 0.03253
        self.e: float = 0.0495
        self.f: float = self.LEG_L2
        self.g: float = 0.035
        self.h: float = 0.035
        self.i: float = self.LEG_L2

        self.gamma: float = m.atan(23 / 23)
        self.EDC: float = m.acos((self.c ** 2 + self.h ** 2 - self.e ** 2) / (2 * self.c * self.h))
        self.phi: float = m.pi / 2

        # Leg base origin coordinates in body frame
        self.LEG_ORIGINS: NDArray[np.float64] = np.array([
            [self.LEG_FRONT_X, self.LEG_FRONT_X, -self.LEG_REAR_X, -self.LEG_REAR_X],
            [-self.LEG_LR,      self.LEG_LR,      -self.LEG_LR,      self.LEG_LR     ],
            [0.0,              0.0,               0.0,               0.0             ],
        ])

        self.ABDUCTION_OFFSETS: NDArray[np.float64] = np.array([
            -self.ABDUCTION_OFFSET,
             self.ABDUCTION_OFFSET,
            -self.ABDUCTION_OFFSET,
             self.ABDUCTION_OFFSET,
        ])

    @property
    def default_stance(self) -> NDArray[np.float64]:
        """A (3, 4) matrix of nominal foot locations in the body coordinate frame."""
        return np.array([
            [
                self.delta_x + self.x_shift,
                self.delta_x + self.x_shift,
                -self.delta_x + self.x_shift,
                -self.delta_x + self.x_shift,
            ],
            [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
            [0.0, 0.0, 0.0, 0.0],
        ])

    @property
    def z_clearance(self) -> float:
        """Maximum vertical foot clearance during aerial swing phase [m]."""
        return self.__z_clearance

    @z_clearance.setter
    def z_clearance(self, z: float) -> None:
        self.__z_clearance = float(z)

    @property
    def overlap_ticks(self) -> int:
        """Number of ticks in 4-foot ground support phase."""
        return int(self.overlap_time / self.dt)

    @property
    def swing_ticks(self) -> int:
        """Number of ticks in diagonal swing phase."""
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self) -> int:
        """Number of ticks in ground support phase for a single leg."""
        return 2 * self.overlap_ticks + self.swing_ticks

    @property
    def phase_ticks(self) -> NDArray[np.int_]:
        """Duration in ticks for each of the 4 subphases in a trot cycle."""
        return np.array([
            self.overlap_ticks, 
            self.swing_ticks, 
            self.overlap_ticks, 
            self.swing_ticks
        ])

    @property
    def phase_length(self) -> int:
        """Total duration in ticks for a complete gait cycle (double stride)."""
        return 2 * self.overlap_ticks + 2 * self.swing_ticks
