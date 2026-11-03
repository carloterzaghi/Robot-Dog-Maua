"""Central Locomotion Controller and State Machine Manager (Controller).

Orchestrates behavioral finite state machine transitions (DEACTIVATED, REST, TROT),
integrates ground support (Stance) and aerial trajectory (Swing) controllers,
applies IMU inertial body tilt compensation, and dispatches foot coordinates to
inverse kinematics.
"""

from __future__ import annotations
from typing import Callable, Tuple
import numpy as np
from numpy.typing import NDArray

from Command import Command
from Config import Configuration
from Gaits import GaitController
from StanceController import StanceController
from State import BehaviorState, State
from SwingLegController import SwingController
from Transformations import euler2mat, quat2euler
from Utilities import clipped_first_order_filter


class Controller:
    """Gait planner and state integrator for the quadruped robot."""

    def __init__(
        self,
        config: Configuration,
        inverse_kinematics: Callable[[NDArray[np.float64], Configuration], NDArray[np.float64]],
    ) -> None:
        """Initializes the controller with system configuration and IK solver.

        Args:
            config: Global configuration instance containing geometry and timing parameters.
            inverse_kinematics: Function computing 4-leg inverse kinematics.
        """
        self.config = config
        self.smoothed_yaw: float = 0.0  # Yaw rate smoothing filter for REST mode
        self.inverse_kinematics = inverse_kinematics

        self.contact_modes: NDArray[np.int_] = np.zeros(4, dtype=int)
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)

        # Finite state transition mappings
        self.trot_transition_mapping = {
            BehaviorState.REST: BehaviorState.TROT, 
            BehaviorState.TROT: BehaviorState.REST
        }
        self.activate_transition_mapping = {
            BehaviorState.DEACTIVATED: BehaviorState.REST,
            BehaviorState.REST: BehaviorState.DEACTIVATED,
            BehaviorState.TROT: BehaviorState.DEACTIVATED,  # Immediate deactivation (E-Stop)
        }

    def step_gait(
        self, 
        state: State, 
        command: Command
    ) -> Tuple[NDArray[np.float64], NDArray[np.int_]]:
        """Calculates desired 3D foot locations for the next time step.

        Args:
            state: Current robot state.
            command: Operator command velocity vector.

        Returns:
            Tuple of (foot_locations_matrix, contact_modes_vector).
        """
        contact_modes = self.gait_controller.contacts(state.ticks)
        new_foot_locations = np.zeros((3, 4))

        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            if contact_mode == 1:
                new_location = self.stance_controller.next_foot_location(leg_index, state, command)
            else:
                swing_proportion = (
                    self.gait_controller.subphase_ticks(state.ticks) / self.config.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion,
                    leg_index,
                    state,
                    command
                )
            new_foot_locations[:, leg_index] = new_location

        return new_foot_locations, contact_modes

    def run(self, state: State, command: Command) -> None:
        """Advances the controller forward by one time step (dt = 0.01 s).

        Args:
            state: Global state structure updated in-place.
            command: Latest commands polled from the gamepad.
        """
        # 1. Event-driven state machine transitions
        if command.activate_event:
            state.behavior_state = self.activate_transition_mapping[state.behavior_state]
        elif command.trot_event:
            state.behavior_state = self.trot_transition_mapping[state.behavior_state]

        # 2. Execute active behavioral mode
        if state.behavior_state == BehaviorState.TROT:
            state.foot_locations, self.contact_modes = self.step_gait(state, command)

            # Apply commanded body orientation (Roll & Pitch)
            rotated_foot_locations = (
                euler2mat(command.roll, command.pitch, 0.0) @ state.foot_locations
            )

            # Inertial body tilt compensation using IMU telemetry
            (roll, pitch, yaw) = quat2euler(state.quat_orientation)
            correction_factor = 0.8
            max_tilt = 0.4
            roll_compensation = correction_factor * np.clip(roll, -max_tilt, max_tilt)
            pitch_compensation = correction_factor * np.clip(pitch, -max_tilt, max_tilt)
            rmat = euler2mat(roll_compensation, pitch_compensation, 0.0)

            rotated_foot_locations = rmat.T @ rotated_foot_locations

            # Transform 3D Cartesian coordinates to joint angles
            state.joint_angles = self.inverse_kinematics(rotated_foot_locations, self.config)

        elif state.behavior_state == BehaviorState.REST:
            yaw_proportion = command.yaw_rate / self.config.max_yaw_rate
            self.smoothed_yaw += (
                self.config.dt
                * clipped_first_order_filter(
                    self.smoothed_yaw,
                    yaw_proportion * -self.config.max_stance_yaw,
                    self.config.max_stance_yaw_rate,
                    self.config.yaw_time_constant,
                )
            )

            # Position feet at standard standing stance
            state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, command.height])[:, np.newaxis]
            )

            # Apply static rotational attitude
            rotated_foot_locations = (
                euler2mat(command.roll, command.pitch, self.smoothed_yaw)
                @ state.foot_locations
            )
            state.joint_angles = self.inverse_kinematics(rotated_foot_locations, self.config)

        # 3. Update telemetry and loop counters
        state.ticks += 1
        state.pitch = command.pitch
        state.roll = command.roll
        state.height = command.height

        if state.behavior_state == BehaviorState.DEACTIVATED:
            self.set_pose_to_default(state)

    def set_pose_to_default(self, state: State) -> None:
        """Restores kinematic state to the nominal level standing posture.

        Args:
            state: State structure to reset.
        """
        state.foot_locations = (
            self.config.default_stance
            + np.array([0, 0, self.config.default_z_ref])[:, np.newaxis]
        )
        state.joint_angles = self.inverse_kinematics(state.foot_locations, self.config)

        state.roll = 0.0
        state.pitch = 0.0
        state.yaw_rate = 0.0
        self.smoothed_yaw = 0.0
        state.height = self.config.default_z_ref
