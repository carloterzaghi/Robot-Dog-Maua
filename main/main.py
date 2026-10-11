import numpy as np
import time
#from IMU import IMU
from Controller import Controller
from JoystickInterface import JoystickInterface
from State import State, BehaviorState
from Config import Configuration
from Kinematics import four_legs_inverse_kinematics
from adafruit_servokit import ServoKit
from HardwareInterface import HardwareInterface

use_imu = False

config = Configuration()
state = State()
state.height = config.default_z_ref
hardware_interface = HardwareInterface()

joystick = JoystickInterface(config)
last_loop = time.time()

# Create controller and user input handles
controller = Controller(
    config,
    four_legs_inverse_kinematics,
)

SMOOTH_N_STEPS: int   = 60     # passos da interpolação smooth_move
SMOOTH_DELAY:   float = 0.02   # s — delay entre passos


hardware_interface.set_actuator_positions(config.neutral_angle)

def calcualte_default_angles():
    state.foot_locations = (
        config.default_stance
        + np.array([0, 0, config.default_z_ref])[:, np.newaxis]
    )
    state.joint_angles = four_legs_inverse_kinematics(
        state.foot_locations, config
    )
    return state.joint_angles

while True:
    print("Waiting for L1 to activate robot. (Press A to lay/stand)")
    while True:
        command = joystick.get_command(state)


        # if command.L3_button:
        #     if np.all(config.physical_calibration_offsets == 0):
        #         config.physical_calibration_offsets = config.physical_calibration_offsets_oriented
        #         hardware_interface.set_actuator_positions(hardware_interface.current_angles)
        #     else:
        #         config.physical_calibration_offsets = np.zeros((3, 4))
        #         hardware_interface.set_actuator_positions(hardware_interface.current_angles)

        if command.X_button:
            if not state.is_calibration:
                print("Calibração")
                hardware_interface.smooth_transition(config.neutral_angle, n_steps=SMOOTH_N_STEPS, delay=SMOOTH_DELAY)
                state.is_calibration = True
                state.is_laying = False
            else:
                print("Levantando")
                hardware_interface.smooth_transition(calcualte_default_angles(), n_steps=SMOOTH_N_STEPS, delay=SMOOTH_DELAY)
                state.is_calibration = False

        if command.X_button:
            if not state.is_calibration:
                print("Calibração")
                hardware_interface.smooth_transition(config.neutral_angle, n_steps=SMOOTH_N_STEPS, delay=SMOOTH_DELAY)
                state.is_calibration = True
                state.is_laying = False
            else:
                print("Levantando")
                hardware_interface.smooth_transition(calcualte_default_angles(), n_steps=SMOOTH_N_STEPS, delay=SMOOTH_DELAY)
                state.is_calibration = False
            
        if command.A_button:
            if not state.is_laying:
                print("Deitando")
                hardware_interface.smooth_transition(config.lay_angles, n_steps=SMOOTH_N_STEPS, delay=SMOOTH_DELAY)
                state.is_laying = True
                state.is_calibration = False
            else:
                print("Levantando")
                hardware_interface.smooth_transition(calcualte_default_angles(), n_steps=SMOOTH_N_STEPS, delay=SMOOTH_DELAY)
                state.is_laying = False

        if command.activate_event == 1:
            if state.is_laying or state.is_calibration:
                print("Robô estava deitado. Levantando antes de ativar...")
                hardware_interface.smooth_transition(calcualte_default_angles(), n_steps=SMOOTH_N_STEPS, delay=SMOOTH_DELAY)
                state.is_laying = False
                state.is_calibration = False



            state.behavior_state = BehaviorState.REST
            break
        time.sleep(0.1)

    
    print("Robot activated.")
    # config.physical_calibration_offsets = config.physical_calibration_offsets_oriented
    last_loop = time.time()


    while True:

        #print(f"Roll: {state.roll:.2f} | Pitch: {state.pitch:.2f} | Yaw rate: {state.yaw_rate:.2f}")

        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Parse the udp joystick commands and then update the robot controller's parameters
        command = joystick.get_command(state)


        # Read imu data. Orientation will be None if no data was available
        quat_orientation = np.array([1, 0, 0, 0])
        # (
        #     imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        # )
        state.quat_orientation = quat_orientation



        # Step the controller forward by dt
        controller.run(state, command)

        if command.activate_event == 1:
            print("Deactivating Robot")
            hardware_interface.smooth_transition(state.joint_angles, n_steps=30, delay=0.04)
 
            break

        # Atualizar posição dos servos
        hardware_interface.set_actuator_positions(state.joint_angles) 
        
        # print("    ")
        # print("Raw Angles")
        # print(np.matrix( state.joint_angles  ))
        # print("Adjusted angels:")
        # print(np.matrix( abs(config.servo_orientation_adjusts-state.joint_angles) ))


