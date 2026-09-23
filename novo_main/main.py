import numpy as np
import time
#from IMU import IMU
from Controller import Controller
from JoystickInterface import JoystickInterface
from State import State
from Config import Configuration
from Kinematics import four_legs_inverse_kinematics
from adafruit_servokit import ServoKit
#from HardwareInterface import HardwareInterface

use_imu = False

config = Configuration()

state = State()
state.height = config.default_z_ref
kit = ServoKit(channels=16)
#hardware_interface = HardwareInterface()

joystick = JoystickInterface(config)
last_loop = time.time()

# Create controller and user input handles
controller = Controller(
    config,
    four_legs_inverse_kinematics,
)

def set_actuator_postions(joint_angles, pins, kit):
    """
    Envia os ângulos da matriz 3x4 para os servos correspondentes.

    Args:
        joint_angles : np.ndarray (3, 4) — ângulos em RADIANOS
                       linhas → articulação (0=angular, 1=fêmur, 2=tíbia)
                       colunas → perna      (0=FR, 1=FL, 2=RR, 3=RL)
        pins         : np.ndarray (3, 4) — canal PCA9685 de cada servo
                       mesma organização de linhas/colunas
        kit          : ServoKit — instância do adafruit_servokit
    """
    print(" ")
    print("Definindo angulos:")
    print(np.matrix((joint_angles)))
    
    for row in range(3):          # articulação
        for col in range(4):      # perna
            canal = int(pins[row, col])
            angulo_deg = (joint_angles[row, col])
            angulo_deg = float(np.clip(angulo_deg, 0.0, 180.0))
            kit.servo[canal].angle = angulo_deg
    

# controller.set_pose_to_default(state)
# hardware_interface.set_actuator_postions(state.joint_angles)



while True:
    print("Waiting for L1 to activate robot.")
    while True:
        command = joystick.get_command(state)
        if command.activate_event == 1:
            break
        time.sleep(0.1)
    print("Robot activated.")



    while True:
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()

        # Parse the udp joystick commands and then update the robot controller's parameters
        command = joystick.get_command(state)
        if command.activate_event == 1:
            print("Deactivating Robot")
            break

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = np.array([1, 0, 0, 0])
        # (
        #     imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        # )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        controller.run(state, command)

        # Atualizar posição dos servos
        #hardware_interface.set_actuator_postions(state.joint_angles)
        set_actuator_postions(abs(-state.joint_angles-config.servo_multipliers), config.pins, kit)
        print("Raw Angles")
        print(np.matrix( state.joint_angles  ))
        print("       ")





# while True:

#     command = joystick.get_command(state)

#     print(
#         "Vel:",
#         np.round(command.horizontal_velocity, 3),
#         "| Yaw:",
#         round(command.yaw_rate, 3),
#         "| Pitch:",
#         round(command.pitch, 3),
#         "| Roll:",
#         round(command.roll, 3),
#         "| Height:",
#         round(command.height, 3),
#         "| ACT:",
#         command.activate_event,
#         "| TROT:",
#         command.trot_event,
#     )
#     state.height = command.height
#     state.roll = command.roll
#     state.pitch = command.pitch

#     time.sleep(0.05)

