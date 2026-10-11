import time
from Config import ServoParams
from adafruit_servokit import ServoKit
from Config import Configuration
import numpy as np

config = Configuration()

class HardwareInterface:
    def __init__(self):
        self.kit = ServoKit(channels=16)
        self.servo_params = ServoParams()
        self.current_angles = np.zeros((3, 4))
        self.initialize_servo()

    def initialize_servo(self):
        for i in range(16):
            self.kit.servo[i].actuation_range = 180
            self.kit.servo[i].set_pulse_width_range(440, 2560)
            
    def set_actuator_positions(self, joint_angles, use_servo_calibration = True):
        """
        Envia os ângulos da matriz 3x4 para os servos correspondentes.

        Args:
            joint_angles : np.ndarray (3, 4) — ângulos em GRAUS
                        linhas → articulação (0=angular, 1=fêmur, 2=tíbia)
                        colunas → perna      (0=FR, 1=FL, 2=RR, 3=RL)
            pins         : np.ndarray (3, 4) — canal PCA9685 de cada servo
                        mesma organização de linhas/colunas
            kit          : ServoKit — instância do adafruit_servokit
        """

        # print(" ")
        # print("Definindo angulos:")
        # print(np.matrix((joint_angles)))
        self.current_angles = np.copy(joint_angles)        

        for row in range(3):          # articulação
            for col in range(4):      # perna
                canal = int(self.servo_params.pins[row, col])
                if use_servo_calibration:
                    angulo_deg = (joint_angles[row, col] - config.physical_calibration_offsets[row, col])
                else:
                    angulo_deg = joint_angles[row, col]
                angulo_deg = float(np.clip(angulo_deg, 0.0, 180.0))
                self.kit.servo[canal].angle = angulo_deg

        

    def smooth_transition(self, target_angles, n_steps=25, delay=0.02):
        """
        Transição suave da posição atual para a posição alvo 
        """

        start_angles = np.copy(self.current_angles)
        for step in range(1, n_steps + 1):
            t = step / n_steps
            t_smooth = t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
            interpolated = start_angles + (target_angles - start_angles) * t_smooth
            self.set_actuator_positions(interpolated)
            time.sleep(delay)

