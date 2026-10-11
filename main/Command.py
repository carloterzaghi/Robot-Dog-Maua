import numpy as np


class Command:
    def __init__(self, config):
        # Velocidade desejada [vx, vy] em m/s
        self.horizontal_velocity = np.array([0.0, 0.0])

        # Velocidade angular desejada em yaw [rad/s]
        self.yaw_rate = 0.0

        # Altura desejada do corpo [m]
        self.height = -0.22

        # Orientação desejada do corpo [rad]
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0

        # Eventos
        self.activate_event = False
        self.trot_event = False
        self.A_button = False
        self.X_button = False
        self.L3_button = False

        