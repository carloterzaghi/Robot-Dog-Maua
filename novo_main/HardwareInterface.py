from Config import ServoParams
from adafruit_servokit import ServoKit


kit = ServoKit(channels=16)

def enviar_sinal(canal: int, angulo: float):
    """Envia o comando de ângulo (0° a 180°) para o servo da porta escolhida."""
    kit.servo[canal].angle = angulo
    print(f"-> Sinal enviado: kit.servo[{canal}].angle = {angulo}° ({PORTAS[canal]})")


class HardwareInterface:
    def __init__(self):
        self.pi = pigpio.pi()
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()
        initialize_pwm(self.pi, self.pwm_params)

    def set_actuator_postions(self, joint_angles):
        send_servo_commands(self.pi, self.pwm_params, self.servo_params, joint_angles)
    
    def set_actuator_position(self, joint_angle, axis, leg):
        send_servo_command(self.pi, self.pwm_params, self.servo_params, joint_angle, axis, leg)