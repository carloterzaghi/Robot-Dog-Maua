"""
IMU_Teste.py — Script de teste da estabilização (motion/stabilization.py).

Levanta o robô (stand) e mantém a estabilização de roll + pitch ativa via
MPU6050, imprimindo os ângulos lidos e as correções aplicadas em tempo real.

Uso:
    python IMU_Teste.py

Pressione Ctrl+C para parar e colocar o robô em modo sleep com segurança.
"""

import threading

from adafruit_servokit import ServoKit  # type: ignore

from configs.robot_config import SERVO_CHANNELS
from core.servo_manager import ServoManager
from input.gamepad import _ServoManagerAdapter
from motion import poses
from motion.stabilization import stabilize


def main() -> None:
    # 1. Inicializa o controlador de servos (PCA9685, 16 canais, I2C)
    kit = ServoKit(channels=16)

    # 2. Gerenciador de servos (carrega calibração e restaura último estado)
    servo_mgr = ServoManager(kit, channels=SERVO_CHANNELS)

    # 3. Adapta o ServoManager para a interface de atributos usada por
    #    stabilize() (ex: robot_leg.frente_angular_dir.angle)
    robot_leg = _ServoManagerAdapter(servo_mgr)

    stop_event = threading.Event()

    try:
        print("Colocando o robô de pé...")
        poses.stand(servo_mgr)

        print("Iniciando teste de estabilização (IMU). Ctrl+C para parar.\n")
        stabilize(robot_leg, stop_event)

    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário.")

    finally:
        stop_event.set()
        print("Colocando o robô em modo sleep...")
        poses.sleep(servo_mgr)
        print("Teste encerrado.")


if __name__ == "__main__":
    main()
