"""
main.py — Ponto de entrada de produção do robô quadrúpede (Robot-Dog-Maua).

Inicializa os componentes e entra diretamente no modo de controle Gamesir.
Para testes individuais de subsistemas, use os scripts em test/.

Estrutura do projeto:
  configs/robot_config.py   — constantes de hardware e marcha
  core/servo_manager.py     — gerenciamento dos 12 servos
  core/leg.py               — cinemática de cada perna
  motion/poses.py           — transições de pose (stand, sleep)
  motion/locomotion.py      — orquestração de threads de marcha
  motion/stabilization.py   — estabilização roll/pitch via MPU6050
  input/gamepad.py          — leitura de gamepad e máquina de estados
"""

from adafruit_servokit import ServoKit  # type: ignore

from configs.robot_config import LEG_CONFIG, SERVO_CHANNELS
from core.leg import Leg
from core.servo_manager import ServoManager
from input.gamepad import RobotController


def main() -> None:
    # 1. Inicializa o controlador de servos (PCA9685, 16 canais, I2C)
    kit = ServoKit(channels=16)

    # 2. Cria o gerenciador de servos
    #    (carrega calibração e restaura último estado automaticamente)
    servo_mgr = ServoManager(kit, channels=SERVO_CHANNELS)

    # 3. Instancia as 4 pernas com suas configurações individuais
    legs: dict[str, Leg] = {
        name: Leg(name, servo_mgr, cfg)
        for name, cfg in LEG_CONFIG.items()
    }

    # 4. Inicia o modo de controle Gamesir
    controller = RobotController(servo_mgr, legs)
    controller.run()


if __name__ == "__main__":
    main()
