from adafruit_servokit import ServoKit        
import threading
import time
import numpy as np


# ── Inicialização do controlador de servos (PCA9685, 16 canais, I2C) ──────────
kit = ServoKit(channels=16)

class RobotLeg:
    def __init__(self, dict_servos = {}):
        """
        Inicializa os servos da perna do robô. Se dict_servos for fornecido, ele será 
        usado para inicializar os servos; caso contrário, os servos serão inicializados 
        com os canais padrão do ServoKit.
        """
        if dict_servos == {}:
            dict_servos = {
                "frente_femur_dir": kit.servo[1],
                "frente_angular_dir": kit.servo[2],
                "frente_tibia_dir": kit.servo[3],
                "frente_femur_esq": kit.servo[5],
                "frente_angular_esq": kit.servo[6],
                "frente_tibia_esq": kit.servo[7]
            }

        # ── Servos da perna frontal direita ─────────────────────────────────────────────────────
        self.frente_femur_dir   = dict_servos["frente_femur_dir"]
        self.frente_angular_dir = dict_servos["frente_angular_dir"]
        self.frente_tibia_dir   = dict_servos["frente_tibia_dir"]

        # ── Servos da perna frontal esquerda ─────────────────────────────────────────────────────
        self.frente_femur_esq   = dict_servos["frente_femur_esq"]
        self.frente_angular_esq = dict_servos["frente_angular_esq"]
        self.frente_tibia_esq   = dict_servos["frente_tibia_esq"]
    
    def set_leg_default_position(self):
        """
        Move a perna do robô para a posição padrão.
        """
        self.frente_femur_dir.angle   = 90
        self.frente_angular_dir.angle = 100
        self.frente_tibia_dir.angle   = 90

        self.frente_femur_esq.angle = 90
        self.frente_angular_esq.angle = 105
        self.frente_tibia_esq.angle = 90

    def smooth_sleep_robot(self, n_steps=60, delay=0.02):
        """
        Move o robô para a posição de descanso de forma gradual,
        interpolando dos ângulos atuais até o destino.
        """
        targets = {
            "frente_femur_dir":   70,
            "frente_angular_dir": 100,
            "frente_tibia_dir":   90,
            "frente_femur_esq":   110,
            "frente_angular_esq": 105,
            "frente_tibia_esq":   90,
        }
        servos = {
            "frente_femur_dir":   self.frente_femur_dir,
            "frente_angular_dir": self.frente_angular_dir,
            "frente_tibia_dir":   self.frente_tibia_dir,
            "frente_femur_esq":   self.frente_femur_esq,
            "frente_angular_esq": self.frente_angular_esq,
            "frente_tibia_esq":   self.frente_tibia_esq,
        }
        # Lê ângulo atual; usa o alvo como fallback se ainda não foi definido
        starts = {
            name: (srv.angle if srv.angle is not None else targets[name])
            for name, srv in servos.items()
        }
        for step in range(1, n_steps + 1):
            t = step / n_steps
            for name, srv in servos.items():
                srv.angle = starts[name] + (targets[name] - starts[name]) * t
            time.sleep(delay)

    def sleep_robot(self):
        """
        Move o robô para a posição de descanso.
        """
        self.frente_femur_dir.angle   = 70
        self.frente_angular_dir.angle = 100
        self.frente_tibia_dir.angle   = 90

        self.frente_femur_esq.angle = 110
        self.frente_angular_esq.angle = 105
        self.frente_tibia_esq.angle = 90

    def test_leg_movement(self, dict_legs = {"frente_dir": 0, "frente_esq": 0, "tras_dir": 0, "tras_esq": 0}, use_angular=True):
        """
        Testa o movimento da perna do robô, movendo os servos para diferentes ângulos.
        Pernas com valor 1 são movidas simultaneamente em threads separadas.
        """
        from auxiliar_funcs.leg_test import frente_dir, frente_esq, tras_dir, tras_esq

        stop_event = threading.Event()
        leg_map = {
            "frente_dir": frente_dir,
            "frente_esq": frente_esq,
            "tras_dir":   tras_dir,
            "tras_esq":   tras_esq,
        }

        threads = [
            threading.Thread(target=func, args=(self, stop_event, use_angular), daemon=True)
            for leg, func in leg_map.items()
            if dict_legs.get(leg) == 1
        ]

        for t in threads:
            t.start()

        try:
            for t in threads:
                t.join()
        except KeyboardInterrupt:
            print("\nParado. Retornando à posição inicial.")
            stop_event.set()
            for t in threads:
                t.join()
            self.smooth_sleep_robot()

    def test_leg_flexion(self, dict_legs = {"frente_dir": 0, "frente_esq": 0, "tras_dir": 0, "tras_esq": 0}):
        """
        Testa a flexão da perna varrendo o eixo Z com X fixo em -9 mm (posição de descanso).
        Pernas com valor 1 são movidas simultaneamente em threads separadas.
        """
        from auxiliar_funcs.leg_flexion import frente_dir, frente_esq, tras_dir, tras_esq

        stop_event = threading.Event()
        leg_map = {
            "frente_dir": frente_dir,
            "frente_esq": frente_esq,
            "tras_dir":   tras_dir,
            "tras_esq":   tras_esq,
        }

        threads = [
            threading.Thread(target=func, args=(self, stop_event), daemon=True)
            for leg, func in leg_map.items()
            if dict_legs.get(leg) == 1
        ]

        for t in threads:
            t.start()

        try:
            for t in threads:
                t.join()
        except KeyboardInterrupt:
            print("\nParado. Retornando à posição inicial.")
            stop_event.set()
            for t in threads:
                t.join()
            self.smooth_sleep_robot()




if __name__ == "__main__":

    dict_servos = {
        "frente_femur_dir": kit.servo[1],
        "frente_angular_dir": kit.servo[2],
        "frente_tibia_dir": kit.servo[3],

        "frente_femur_esq": kit.servo[5],
        "frente_angular_esq": kit.servo[6],
        "frente_tibia_esq": kit.servo[7],

        "tras_femur_dir": kit.servo[9],
        "tras_angular_dir": kit.servo[10],
        "tras_tibia_dir": kit.servo[11],

        "tras_femur_esq": kit.servo[13],
        "tras_angular_esq": kit.servo[14],
        "tras_tibia_esq": kit.servo[15]
    }

    robot_leg = RobotLeg(dict_servos)

    op = ""

    while op != "0":
        print("\nEscolha uma opção:")
        print("1 - Testar pernas (locomoção)")
        print("2 - Modo Default")
        print("3 - Modo Sleep")
        print("4 - Flexão de pernas (varredura de eixo)")
        print("0 - Sair")

        op = input("Opção: ")

        if op == "1":
            leg_input = input("Pernas para testar - frente_dir, frente_esq, tras_dir, tras_esq (ex: 1,0,0,0): ")
            leg_keys = ["frente_dir", "frente_esq", "tras_dir", "tras_esq"]
            values = [v.strip() for v in leg_input.split(",")]
            legs_to_test = {leg_keys[i]: int(v) for i, v in enumerate(values) if i < len(leg_keys)}
            ang_input = input("Testar movimento angular? (s/n): ")
            use_angular = ang_input.strip().lower() != "n"
            robot_leg.test_leg_movement(legs_to_test, use_angular=use_angular)
        elif op == "2":
            robot_leg.set_leg_default_position()
        elif op == "3":
            robot_leg.sleep_robot() 
        elif op == "4":
            leg_input = input("Pernas para testar - frente_dir, frente_esq, tras_dir, tras_esq (ex: 1,0,0,0): ")
            leg_keys = ["frente_dir", "frente_esq", "tras_dir", "tras_esq"]
            values = [v.strip() for v in leg_input.split(",")]
            legs_to_test = {leg_keys[i]: int(v) for i, v in enumerate(values) if i < len(leg_keys)}
            robot_leg.test_leg_flexion(legs_to_test)
        elif op == "0":
            print("Saindo...")
            break
        else:
            print("Opção inválida. Tente novamente.")
    robot_leg.sleep_robot()