from adafruit_servokit import ServoKit
import threading
import time
import json
import os


# ── Inicialização do controlador de servos (PCA9685, 16 canais, I2C) ──────────
kit = ServoKit(channels=16)

class RobotLeg:

    # ── Ângulos base da posição default (referência para cálculo de offsets) ───
    _DEFAULT_ANGLES = {
        "frente_femur_dir":    90,
        "frente_angular_dir": 100,
        "frente_tibia_dir":    90,
        "frente_femur_esq":    90,
        "frente_angular_esq": 105,
        "frente_tibia_esq":    90,
        "tras_femur_dir":      90,
        "tras_angular_dir":   100,
        "tras_tibia_dir":      90,
        "tras_femur_esq":      90,
        "tras_angular_esq":   105,
        "tras_tibia_esq":      90,
    }

    # Caminho do arquivo de calibração (mesmo diretório do script)
    _CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs", "calibration.json")

    # Caminho do arquivo de estado dos servos (ultimo ângulo comandado)
    _STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs", "servo_state.json")

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
                "frente_tibia_esq": kit.servo[7],
                "tras_femur_dir": kit.servo[9],
                "tras_angular_dir": kit.servo[10],
                "tras_tibia_dir": kit.servo[11],
                "tras_femur_esq": kit.servo[13],
                "tras_angular_esq": kit.servo[14],
                "tras_tibia_esq": kit.servo[15],
            }

        # ── Servos da perna frontal direita ─────────────────────────────────────────────────────
        self.frente_femur_dir   = dict_servos["frente_femur_dir"]
        self.frente_angular_dir = dict_servos["frente_angular_dir"]
        self.frente_tibia_dir   = dict_servos["frente_tibia_dir"]

        # ── Servos da perna frontal esquerda ─────────────────────────────────────────────────────
        self.frente_femur_esq   = dict_servos["frente_femur_esq"]
        self.frente_angular_esq = dict_servos["frente_angular_esq"]
        self.frente_tibia_esq   = dict_servos["frente_tibia_esq"]

        # ── Servos da perna traseira direita ─────────────────────────────────────────────────────
        self.tras_femur_dir   = dict_servos["tras_femur_dir"]
        self.tras_angular_dir = dict_servos["tras_angular_dir"]
        self.tras_tibia_dir   = dict_servos["tras_tibia_dir"]

        # ── Servos da perna traseira esquerda ─────────────────────────────────────────────────────
        self.tras_femur_esq   = dict_servos["tras_femur_esq"]
        self.tras_angular_esq = dict_servos["tras_angular_esq"]
        self.tras_tibia_esq   = dict_servos["tras_tibia_esq"]

        # ── Carrega offsets de calibração salvos (ou inicializa com zeros) ──────
        if os.path.exists(self._CALIB_FILE):
            try:
                with open(self._CALIB_FILE, "r") as f:
                    self.offsets = json.load(f)
                print(f"[Calibração] Offsets carregados de {self._CALIB_FILE}")
            except Exception as e:
                print(f"[Calibração] Falha ao carregar offsets: {e} — usando zeros.")
                self.offsets = {k: 0.0 for k in self._DEFAULT_ANGLES}
        else:
            self.offsets = {k: 0.0 for k in self._DEFAULT_ANGLES}

        # ── Restaura última posição conhecida para evitar pico de corrente ─────
        # Ao comandar cada servo para seu último ângulo gravado, o adafruit_servokit
        # passa a conhecer a posição inicial e a interpolação funciona corretamente.
        # Como os servos já estão fisicamente nessa posição, nenhum movimento ocorre.
        self._load_state()

    def _all_servos(self):
        """Retorna dict com todos os servos indexados pelo nome."""
        return {
            "frente_femur_dir":   self.frente_femur_dir,
            "frente_angular_dir": self.frente_angular_dir,
            "frente_tibia_dir":   self.frente_tibia_dir,
            "frente_femur_esq":   self.frente_femur_esq,
            "frente_angular_esq": self.frente_angular_esq,
            "frente_tibia_esq":   self.frente_tibia_esq,
            "tras_femur_dir":     self.tras_femur_dir,
            "tras_angular_dir":   self.tras_angular_dir,
            "tras_tibia_dir":     self.tras_tibia_dir,
            "tras_femur_esq":     self.tras_femur_esq,
            "tras_angular_esq":   self.tras_angular_esq,
            "tras_tibia_esq":     self.tras_tibia_esq,
        }

    def _load_state(self):
        """
        Carrega o ultimo angulo comandado de cada servo a partir de servo_state.json
        e re-comanda o servo para esse angulo. Como o servo ja esta fisicamente
        nessa posicao (nenhum movimento ocorre), o adafruit_servokit passa a
        conhecer o ponto de partida e a interpolacao suave funciona na proxima chamada.
        """
        if not os.path.exists(self._STATE_FILE):
            return
        try:
            with open(self._STATE_FILE, "r") as f:
                state = json.load(f)
            for name, srv in self._all_servos().items():
                angle = state.get(name)
                if angle is not None:
                    srv.angle = float(angle)
            print(f"[Estado] Posição anterior restaurada de {self._STATE_FILE}")
        except Exception as e:
            print(f"[Estado] Falha ao restaurar estado: {e}")

    def _save_state(self):
        """
        Grava o angulo atual de todos os servos em servo_state.json
        para que a proxima execucao saiba o ponto de partida.
        """
        state = {
            name: (srv.angle if srv.angle is not None else None)
            for name, srv in self._all_servos().items()
        }
        try:
            with open(self._STATE_FILE, "w") as f:
                json.dump(state, f, indent=4)
        except Exception as e:
            print(f"[Estado] Falha ao salvar estado: {e}")

    def smooth_default_position(self, n_steps=60, delay=0.02):
        """
        Move o robô para a posição padrão de forma gradual,
        usando uma curva ease-in-out (smootherstep) para evitar picos de corrente.
        """
        targets = {
            "frente_femur_dir":    90,
            "frente_angular_dir": 100,
            "frente_tibia_dir":    90,
            "frente_femur_esq":    90,
            "frente_angular_esq": 105,
            "frente_tibia_esq":    90,
            "tras_femur_dir":    90,
            "tras_angular_dir": 100,
            "tras_tibia_dir":    90,
            "tras_femur_esq":    90,
            "tras_angular_esq": 105,
            "tras_tibia_esq":    90,
        }
        self._smooth_move(targets, n_steps, delay)

    def _smooth_move(self, targets, n_steps=60, delay=0.02):
        """
        Move os servos dos ângulos atuais até os alvos de forma gradual,
        usando uma curva smootherstep (ease-in-out: 6t⁵ - 15t⁴ + 10t³) para
        acelerar e desacelerar suavemente, evitando picos de corrente.
        """
        servos = {
            "frente_femur_dir":   self.frente_femur_dir,
            "frente_angular_dir": self.frente_angular_dir,
            "frente_tibia_dir":   self.frente_tibia_dir,
            "frente_femur_esq":   self.frente_femur_esq,
            "frente_angular_esq": self.frente_angular_esq,
            "frente_tibia_esq":   self.frente_tibia_esq,
            "tras_femur_dir":   self.tras_femur_dir,
            "tras_angular_dir": self.tras_angular_dir,
            "tras_tibia_dir":   self.tras_tibia_dir,
            "tras_femur_esq":   self.tras_femur_esq,
            "tras_angular_esq": self.tras_angular_esq,
            "tras_tibia_esq":   self.tras_tibia_esq,
        }
        # Filtra apenas os servos que estão presentes nos alvos
        active_servos = {name: srv for name, srv in servos.items() if name in targets}

        # Aplica offsets de calibração e clipa para [0, 180]
        adjusted = {
            name: max(0.0, min(180.0, targets[name] + self.offsets.get(name, 0.0)))
            for name in active_servos
        }

        # Lê ângulo atual; usa o alvo ajustado como fallback se ainda não foi definido
        starts = {
            name: (srv.angle if srv.angle is not None else adjusted[name])
            for name, srv in active_servos.items()
        }
        for step in range(1, n_steps + 1):
            t = step / n_steps
            # Smootherstep ease-in-out (C²): começa devagar, acelera, desacelera no final
            t_smooth = t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
            for name, srv in active_servos.items():
                srv.angle = starts[name] + (adjusted[name] - starts[name]) * t_smooth
            time.sleep(delay)

        # Persiste a posição final de todos os servos
        self._save_state()

    def smooth_flexion_start(self, n_steps=60, delay=0.02):
        """
        Move o robô para a posição inicial de flexão de forma gradual,
        usando uma curva ease-in-out (smootherstep) para evitar picos de corrente.
        Posição calculada via IK para x=X_FIXO=-9 mm, z=MAX_Z=-20 mm.

        Sequência de movimentos:
          1. Tíbias e fêmures da frente → espera 0.2 s → tíbias e fêmures de trás.
          2. Angulares da frente       → espera 0.2 s → angulares de trás.
        """
        # ── Alvos de tíbia/fêmur ─────────────────────────────────────────────────
        struct_start = {
            "frente_femur_dir":   30,
            "frente_tibia_dir":   60,
            "frente_femur_esq":  150,
            "frente_tibia_esq":  120,
            "tras_femur_dir":   30,
            "tras_tibia_dir":   40,
            "tras_femur_esq":  150,
            "tras_tibia_esq":  135,
        }

        # ── Alvos dos angulares ───────────────────────────────────────────────────
        ang_start = {
            "frente_angular_dir": 100,
            "frente_angular_esq": 105,
            "tras_angular_dir": 100,
            "tras_angular_esq": 105,
        }

        # 1. Tíbias e fêmures da frente e tras
        self._smooth_move(struct_start, n_steps, delay)

        # 2. Angulares da frente e tras
        self._smooth_move(ang_start, n_steps, delay)

    def smooth_sleep_robot(self, n_steps=60, delay=0.02):
        """
        Move o robô para a posição de descanso de forma gradual,
        usando uma curva ease-in-out (smootherstep) para evitar picos de corrente.
        """
        

        struct_start_flexao = {
                "frente_femur_dir":   30,
                "frente_tibia_dir":   60,
                "frente_femur_esq":  150,
                "frente_tibia_esq":  120,
                "tras_femur_dir":   30,
                "tras_tibia_dir":   40,
                "tras_femur_esq":  150,
                "tras_tibia_esq":  135,
            }

        self._smooth_move(struct_start_flexao, 60, 0.02)

        # ── Alvos dos angulares para modo sleep ──────────────────────────────────
        ang_start_flexao = {
            "frente_angular_dir": 70,
            "frente_angular_esq": 135,
            "tras_angular_dir": 135,
            "tras_angular_esq": 70,
        }


        # ── Alvos de tíbia/fêmur para modo sleep ─────────────────────────────────
        struct_sleep_flexao = {
            "frente_femur_dir": 80,
            "frente_tibia_dir": 120,
            "frente_femur_esq": 100,
            "frente_tibia_esq": 70,
            "tras_femur_dir": 55,
            "tras_tibia_dir": 60,
            "tras_femur_esq": 120,
            "tras_tibia_esq": 100
        }

        # 2. Angulares da frente e trás
        self._smooth_move(ang_start_flexao, n_steps, delay)

        # 3. Tíbias e fêmures da frente e trás
        self._smooth_move(struct_sleep_flexao, n_steps, delay)


    def test_leg_movement(self, dict_legs={"frente_dir": 0, "frente_esq": 0, "tras_dir": 0, "tras_esq": 0}, use_angular=True):
        """
        Testa o movimento da perna do robô, movendo os servos para diferentes ângulos.
        Pernas com valor 1 são movidas simultaneamente em threads separadas.
        """
        from auxiliar_funcs.leg_test import frente_dir, frente_esq, tras_dir, tras_esq
        import threading

        stop_event = threading.Event()
        leg_map = {
            "frente_dir": frente_dir,
            "frente_esq": frente_esq,
            "tras_dir":   tras_dir,
            "tras_esq":   tras_esq,
        }

        active_legs = [leg for leg, enabled in dict_legs.items() if enabled == 1]
        if not active_legs:
            return

        sync_barrier = threading.Barrier(len(active_legs))
        threads = []

        for leg in active_legs:
            func = leg_map[leg]
            delay_val = 0.8 if "tras" in leg else 0.0
            t = threading.Thread(
                target=func, 
                args=(self, stop_event, use_angular, delay_val, sync_barrier), 
                daemon=True
            )
            threads.append(t)

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
        Testa a flexão da perna varrendo o eixo Z com X fixo em -9 mm.
        Quando há pernas de frente E de trás selecionadas:
          1. Pernas da frente iniciam a descida imediatamente.
          2. Pernas de trás aguardam 0.2 segundos em cada ciclo antes de descer.
          3. Pernas da frente esperam no fundo (barrier) até as de trás chegarem.
          4. Todas sobem juntas do fundo ao topo.
          5. Repete o ciclo continuamente com o mesmo escalonamento.
        """
        from auxiliar_funcs.leg_flexion import frente_dir, frente_esq, tras_dir, tras_esq

        stop_event = threading.Event()

        n_front = sum(1 for leg in ["frente_dir", "frente_esq"] if dict_legs.get(leg) == 1)
        n_rear  = sum(1 for leg in ["tras_dir",   "tras_esq"]   if dict_legs.get(leg) == 1)
        n_total = n_front + n_rear

        # Barrier com N = total de pernas ativas (todas esperam no fundo)
        sync_barrier = threading.Barrier(n_total) if n_total > 1 else None
        rear_delay = 0.2 if (n_front > 0 and n_rear > 0) else 0.0


        front_threads = [
            threading.Thread(
                target=func,
                args=(self, stop_event, sync_barrier, 0.0),
                daemon=True,
            )
            for leg, func in [("frente_dir", frente_dir), ("frente_esq", frente_esq)]
            if dict_legs.get(leg) == 1
        ]
        rear_threads = [
            threading.Thread(
                target=func,
                args=(self, stop_event, sync_barrier, rear_delay),
                daemon=True,
            )
            for leg, func in [("tras_dir", tras_dir), ("tras_esq", tras_esq)]
            if dict_legs.get(leg) == 1
        ]

        all_threads = front_threads + rear_threads
        for t in all_threads:
            t.start()

        try:
            for t in all_threads:
                t.join()
        except KeyboardInterrupt:
            print("\nParado. Retornando à posição inicial.")
            stop_event.set()
            if sync_barrier is not None:
                try:
                    sync_barrier.abort()
                except Exception:
                    pass
            for t in all_threads:
                t.join() 

            self.smooth_sleep_robot()


    def test_stabilization(self):
        """
        Testa a estabilização completa do robô (4 pernas — roll + pitch) usando
        o MPU6050 com filtro de Kalman.
        - Roll: ajusta os 4 servos angulares (canais 2, 6, 10, 14).
        - Pitch: ajusta fêmur e tíbia das 4 pernas via IK variando Z.
        """
        from auxiliar_funcs.stabilization import stabilize


        stop_event = threading.Event()
        t = threading.Thread(target=stabilize, args=(self, stop_event), daemon=True)
        t.start()

        try:
            t.join()
        except KeyboardInterrupt:
            print("\nParado. Retornando à posição de repouso.")
            stop_event.set()
            t.join()
            self.smooth_sleep_robot()

    def calibration_mode(self):
        """
        Modo de calibração interativo: move para a posição default, exibe os
        ângulos atuais de todos os servos e permite ajustar qualquer servo
        individualmente até o usuário digitar 'sair'.
        """
        SERVO_NAMES = [
            "frente_femur_dir",
            "frente_angular_dir",
            "frente_tibia_dir",
            "frente_femur_esq",
            "frente_angular_esq",
            "frente_tibia_esq",
            "tras_femur_dir",
            "tras_angular_dir",
            "tras_tibia_dir",
            "tras_femur_esq",
            "tras_angular_esq",
            "tras_tibia_esq",
        ]

        servos = {
            "frente_femur_dir":   self.frente_femur_dir,
            "frente_angular_dir": self.frente_angular_dir,
            "frente_tibia_dir":   self.frente_tibia_dir,
            "frente_femur_esq":   self.frente_femur_esq,
            "frente_angular_esq": self.frente_angular_esq,
            "frente_tibia_esq":   self.frente_tibia_esq,
            "tras_femur_dir":     self.tras_femur_dir,
            "tras_angular_dir":   self.tras_angular_dir,
            "tras_tibia_dir":     self.tras_tibia_dir,
            "tras_femur_esq":     self.tras_femur_esq,
            "tras_angular_esq":   self.tras_angular_esq,
            "tras_tibia_esq":     self.tras_tibia_esq,
        }

        print("\n── Modo de Calibração ──────────────────────────")
        print("Movendo para posição default...")
        self.smooth_default_position()

        while True:
            print("\nÂngulos atuais dos servos:")
            print(f"  {'#':>2}  {'Nome':<22} {'Atual':>7}  {'Default':>8}  {'Offset':>7}")
            print("  " + "-" * 52)
            for i, name in enumerate(SERVO_NAMES, start=1):
                angle = servos[name].angle
                angle_str = f"{angle:.1f}°" if angle is not None else "  N/A"
                default_str = f"{self._DEFAULT_ANGLES[name]:.1f}°"
                offset_str  = f"{self.offsets.get(name, 0.0):+.1f}°"
                print(f"  {i:>2}  {name:<22} {angle_str:>7}  {default_str:>8}  {offset_str:>7}")

            print("\nComandos:")
            print("  <número> <ângulo>  — move o servo  (ex: 3 120)")
            print("  salvar             — calcula e salva os offsets")
            print("  sair               — volta ao menu principal")
            raw = input("\n> ").strip()

            if raw.lower() == "sair":
                print("Saindo do modo de calibração.")
                break

            if raw.lower() == "salvar":
                # Calcula offset = ângulo calibrado - ângulo default
                new_offsets = {}
                for name in SERVO_NAMES:
                    current = servos[name].angle
                    if current is None:
                        current = self._DEFAULT_ANGLES[name]
                    new_offsets[name] = round(current - self._DEFAULT_ANGLES[name], 2)
                self.offsets = new_offsets
                try:
                    with open(self._CALIB_FILE, "w") as f:
                        json.dump(self.offsets, f, indent=4)
                    print(f"  ✓  Offsets salvos em {self._CALIB_FILE}")
                except Exception as e:
                    print(f"  ✗  Erro ao salvar: {e}")
                print("\n  Offsets calculados:")
                for name, off in self.offsets.items():
                    print(f"     {name:<22}  {off:+.2f}°")
                continue

            parts = raw.split()
            if len(parts) != 2:
                print("  ✗  Formato inválido. Use: <número> <ângulo>  (ex: 3 120)")
                continue

            try:
                idx = int(parts[0])
                angle = float(parts[1])
            except ValueError:
                print("  ✗  Número ou ângulo inválido.")
                continue

            if not (1 <= idx <= len(SERVO_NAMES)):
                print(f"  ✗  Número do servo deve ser entre 1 e {len(SERVO_NAMES)}.")
                continue

            if not (0 <= angle <= 180):
                print("  ✗  Ângulo deve estar entre 0 e 180.")
                continue

            name = SERVO_NAMES[idx - 1]
            servos[name].angle = angle
            print(f"  ✓  {name} → {angle:.1f}°")

    def ps3_control_mode(self):
        """
        Modo de controle PS3:
        - O robô se inicia no modo smooth sleep.
        - No modo sleep, apenas o botão O (BTN_EAST) funciona, levantando o robô (smooth_flexion_start).
        - No modo levantado, o analógico esquerdo controla a flexão (X/Z) e o botão O faz o robô voltar ao modo sleep.
        - Botão Start ou Ctrl+C sai do modo e retorna ao modo sleep.
        """
        import sys
        import os
        import math as m
        import select
        import numpy as np

        # Adiciona pasta lib ao sys.path
        _HERE = os.path.dirname(os.path.abspath(__file__))
        _LIB  = os.path.abspath(os.path.join(_HERE, "..", "..", "..", "lib"))
        if _LIB not in sys.path:
            sys.path.insert(0, _LIB)

        try:
            from connect_ps3_control import ensure_connected # type: ignore
            from evdev import InputDevice, ecodes # type: ignore
        except ImportError as e:
            print(f"\nErro ao importar bibliotecas do PS3 (evdev/connect_ps3_control): {e}")
            print("Certifique-se de que o evdev está instalado e o script de conexão disponível.")
            return

        from auxiliar_funcs.leg_flexion import _ik, MAX_Z, MAX_RADIUS, X_FIXO

        Z_FUNDO = float(-np.sqrt(max(0.0, MAX_RADIUS**2 - X_FIXO**2)))
        Z_TOPO  = float(MAX_Z)

        print("\nConectando ao controle PS3...")
        device_path = ensure_connected(timeout=15)
        if not device_path:
            print("Falha ao conectar o controle PS3.")
            return

        try:
            gamepad = InputDevice(device_path)
            print(f"Controle conectado com sucesso: {gamepad.name} ({device_path})")
        except Exception as e:
            print(f"Erro ao acessar dispositivo do controle: {e}")
            return

        print("\n=== Modo Controle PS3 ===")
        print(" -> Robô em modo SLEEP.")
        print(" -> Pressione [Botão TOP] para LEVANTAR o robô e iniciar locomoção.")
        print(" -> Quando levantado, use o Analógico Esquerdo (Cima) para andar para frente.")
        print(" -> Pressione [Botão TOP] novamente para RETORNAR ao modo sleep.")
        print(" -> Pressione [START] ou Ctrl+C para SAIR.")

        self.smooth_sleep_robot()
        is_standing = False

        DEADZONE = 0.20
        def deadzone(val, dz=DEADZONE):
            if abs(val) < dz:
                return 0.0
            sign = 1.0 if val > 0 else -1.0
            return sign * (abs(val) - dz) / (1.0 - dz)

        import threading
        locomotion_threads = []
        locomotion_stop = threading.Event()
        shared_state = {"speed": 0, "direction": 1}
        current_walking_state = 0 # 0=parado, 1=frente, -1=tras

        def start_walking():
            from auxiliar_funcs.leg_test import frente_dir, frente_esq, tras_dir, tras_esq
            nonlocal locomotion_threads, locomotion_stop
            locomotion_stop.clear()
            leg_map = {
                "frente_dir": frente_dir, "frente_esq": frente_esq,
                "tras_dir": tras_dir, "tras_esq": tras_esq
            }
            sync_barrier = threading.Barrier(4)
            locomotion_threads = []
            for leg, func in leg_map.items():
                delay_val = 0.8 if "tras" in leg else 0.0
                t = threading.Thread(
                    target=func,
                    args=(self, locomotion_stop, False, delay_val, sync_barrier, shared_state),
                    daemon=True
                )
                locomotion_threads.append(t)
                t.start()

        def stop_walking():
            nonlocal locomotion_threads, locomotion_stop
            locomotion_stop.set()
            for t in locomotion_threads:
                t.join(timeout=2.0)
            locomotion_threads = []

        import time
        pending_walk = False
        walk_start_time = 0
        last_axis_y = 0.0
        DEBOUNCE_TIME = 0.40 # 400ms segurando o analógico para confirmar
        
        top_pressed = False
        top_press_time = 0.0

        # ── Whitelist de códigos aceitos ─────────────────────────────────────────────
        # Apenas os botões e eixos que realmente usamos. Tudo fora é descartado.
        ALLOWED_BUTTONS = set()
        # Botão TOP
        ALLOWED_BUTTONS.update([ecodes.BTN_TOP])
        # Start
        ALLOWED_BUTTONS.update([ecodes.BTN_START, 315])

        # Apenas o eixo Y do analógico esquerdo
        ALLOWED_AXES = {ecodes.ABS_Y}

        # Proteção contra duplo-clique do Botão TOP durante transição
        TOGGLE_COOLDOWN = 1.5  # segundos mínimos entre toggles
        last_toggle_time = 0.0
        transitioning = False  # True enquanto a thread de toggle está rodando

        try:
            # Grab exclusivo: impede que outros processos leiam o controle
            gamepad.grab()
        except Exception:
            print("Aviso: não foi possível obter grab exclusivo do controle.")

        try:
            # Lemos os eventos em loop bloqueante
            for event in gamepad.read_loop():

                # ── Ignora eventos de sincronização (EV_SYN) ──────────────────────
                if event.type == ecodes.EV_SYN:
                    continue

                # Debounce temporal: aproveitamos qualquer evento para checar o tempo
                if pending_walk and (time.time() - walk_start_time > DEBOUNCE_TIME):
                    shared_state["speed"] = abs(last_axis_y)
                    shared_state["direction"] = 1
                    if current_walking_state != 1:
                        print(f"Andando para FRENTE... (Força: {abs(last_axis_y):.2f})")
                        current_walking_state = 1
                    pending_walk = False
                    
                # Checa se o Botão TOP está pressionado por 2 segundos
                if top_pressed and (time.time() - top_press_time >= 2.0):
                    top_pressed = False # Reseta para não ativar repetidas vezes
                    if not is_standing:
                        print("\n[Botão TOP 2s] Levantando robô (Modo Caminhada)...")
                        shared_state["speed"] = 0
                        start_walking()
                        is_standing = True
                        print("Robô levantado. Use o analógico esquerdo (Cima) para andar.")
                    else:
                        print("\n[Botão TOP 2s] Voltando para o modo sleep...")
                        stop_walking()
                        self.smooth_sleep_robot()
                        is_standing = False
                        print("Robô em repouso (sleep). Pressione [Botão TOP] por 2s para levantar.")

                # ── Filtragem de botões (EV_KEY) ──────────────────────────────────
                if event.type == ecodes.EV_KEY:
                    code = event.code
                    val  = event.value  # 1 = pressionado, 0 = solto

                    # Descarta qualquer botão que não está na whitelist
                    if code not in ALLOWED_BUTTONS:
                        continue

                    # Ignora evento de "repeat" (val == 2)
                    if val not in (0, 1):
                        continue
                    
                    # Lógica do Botão TOP
                    if code == ecodes.BTN_TOP:
                        if val == 1:
                            if not top_pressed:
                                top_pressed = True
                                top_press_time = time.time()
                                print("Segurando Botão TOP... Aguarde 2 segundos.")
                        elif val == 0:
                            elapsed = time.time() - top_press_time
                            if top_pressed and elapsed < 2.0:
                                # Cooldown: ignora se estamos em transição ou dentro do período
                                if transitioning or (time.time() - last_toggle_time < TOGGLE_COOLDOWN):
                                    print("Aguarde a transição anterior finalizar.")
                                    top_pressed = False
                                    continue

                                # Clique simples: alterna entre levantar e deitar
                                stop_walking()
                                shared_state["speed"] = 0
                                last_toggle_time = time.time()
                                
                                def _run_toggle():
                                    nonlocal is_standing, transitioning
                                    transitioning = True
                                    try:
                                        if not is_standing:
                                            print("\\n[Botão TOP] Indo para posição inicial e levantando (smooth_flexion_start)...")
                                            self.smooth_flexion_start()
                                            print("Robô levantando para modo de caminhada...")
                                            start_walking()
                                            is_standing = True
                                            print("Robô levantado e pronto para andar.")
                                        else:
                                            print("\\n[Botão TOP] Retornando para posição de descanso...")
                                            struct_start_flexao = {
                                                "frente_femur_dir":   30,
                                                "frente_tibia_dir":   60,
                                                "frente_femur_esq":  150,
                                                "frente_tibia_esq":  120,
                                                "tras_femur_dir":   30,
                                                "tras_tibia_dir":   40,
                                                "tras_femur_esq":  150,
                                                "tras_tibia_esq":  135,
                                            }
                                            self._smooth_move(struct_start_flexao, 60, 0.02)
                                            self.smooth_sleep_robot()
                                            is_standing = False
                                            print("Robô em repouso (sleep).")
                                    finally:
                                        transitioning = False
                                
                                t = threading.Thread(target=_run_toggle, daemon=True)
                                t.start()
                            top_pressed = False

                    if val == 1:
                        # Botão Start (BTN_START / 315)
                        if code in (ecodes.BTN_START, 315):
                            print("\n[START Pressionado] Encerrando modo PS3...")
                            break

                # ── Filtragem de eixos analógicos (EV_ABS) ────────────────────────
                elif event.type == ecodes.EV_ABS:
                    # Ignora eixos durante transição ou se não está de pé
                    if not is_standing or transitioning:
                        continue

                    # Descarta qualquer eixo fora da whitelist (acelerômetros, gatilhos, etc.)
                    if event.code not in ALLOWED_AXES:
                        continue

                    # Filtra apenas o eixo Y do analógico esquerdo
                    if event.code == ecodes.ABS_Y:
                        
                        # Filtro de hardware: Se o valor for muito bizarro (fora de 0-255), 
                        # é o acelerômetro 16-bit do controle mandando lixo no mesmo eixo.
                        if event.value < -50 or event.value > 305:
                            continue
                            
                        raw_y = (event.value - 128.0) / 128.0
                        axis_y = deadzone(raw_y)
                        
                        # Eixo Y negativo = para cima (frente)
                        if axis_y <= -1.03:
                            last_axis_y = axis_y
                            if current_walking_state != 1 and not pending_walk:
                                # Inicia o timer de debounce
                                pending_walk = True
                                walk_start_time = time.time()
                            elif current_walking_state == 1:
                                # Se já está andando, apenas atualiza a força contínua
                                shared_state["speed"] = abs(axis_y)
                        else:
                            # Se a força for menor que 1.03, ou se foi só um ruído fantasma, PARA.
                            pending_walk = False
                            shared_state["speed"] = 0
                            if current_walking_state != 0:
                                print("Robô PARADO.")
                                current_walking_state = 0

        except KeyboardInterrupt:
            print("\nInterrompido pelo usuário.")
        finally:
            stop_walking()
            try:
                gamepad.ungrab()
            except Exception:
                pass
            print("Retornando robô ao modo sleep...")
            self.smooth_sleep_robot()
            print("Modo PS3 finalizado.")

# ── Execução principal / Menu interativo ──────────────────────────────────────
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

    print("Movendo para posição inicial...")
    robot_leg.smooth_sleep_robot()

    while op != "0":
        print("\nEscolha uma opção:")
        print("1 - Testar pernas (locomoção)")
        print("2 - Modo Default")
        print("3 - Modo Sleep")
        print("4 - Flexão de pernas (varredura de eixo)")
        print("5 - Estabilização lateral (roll + pitch + Filtro de Kalman)")
        print("6 - Modo de Calibração")
        print("7 - Controle PS3 (Movimentação do Robô)")
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
            robot_leg.smooth_default_position()
        elif op == "3":
            robot_leg.smooth_sleep_robot() 
        elif op == "4":
            leg_input = input("Pernas para testar - frente_dir, frente_esq, tras_dir, tras_esq (ex: 1,0,0,0): ")
            leg_keys = ["frente_dir", "frente_esq", "tras_dir", "tras_esq"]
            values = [v.strip() for v in leg_input.split(",")]
            robot_leg.smooth_flexion_start()
            legs_to_test = {leg_keys[i]: int(v) for i, v in enumerate(values) if i < len(leg_keys)}
            robot_leg.test_leg_flexion(legs_to_test)
        elif op == "5":
            print("\nIniciando estabilização lateral...")
            robot_leg.smooth_default_position()
            robot_leg.test_stabilization()
        elif op == "6":
            robot_leg.calibration_mode()
        elif op == "7":
            robot_leg.ps3_control_mode()
        elif op == "0":
            print("Saindo...")
            break
        else:
            print("Opção inválida. Tente novamente.")
    robot_leg.smooth_sleep_robot()