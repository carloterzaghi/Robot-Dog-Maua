from adafruit_servokit import ServoKit
import threading
import time
import json
import os

"""
Ponto de entrada de produção: roda direto o Controle Gamesir (opção 7 do menu
de teste em test/leg_test/v4/main.py), sem menu interativo.
"""

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
        com os canais padrão do ServoKit (mapeamento da nova placa v4).
        """
        if dict_servos == {}:
            dict_servos = {
                "frente_femur_dir": kit.servo[10],
                "frente_angular_dir": kit.servo[9],
                "frente_tibia_dir": kit.servo[8],
                "frente_femur_esq": kit.servo[13],
                "frente_angular_esq": kit.servo[14],
                "frente_tibia_esq": kit.servo[15],
                "tras_femur_dir": kit.servo[7],
                "tras_angular_dir": kit.servo[6],
                "tras_tibia_dir": kit.servo[5],
                "tras_femur_esq": kit.servo[2],
                "tras_angular_esq": kit.servo[1],
                "tras_tibia_esq": kit.servo[0],
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
                "tras_tibia_dir":   60,
                "tras_femur_esq":  150,
                "tras_tibia_esq":  120,
            }

        self._smooth_move(struct_start_flexao, 60, 0.02)

        # ── Alvos dos angulares para modo sleep ──────────────────────────────────
        # Servo angular traseiro continua espelhado em relação ao frontal
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
            "tras_femur_dir": 80,
            "tras_tibia_dir": 120,
            "tras_femur_esq": 100,
            "tras_tibia_esq": 70
        }

        # 2. Angulares da frente e trás
        self._smooth_move(ang_start_flexao, n_steps, delay)

        # 3. Tíbias e fêmures da frente e trás
        self._smooth_move(struct_sleep_flexao, n_steps, delay)

    def gamesir_control_mode(self):
        """
        Modo de controle Gamesir:
        - O robô se inicia no modo smooth sleep.
        - No modo sleep, apenas o botão Y funciona, levantando o robô (smooth_flexion_start).
        - No modo levantado, o analógico esquerdo controla a flexão (X/Z) e o botão Y faz o robô voltar ao modo sleep.
        - Botão Start ou Ctrl+C sai do modo e retorna ao modo sleep.
        """
        import sys
        import os
        import math as m
        import select
        import numpy as np

        # Adiciona pasta lib ao sys.path
        _HERE = os.path.dirname(os.path.abspath(__file__))
        _LIB  = os.path.abspath(os.path.join(_HERE, "..", "lib"))
        if _LIB not in sys.path:
            sys.path.insert(0, _LIB)

        try:
            from evdev import InputDevice, ecodes, list_devices # type: ignore
        except ImportError as e:
            print(f"\nErro ao importar bibliotecas (evdev): {e}")
            print("Certifique-se de que o evdev está instalado.")
            return

        from auxiliar_funcs.leg_flexion import _ik, MAX_Z, MAX_RADIUS, X_FIXO

        Z_FUNDO = float(-np.sqrt(max(0.0, MAX_RADIUS**2 - X_FIXO**2)))
        Z_TOPO  = float(MAX_Z)

        print("\nProcurando controle Gamesir...")
        
        devices = [InputDevice(path) for path in list_devices()]
        gamepads = []
        for device in devices:
            cap = device.capabilities()
            if ecodes.EV_KEY in cap:
                keys = cap[ecodes.EV_KEY]
                name_lower = device.name.lower()
                if (isinstance(keys, list) and (ecodes.BTN_SOUTH in keys or ecodes.BTN_A in keys or ecodes.BTN_GAMEPAD in keys)) \
                   or "gamesir" in name_lower or "xbox" in name_lower or "gamepad" in name_lower:
                    gamepads.append(device)
                    
        if not gamepads:
            print("Falha ao detectar o controle Gamesir (nenhum gamepad encontrado).")
            return
            
        device_path = gamepads[0].path

        try:
            gamepad = InputDevice(device_path)
            print(f"Controle conectado com sucesso: {gamepad.name} ({device_path})")
        except Exception as e:
            print(f"Erro ao acessar dispositivo do controle: {e}")
            return
            
        # Obter limites dos analógicos para normalização automática (centro exato em 0.0)
        abs_cap = gamepad.capabilities().get(ecodes.EV_ABS, [])
        abs_x_info = next((info for code, info in abs_cap if code == ecodes.ABS_X), None)
        abs_y_info = next((info for code, info in abs_cap if code == ecodes.ABS_Y), None)
        
        if abs_x_info:
            center_x = (abs_x_info.max + abs_x_info.min) / 2.0
            range_x  = (abs_x_info.max - abs_x_info.min) / 2.0
            if range_x == 0: range_x = 32767.0
        else:
            center_x = 0.0
            range_x = 32767.0
            
        if abs_y_info:
            center_y = (abs_y_info.max + abs_y_info.min) / 2.0
            range_y  = (abs_y_info.max - abs_y_info.min) / 2.0
            if range_y == 0: range_y = 32767.0
        else:
            center_y = 0.0
            range_y = 32767.0

        print("\n=== Modo Controle Gamesir ===")
        print(" -> Robô em modo SLEEP.")
        print(" -> Pressione [Botão Y] para LEVANTAR o robô e iniciar locomoção.")
        print(" -> Quando levantado, use o Analógico Esquerdo (Cima) para andar para frente.")
        print(" -> Pressione [Botão Y] novamente para RETORNAR ao modo sleep.")
        print(" -> Pressione [START] ou Ctrl+C para SAIR.")

        self.smooth_sleep_robot()
        is_standing = False

        import threading
        locomotion_threads = []
        locomotion_stop = threading.Event()
        shared_state = {"speed": 0, "direction": 1, "z_pitch_frente": 0.0, "z_pitch_tras": 0.0, "yaw": 0.0,
                        "imu_roll_offset": 0.0, "imu_pitch_offset": 0.0}
        current_walking_state = 0 # 0=parado, 1=frente, -1=tras
        imu_timer = None

        try:
            from auxiliar_funcs.stabilization import _read_word, MPU6050_ADDR, ACCEL_XOUT_H, PWR_MGMT_1
            import math as _math
            from smbus2 import SMBus as _SMBus
            import time as _t
            with _SMBus(1) as _bus:
                _bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
                _t.sleep(0.1)
                _ax = _read_word(_bus, MPU6050_ADDR, ACCEL_XOUT_H)     / 16384.0
                _ay = _read_word(_bus, MPU6050_ADDR, ACCEL_XOUT_H + 2) / 16384.0
                _az = _read_word(_bus, MPU6050_ADDR, ACCEL_XOUT_H + 4) / 16384.0
                # Mapeamento: +X=TRÁS, +Y=BAIXO (ay=-1g), +Z=DIREITA
                # roll=atan2(az, -ay), pitch=atan2(-ax, -ay)
                shared_state["imu_roll_offset"]  = _math.degrees(_math.atan2(_az, -_ay))
                shared_state["imu_pitch_offset"] = _math.degrees(_math.atan2(-_ax, -_ay))
                print(f"[IMU] Referencial zero capturado: Roll={shared_state['imu_roll_offset']:+.2f}° Pitch={shared_state['imu_pitch_offset']:+.2f}°")
        except Exception as _e:
            print(f"[IMU] Aviso: não foi possível capturar referencial zero: {_e}")

        def schedule_imu_print(delay=2.5):
            nonlocal imu_timer
            if imu_timer is not None:
                imu_timer.cancel()
            imu_timer = threading.Timer(delay, print_current_imu)
            imu_timer.start()

        def print_current_imu():
            try:
                from auxiliar_funcs.stabilization import _read_word, MPU6050_ADDR, ACCEL_XOUT_H
                import math as _math
                from smbus2 import SMBus as _SMBus
                with _SMBus(1) as _bus:
                    _ax = _read_word(_bus, MPU6050_ADDR, ACCEL_XOUT_H)     / 16384.0
                    _ay = _read_word(_bus, MPU6050_ADDR, ACCEL_XOUT_H + 2) / 16384.0
                    _az = _read_word(_bus, MPU6050_ADDR, ACCEL_XOUT_H + 4) / 16384.0
                    # Mapeamento: +X=TRÁS, +Y=BAIXO, +Z=DIREITA
                    roll_raw  = _math.degrees(_math.atan2(_az, -_ay))
                    pitch_raw = _math.degrees(_math.atan2(-_ax, -_ay))
                    # Valores corrigidos pelo offset (o que o estabilizador realmente vê)
                    roll_corr  = roll_raw  - shared_state.get("imu_roll_offset",  0.0)
                    pitch_corr = pitch_raw - shared_state.get("imu_pitch_offset", 0.0)
                    status_roll  = "✓" if abs(roll_corr)  < 2.0 else "✗"
                    status_pitch = "✓" if abs(pitch_corr) < 2.0 else "✗"
                    print(f"[MPU] Bruto  → Roll: {roll_raw:+.2f}°  | Pitch: {pitch_raw:+.2f}°")
                    print(f"[MPU] Erro   → Roll: {roll_corr:+.2f}° {status_roll} | Pitch: {pitch_corr:+.2f}° {status_pitch}  (|<2°| = reto)")
            except Exception as _e:
                print(f"[MPU] Erro ao ler dados: {_e}")

        def start_walking():
            from auxiliar_funcs.leg_test import frente_dir, frente_esq, tras_dir, tras_esq
            from auxiliar_funcs.stabilization import stabilize_full_walking
            nonlocal locomotion_threads, locomotion_stop
            locomotion_stop.clear()
            leg_map = {
                "frente_dir": frente_dir, "frente_esq": frente_esq,
                "tras_dir": tras_dir, "tras_esq": tras_esq
            }
            sync_barrier = threading.Barrier(5)
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
            stab_t = threading.Thread(
                target=stabilize_full_walking,
                args=(locomotion_stop, self, shared_state, sync_barrier),
                daemon=True
            )
            locomotion_threads.append(stab_t)
            stab_t.start()

        def stop_walking():
            nonlocal locomotion_threads, locomotion_stop
            shared_state["yaw"] = 0.0
            locomotion_stop.set()
            for t in locomotion_threads:
                t.join(timeout=3.0)
            # Aguarda um ciclo extra para garantir que a thread de estabilização
            # terminou de escrever nos servos antes de smooth_sleep_robot() assumir.
            time.sleep(0.15)
            locomotion_threads = []

        import time
        top_pressed = False
        top_press_time = 0.0

        # ── Whitelist de códigos aceitos ─────────────────────────────────────────────
        ALLOWED_BUTTONS = set()
        # Botões para levantar/deitar (BTN_NORTH = Y do Xbox, BTN_WEST = X, BTN_TOP)
        ALLOWED_BUTTONS.update([ecodes.BTN_NORTH, ecodes.BTN_WEST, ecodes.BTN_TOP, getattr(ecodes, 'BTN_Y', 308), getattr(ecodes, 'BTN_X', 307)])
        # Start
        ALLOWED_BUTTONS.update([ecodes.BTN_START, 315])

        # Eixos Y (frente/trás) e X (rotação) do analógico esquerdo
        ALLOWED_AXES = {ecodes.ABS_Y, ecodes.ABS_X}

        # Proteção contra duplo-clique de botão durante transição
        TOGGLE_COOLDOWN = 1.5  # segundos mínimos entre toggles
        last_toggle_time = 0.0
        transitioning = False  # True enquanto a thread de toggle está rodando

        def toggle_stand_sleep():
            nonlocal is_standing, transitioning, last_toggle_time
            if transitioning or (time.time() - last_toggle_time < TOGGLE_COOLDOWN):
                print("Aguarde a transição anterior finalizar.")
                return
                
            stop_walking()
            shared_state["speed"] = 0
            last_toggle_time = time.time()
            
            def _run_toggle():
                nonlocal is_standing, transitioning
                transitioning = True
                try:
                    if not is_standing:
                        print("\n[Ação] Levantando robô (smooth_flexion_start)...")
                        self.smooth_flexion_start()
                        print("Ativando estabilização e modo de caminhada...")
                        start_walking()
                        is_standing = True
                        print("Robô levantado e pronto para andar.")
                        schedule_imu_print(2.5)
                    else:
                        print("\n[Ação] Retornando para posição de descanso...")
                        self.smooth_sleep_robot()
                        is_standing = False
                        print("Robô em repouso (sleep). Pressione Botão Y para levantar.")
                finally:
                    transitioning = False
            
            t = threading.Thread(target=_run_toggle, daemon=True)
            t.start()

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
                    
                # Checa se o Botão está pressionado por 2 segundos
                if top_pressed and (time.time() - top_press_time >= 2.0):
                    top_pressed = False # Reseta para não ativar repetidas vezes
                    toggle_stand_sleep()

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
                    
                    # Lógica do Botão Levantar/Deitar
                    if code in (ecodes.BTN_NORTH, ecodes.BTN_WEST, ecodes.BTN_TOP, getattr(ecodes, 'BTN_Y', 308), getattr(ecodes, 'BTN_X', 307)):
                        if val == 1:
                            if not top_pressed:
                                top_pressed = True
                                top_press_time = time.time()
                                print("Segurando Botão de Ação... Aguarde 2 segundos.")
                        elif val == 0:
                            elapsed = time.time() - top_press_time
                            if top_pressed and elapsed < 2.0:
                                top_pressed = False
                                toggle_stand_sleep()
                            top_pressed = False

                    if val == 1:
                        # Botão Start (BTN_START / 315)
                        if code in (ecodes.BTN_START, 315):
                            print("\n[START Pressionado] Encerrando modo Gamesir...")
                            break

                # ── Filtragem de eixos analógicos (EV_ABS) ────────────────────────
                elif event.type == ecodes.EV_ABS:
                    # Ignora eixos durante transição ou se não está de pé
                    if not is_standing or transitioning:
                        continue

                    # Descarta qualquer eixo fora da whitelist (acelerômetros, gatilhos, etc.)
                    if event.code not in ALLOWED_AXES:
                        continue

                    # Filtra os eixos do analógico esquerdo
                    if event.code == ecodes.ABS_Y:
                        axis_y = (event.value - center_y) / range_y
                        
                        # Eixo Y negativo = para cima (frente) / positivo = para trás
                        # Uma zona morta de 15% (0.15) para acomodar folga física do analógico
                        if axis_y <= -0.15:
                            shared_state["speed"] = abs(axis_y)
                            if current_walking_state != 1:
                                shared_state["direction"] = 1
                                print(f"Andando para FRENTE... (Força: {abs(axis_y):.2f})")
                                current_walking_state = 1
                        elif axis_y >= 0.15:
                            shared_state["speed"] = abs(axis_y)
                            if current_walking_state != -1:
                                shared_state["direction"] = -1
                                print(f"Andando para TRÁS... (Força: {abs(axis_y):.2f})")
                                current_walking_state = -1
                        else:
                            shared_state["speed"] = 0
                            if current_walking_state != 0:
                                print("Robô PARADO.")
                                current_walking_state = 0
                                schedule_imu_print(2.0)

                    elif event.code == ecodes.ABS_X:
                        axis_x = (event.value - center_x) / range_x
                        prev_yaw = shared_state["yaw"]
                        
                        # Resposta instântanea e direta
                        shared_state["yaw"] = axis_x

                        if abs(axis_x) > 0.15 and abs(prev_yaw) <= 0.15:
                            side = "DIREITA" if axis_x > 0 else "ESQUERDA"
                            print(f"Girando para {side}... (Força: {abs(axis_x):.2f})")
                        elif abs(axis_x) <= 0.15 and abs(prev_yaw) > 0.15:
                            print("Robô PARADO.")
                            schedule_imu_print(2.0)

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
            print("Modo Gamesir finalizado.")


# ── Execução principal ─────────────────────────────────────────────────────────
if __name__ == "__main__":
    dict_servos = {
        "frente_femur_dir": kit.servo[10],
        "frente_angular_dir": kit.servo[9],
        "frente_tibia_dir": kit.servo[8],

        "frente_femur_esq": kit.servo[13],
        "frente_angular_esq": kit.servo[14],
        "frente_tibia_esq": kit.servo[15],

        "tras_femur_dir": kit.servo[7],
        "tras_angular_dir": kit.servo[6],
        "tras_tibia_dir": kit.servo[5],

        "tras_femur_esq": kit.servo[2],
        "tras_angular_esq": kit.servo[1],
        "tras_tibia_esq": kit.servo[0]
    }

    robot_leg = RobotLeg(dict_servos)
    robot_leg.gamesir_control_mode()
