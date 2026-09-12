"""
main.py — Ponto de entrada de produção do robô quadrúpede (Robot-Dog-Maua).

Ao iniciar, vai diretamente para o modo Normal (sleep) sem tela de seleção.
O robô deita imediatamente com fêmures e tíbias se movendo juntos.

Botões disponíveis durante operação:
  [Botão Y]  → Toggle stand/sleep
  [START]    → Sair

Para entrar no Modo Config (calibração), edite a variável START_IN_CONFIG
    abaixo ou inicie com argumento --config.

Para testes individuais de subsistemas, use os scripts em test/.

Estrutura do projeto:
  configs/robot_config.py         — constantes de hardware e marcha
  core/servo_manager.py           — gerenciamento dos 12 servos
  core/leg.py                     — cinemática de cada perna
  motion/poses.py                 — transições de pose (stand, sleep)
  motion/locomotion.py            — orquestração de threads de marcha
  motion/stabilization.py        — estabilização roll/pitch via MPU6050
  input/gamepad.py                — leitura de gamepad e máquina de estados
  input/gamepad_calibration.py   — modo de calibração via gamepad
"""


import sys

from adafruit_servokit import ServoKit  # type: ignore

from configs.robot_config import LEG_CONFIG, SERVO_CHANNELS
from core.leg import Leg
from core.servo_manager import ServoManager
from input.gamepad import GamepadReader, RobotController
from input.gamepad_calibration import run_gamepad_calibration
from motion import poses


# ── Seleção de modo inicial (usada apenas após calibração) —————————————————

def _choose_mode(reader: GamepadReader) -> str:
    """
    Bloqueia até o usuário pressionar um botão na tela de seleção.
    Usada apenas após uma sessão de calibração para decidir o próximo passo.

    Retorna:
      "normal"  — [X] pressionado
      "config"  — [Y] pressionado
      "exit"    — [START] pressionado
    """
    ecodes = reader._ecodes

    print("\n" + "═" * 56)
    print("  Robot-Dog-Maua — Selecione o modo de operação:")
    print("─" * 56)
    print("  [X]     → Modo Normal   (controle de locomoção)")
    print("  [Y]     → Modo Config   (calibração de servos)")
    print("  [START] → Sair")
    print("═" * 56)

    btn_x     = getattr(ecodes, "BTN_WEST", 308)        # X
    btn_y     = getattr(ecodes, "BTN_NORTH", 307)       # Y
    btn_start = ecodes.BTN_START

    for event in reader.events():
        if event.type != ecodes.EV_KEY or event.value != 1:
            continue

        if event.code == btn_x:
            print("\n  [X] Modo Normal selecionado.")
            return "normal"

        if event.code == btn_y:
            print("\n  [Y] Modo de Calibração selecionado.")
            return "config"

        if event.code in reader.EXIT_BUTTONS or event.code == btn_start:
            print("\n  [START] Encerrando...")
            return "exit"

    return "exit"


# ── Ponto de entrada ─────────────────────────────────────────────────────────────

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

    # 4. Conecta o gamepad (compartilhado entre os modos)
    print("\nProcurando controle Gamesir...")
    try:
        reader = GamepadReader()
    except (ImportError, RuntimeError) as exc:
        print(f"Erro: {exc}")
        return

    print(f"Controle conectado: {reader.name} ({reader.path})")
    reader.grab()

    try:
        # Coloca o robô em modo sleep imediatamente antes de aguardar a seleção
        poses.sleep_direct(servo_mgr)
        
        mode = _choose_mode(reader)

        if mode == "config":
            # 5a. Modo Calibração — calibra servos via gamepad
            run_gamepad_calibration(servo_mgr, reader)
            # Após calibrar, oferece ao usuário entrar no modo normal
            print("\nCalibração concluída.")
            mode = _choose_mode(reader)

        if mode == "normal":
            # 5b. Modo Normal — vai direto para sleep ao iniciar.
            #     Fêmures e tíbias se movem juntos (sleep_direct).
            print("\n" + "═" * 56)
            print("  Robot-Dog-Maua — Iniciando em modo SLEEP")
            print("─" * 56)
            print("  [Botão Y] → Levantar / Deitar o robô")
            print("  [START]   → Sair")
            print("═" * 56)

            poses.sleep_direct(servo_mgr)

            # O reader já está com grab ativo; passa-o para o RobotController.
            # Como o robô já foi colocado em sleep, o RobotController não
            # chama poses.sleep() novamente ao entrar no loop.
            controller = RobotController(servo_mgr, legs)
            controller.run_with_reader(reader, skip_initial_sleep=True)

    finally:
        reader.ungrab()
        print("Controle liberado. Encerrando.")


if __name__ == "__main__":
    main()
