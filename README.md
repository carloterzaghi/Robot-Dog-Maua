# Robô Quadrúpede - Inspeção e Operação

Robô quadrúpede para inspeção em ambientes de difícil acesso. Controlado por um Raspberry Pi 4, com servos via PCA9685 (I2C) e comandos recebidos de um controle PS3 original (Sony) via Bluetooth.

## 🛠️ Hardware

| Componente | Modelo / Detalhe |
|---|---|
| Processador | Raspberry Pi 4 |
| Driver de Servos | Adafruit PCA9685 — 16 canais, 12-bit PWM |
| Atuadores | 12 Servomotores (3 por perna × 4 pernas) |
| IMU | MPU6050 (GY-521) — acelerômetro + giroscópio |
| Controle | Gamepad PS3 original Sony — Bluetooth via `sixad` |
| Alimentação Lógica | 3.3V (do próprio Raspberry Pi) |
| Alimentação Motores | Fonte chaveada 5V, 2A ou superior |

> ⚠️ Controles clone (Shanwan/Panhai) **não funcionam via Bluetooth** com BlueZ 5.82+. Usar PS3 **original Sony** ou cabo USB.

---

## 🔌 Wiring

**I2C (Raspberry Pi → PCA9685):**

| Raspberry Pi | PCA9685 |
|---|---|
| 3V3 (Pin 1) | VCC |
| GND | GND |
| SCL (Pin 5) | SCL |
| SDA (Pin 3) | SDA |

**Potência (Fonte Externa → PCA9685):** +5V e GND no Terminal Block (bloco verde).

**Servos:** cada servo no conector de 3 pinos do canal correspondente (PWM, V+, GND).

---

## 🚀 Quick Start

```bash
# 1. Clone e crie o ambiente virtual
git clone <url-do-repositorio>
cd Robot-Dog-Maua
python3 -m venv env
source env/bin/activate
pip install -r requirements.txt

# 2. Rode um script de teste (sudo necessário para I2C e Bluetooth)
sudo PYTHONPATH=. env/bin/python3 test/leg_test/v3/main.py
# O controle PS3 é conectado automaticamente quando necessário.
```

> ⚠️ **sudo com venv:** sempre use `sudo env/bin/python3` — usar `sudo python3` ignora o venv.

---

## ⚙️ Iniciar automaticamente no boot (systemd)

O `main/main.py` pode subir sozinho quando o Raspberry Pi liga, via serviço `robot-dog`.
Arquivos em [`main/systemd/`](main/systemd/):

| Arquivo | Função |
|---|---|
| `install_service.sh` | Gera `/etc/systemd/system/robot-dog.service`, ativa no boot e inicia |
| `robot-dog.service.template` | Modelo da unit (placeholders preenchidos pelo instalador) |

**Instalação (no Raspberry Pi, a partir da raiz do projeto):**

```bash
sudo bash main/systemd/install_service.sh
```

O instalador imprime `Python: ... | Usuário: ... | Projeto: ...`. Se essa linha não aparecer, o script no Pi está desatualizado — rode `git pull` e instale de novo.

**O que o serviço faz (e por quê):**

- **Python do venv:** usa `env/bin/python3` (ou `venv`/`.venv`) na raiz do projeto, onde estão `evdev`, `adafruit-servokit` etc. Para forçar outro: `sudo PYTHON=/caminho/python3 bash main/systemd/install_service.sh`.
- **Roda como `root`:** igual ao `sudo` manual, necessário para ler `/dev/input/event*` (controle) e acessar I2C. Para outro usuário: `sudo RUN_USER=nome bash ...` (ele precisa estar nos grupos `input` e `i2c`).
- **`PYTHONPATH` e `WorkingDirectory`** apontam para a raiz do projeto (equivale ao `PYTHONPATH=.` do comando manual).
- **Reinício automático:** `Restart=on-failure`, com intervalo de 5 s.

**Comandos úteis:**

```bash
systemctl status robot-dog          # estado
journalctl -u robot-dog -f          # logs ao vivo
sudo systemctl restart robot-dog    # reiniciar
sudo systemctl stop robot-dog       # parar (necessário antes de rodar o main.py manualmente)
sudo systemctl disable robot-dog    # não iniciar mais no boot
```

**Troubleshooting:**

| Sintoma no `journalctl` | Causa / solução |
|---|---|
| `ModuleNotFoundError: No module named 'evdev'` | O serviço está usando o `python3` do sistema, não o do venv. Reinstale com a versão atual do script e confira o venv em `env/` (`pip install -r requirements.txt`) |
| Falha ao achar o controle | Pare o serviço e teste manualmente; o Bluetooth pode demorar após o boot e o controle precisa estar pareado |
| Dois processos controlando os servos | Rode `sudo systemctl stop robot-dog` antes de testar manualmente |

---

## 📁 Estrutura do Projeto

```
Robot-Dog-Maua/
├── main/                       # Versão atual do robô
│   ├── main.py                 #   Loop principal (estados, joystick, servos)
│   ├── Config.py               #   Dimensões, posições neutra/deitado, dt, calibração
│   ├── State.py / Command.py   #   Estado do robô e comandos vindos do controle
│   ├── Controller.py           #   Controlador de marcha (stance + swing)
│   ├── Gaits.py                #   Máquina de fases da marcha
│   ├── StanceController.py     #   Pernas apoiadas no chão
│   ├── SwingLegController.py   #   Pernas no ar
│   ├── Kinematics.py           #   Cinemática inversa das 4 pernas
│   ├── Transformations.py      #   Rotações e transformações
│   ├── HardwareInterface.py    #   Servos via PCA9685 (I2C) + transições suaves
│   ├── JoystickInterface.py    #   Leitura do controle (evdev)
│   └── systemd/                #   Autostart no boot (instalador + unit)
├── legacy_main/                # Versão anterior, docs e testes históricos
│   ├── docs/                   #   Cinemática, configuração do sistema (PS3, WiFi, I2C)
│   ├── test/                   #   Testes de componentes e iterações de perna (v1–v4)
│   └── lib/                    #   Conexão automática do controle PS3
├── Testes Interface/           # Scripts de teste de servos e mapeamento do controle
├── requirements.txt
└── README.md
```

---

## 📖 Documentação

| Documento | Conteúdo |
|-----------|----------|
| [Cinemática](docs/cinematica.md) | Cinemática inversa (IK), modelo geométrico da perna, marcha (gait), estabilização com filtro de Kalman, mapa de servos |
| [Configuração do Sistema](docs/configuracao_sistema.md) | Bluetooth + sixad, pipeline de conexão PS3, fallback WiFi → AP, I2C, Python venv + sudo, troubleshooting |
