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

## 📁 Estrutura do Projeto

```
Robot-Dog-Maua/
├── docs/                           # Documentação detalhada
│   ├── cinematica.md               # Cinemática inversa, marcha e estabilização
│   └── configuracao_sistema.md     # PS3, WiFi fallback, I2C, venv + sudo
├── lib/
│   └── connect_ps3_control.py      # Conexão automática do controle PS3
├── test/
│   ├── general_test/               # Testes de componentes individuais
│   │   ├── GY-521_test.py          #   IMU (acelerômetro + giroscópio)
│   │   ├── servo_test.py           #   Servos individuais
│   │   ├── ble_control_test.py     #   Controle PS3 (leitura de inputs)
│   │   └── ble_read.py             #   Listagem de dispositivos /dev/input/
│   └── leg_test/
│       ├── v1/                     # Primeira iteração (4-bar linkage)
│       ├── v2/                     # Segunda iteração
│       └── v3/                     # Versão atual
│           ├── main.py             #   Menu principal (locomoção, flexão, estabilização)
│           ├── ps3_leg_flexion.py   #   Controle de flexão via PS3
│           ├── angulo_manual.py    #   IK interativa (input X/Z manual)
│           └── auxiliar_funcs/     #   Módulos de cinemática e controle
├── requirements.txt
└── README.md
```

---

## 📖 Documentação

| Documento | Conteúdo |
|-----------|----------|
| [Cinemática](docs/cinematica.md) | Cinemática inversa (IK), modelo geométrico da perna, marcha (gait), estabilização com filtro de Kalman, mapa de servos |
| [Configuração do Sistema](docs/configuracao_sistema.md) | Bluetooth + sixad, pipeline de conexão PS3, fallback WiFi → AP, I2C, Python venv + sudo, troubleshooting |
