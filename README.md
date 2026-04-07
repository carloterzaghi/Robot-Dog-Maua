# Robô Quadrúpede - Inspeção e Operação

Este repositório documenta o software de controle e o histórico de desenvolvimento de um robô quadrúpede para inspeção em ambientes de difícil acesso. O sistema utiliza um Raspberry Pi 4, comunicando-se via I2C com um driver Adafruit PCA9685 para controle dos atuadores, e recebendo comandos de um controle PS3 original (Sony) via Bluetooth.

## 🛠️ Hardware Utilizado

| Componente | Modelo / Detalhe |
|---|---|
| Processador | Raspberry Pi 4 |
| Driver de Servos | Adafruit PCA9685 — 16 canais, 12-bit PWM |
| Atuadores | Servomotores Padrão (PWM) |
| Controle | Gamepad PS3 original Sony — Bluetooth via `sixad` |
| Alimentação Lógica | 3.3V (do próprio Raspberry Pi) |
| Alimentação Motores | Fonte chaveada 5V, 2A ou superior |

> ⚠️ **Nota:** Controles clone (Shanwan/Panhai) foram testados mas **não funcionam via Bluetooth** com BlueZ 5.82+ (Debian Trixie). O clone funciona apenas via cabo USB. Recomenda-se usar o controle PS3 **original Sony**.

---

## 🔌 Esquema de Ligação Físico (Wiring)

A alimentação do chip lógico deve ser estritamente separada da alimentação de potência dos motores.

**1. Comunicação I2C e Lógica (Raspberry Pi → PCA9685):**

| Raspberry Pi | PCA9685 |
|---|---|
| 3V3 (Pin 1) | VCC |
| GND | GND |
| SCL (Pin 5) | SCL |
| SDA (Pin 3) | SDA |

**2. Alimentação de Potência (Fonte Externa → PCA9685):**

| Fonte | PCA9685 |
|---|---|
| +5V | Terminal Block V+ (bloco verde) |
| GND | Terminal Block GND |

**3. Servomotores:**

Cada servo conecta ao conector de 3 pinos do canal correspondente: `PWM` (Sinal), `V+` (Tensão) e `GND` (Terra).

---

## ⚙️ Configuração do Sistema

### Habilitando o Barramento I2C

O I2C vem desabilitado por padrão no Raspberry Pi OS:

```bash
sudo raspi-config
# Interface Options → I2C → Enable
sudo reboot
```

Verifique se o PCA9685 foi detectado:

```bash
sudo apt install i2c-tools
i2cdetect -y 1
# O endereço 0x40 deve aparecer na matriz
```

### Troubleshooting de Hardware (Erro I/O 121)

O erro `OSError: [Errno 121] Remote I/O error` ou `ValueError: No I2C device at address: 0x40` indica falha física na fiação. Verifique a fiação VCC 3.3V, GND, SDA e SCL e rode `i2cdetect -y 1` até o endereço `40` aparecer.

---

## 🐍 Ambiente Python e Dependências

Para contornar o bloqueio de segurança do Linux (`externally-managed-environment` / PEP 668), utilizamos um ambiente virtual.

```bash
# Dependências do sistema
sudo apt update
sudo apt install python3-pip python3-venv python3-full

# Criação do ambiente virtual
python3 -m venv env
source env/bin/activate

# Instalação das bibliotecas via requirements.txt
pip install -r requirements.txt
```

> ⚠️ **Importante — sudo com venv:** Scripts que precisam de `sudo` (como o de conexão PS3) devem ser executados usando o Python do ambiente virtual diretamente, não o `python3` do sistema:
> ```bash
> sudo /caminho/para/env/bin/python3 seu_script.py
> ```
> Usar `sudo python3` ignora o venv e não enxerga as bibliotecas instaladas.

---

## 🎮 Configuração do Controle PS3 via Bluetooth

O controle PS3 original é conectado via Bluetooth utilizando `sixad`, que bypassa o sistema de autenticação do BlueZ moderno (incompatível com o protocolo PS3).

### Dependências do sistema

```bash
sudo apt install bluez bluez-tools
# sixad deve ser compilado a partir do código fonte: https://github.com/falkTX/sixad
```

### Configuração do bluetoothd com `--compat`

O `bluetoothd` precisa rodar com a flag `--compat` para habilitar o servidor SDP legado:

```bash
sudo mkdir -p /etc/systemd/system/bluetooth.service.d
sudo nano /etc/systemd/system/bluetooth.service.d/override.conf
```

Conteúdo do arquivo:
```ini
[Service]
ExecStart=
ExecStart=/usr/libexec/bluetooth/bluetoothd --compat --noplugin=sap
```

Aplica as mudanças:
```bash
sudo systemctl daemon-reload
sudo systemctl enable bluetooth
sudo systemctl start bluetooth
```

### Primeira conexão (com cabo USB)

Na **primeira vez**, conecte o controle via USB para gravar o endereço Bluetooth do Raspberry:

```bash
sudo ./sixpair
# Saída esperada:
# Current Bluetooth master: xx:xx:xx:xx:xx:xx
# Setting master bd_addr to d8:3a:dd:1c:52:03
```

### Conectar o controle (sem fio)

Todos os scripts que utilizam o controle chamam automaticamente `ensure_connected()` da biblioteca `lib/connect_ps3_control.py`. Essa função:

1. Verifica se o controle **já está conectado** via `/dev/input/` — se sim, retorna o caminho imediatamente.
2. Se não estiver, executa o fluxo completo de conexão (`bluetooth → sixpair → SDP → sixad`) e aguarda o botão PS ser pressionado.
3. Retorna o caminho correto do dispositivo (ex.: `/dev/input/event5`) automaticamente, sem necessidade de hardcodar.

Também é possível usar a biblioteca diretamente em outros scripts:

```python
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))
from connect_ps3_control import ensure_connected

caminho = ensure_connected()   # conecta se necessário, retorna o path
```

Ou rodar o script standalone (requer sudo + Python do venv):

```bash
sudo env/bin/python3 lib/connect_ps3_control.py
# Pressione o botão PS quando solicitado
```

Saída esperada quando conectar:
```
sixad-sixaxis[xxxx]: Connected 'PLAYSTATION(R)3 Controller (04:76:6E:E5:60:1B)' [Battery 04]
✓ Dispositivo detectado em /dev/input/event5
```

### Habilitar conexão automática no boot

```bash
sudo systemctl enable sixad
sudo systemctl enable bluetooth
```

### ⚠️ Troubleshooting Bluetooth

| Erro | Causa | Solução |
|---|---|---|
| `ModuleNotFoundError: No module named 'evdev'` | `sudo` usa Python do sistema, não o venv | Usar `sudo env/bin/python3 script.py` |
| `unable to connect to sdp session` | `bluetoothd` sem `--compat` | Verificar `override.conf` e reiniciar bluetooth |
| `HID create error 110 (Connection timed out)` | Controle não responde | Pressionar botão PS, verificar bateria |
| `HID create error 107 (Transport endpoint is not connected)` | bluetooth inativo | `sudo systemctl start bluetooth` |
| `Authentication Failed (0x05)` | BlueZ tentando autenticar PS3 | Usar `sixad` em vez de `bluetoothctl` |
| `sixpair: Unable to retrieve local bd_addr` | bluetooth parado ao rodar sixpair | Iniciar bluetooth antes do sixpair |
| Clone Shanwan não conecta via BT | Firmware incompatível com BlueZ 5.82+ | Usar PS3 original ou cabo USB |

---

## 🚀 Como Usar

```bash
# 1. Clone o repositório e crie o ambiente virtual
git clone <url-do-repositorio>
cd Robot-Dog-Maua
python3 -m venv env
source env/bin/activate

# 2. Instale as dependências
pip install -r requirements.txt

# 3. Rode qualquer script de teste com o Python do venv (necessário para sudo)
sudo env/bin/python3 test/ble_control_test.py
# O script conectará o controle automaticamente se necessário.
# Pressione o botão PS do controle quando solicitado.
```

### Listando dispositivos de entrada disponíveis

Útil para depurar qual `eventX` o controle está usando:

```bash
env/bin/python3 test/ble_read.py
# Exemplo de saída:
# /dev/input/event5 --- PLAYSTATION(R)3 Controller
```
