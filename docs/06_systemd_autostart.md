# ⚙️ Headless Autostart Daemon (Systemd)

> **Relevant Files:** [`main/systemd/install_service.sh`](../main/systemd/install_service.sh), [`main/systemd/robot-dog.service.template`](../main/systemd/robot-dog.service.template)

---

## 1. Headless Field Operation Overview

For field deployments without network cables, active SSH sessions, or tethered displays, the Raspberry Pi OS runs the primary robot control loop as an isolated **background system daemon** managed by **systemd**.

With the service enabled:
1. Turn on the power switch for both the Raspberry Pi and the external servo supply.
2. The operating system boots and automatically starts `robot-dog.service`.
3. Turn on the paired Bluetooth gamepad. The service detects connection and initializes the robot into safe rest mode.
4. Press `L1` on the controller to command the robot to stand up.

---

## 2. Automated Service Installer (`install_service.sh`)

The installer script [`main/systemd/install_service.sh`](../main/systemd/install_service.sh) configures the system unit file (`/etc/systemd/system/robot-dog.service`).

It dynamically resolves:
* The absolute project root path.
* The exact Python executable inside your virtual environment (`env/bin/python3`, `venv/`, or `.venv/`).
* The correct `PYTHONPATH` and working directory (`WorkingDirectory`).

### Installation (Run once on the Raspberry Pi):

```bash
cd Robot-Dog-Maua
sudo bash main/systemd/install_service.sh
```

Terminal confirmation output:
```text
Python: /home/pi/Robot-Dog-Maua/env/bin/python3 | Usuário: root | Projeto: /home/pi/Robot-Dog-Maua
Serviço instalado e iniciado.
```

---

## 3. Service Lifecycle Commands

| Desired Action | Terminal Command |
|---|---|
| **Check Current Status** | `systemctl status robot-dog` |
| **Stream Live Logs** | `journalctl -u robot-dog -f` |
| **Restart Service** | `sudo systemctl restart robot-dog` |
| **Stop Execution** | `sudo systemctl stop robot-dog` |
| **Disable Boot Autostart** | `sudo systemctl disable robot-dog` |
| **Re-enable Boot Autostart** | `sudo systemctl enable robot-dog` |

---

## 4. Diagnostics & Troubleshooting

### Scenario 1: Two processes competing for I2C / servos simultaneously
* **Symptom:** Servos chatter violently, ignore inputs, or behave erratically.
* **Root Cause:** The background systemd service is active while you are also trying to run `main.py` or a test script manually in a terminal.
* **Resolution:** Stop the system service before running manual commands:
  ```bash
  sudo systemctl stop robot-dog
  ```

### Scenario 2: `RuntimeError: Controle não encontrado` in journalctl logs
* **Symptom:** Log inspection shows the process terminating because the gamepad was not detected.
* **Normal Behavior:** During boot, systemd starts services faster than the Bluetooth stack establishes connection with the gamepad.
* **Automatic Retry:** The service unit specifies `Restart=on-failure` and `RestartSec=5s`. Systemd will automatically retry every 5 seconds until the controller is powered on and connected.

### Scenario 3: `ModuleNotFoundError: No module named 'evdev'`
* **Root Cause:** The service was configured using system Python (`/usr/bin/python3`) instead of the project virtual environment (`env/bin/python3`).
* **Resolution:** Ensure the virtual environment exists in the project root (`python3 -m venv env && source env/bin/activate && pip install -r requirements.txt`) and rerun the installer:
  ```bash
  sudo bash main/systemd/install_service.sh
  ```
