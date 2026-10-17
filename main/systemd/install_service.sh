#!/usr/bin/env bash
# Instala o serviço systemd para iniciar main/main.py no boot.
# Uso (no Raspberry Pi):  sudo bash main/systemd/install_service.sh
set -euo pipefail

if [ "${EUID:-$(id -u)}" -ne 0 ]; then
  echo "Execute com sudo: sudo bash $0"; exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKDIR="$(dirname "$SCRIPT_DIR")"          # pasta main/ (onde está o main.py)
PROJECT_DIR="$(dirname "$WORKDIR")"         # raiz do projeto (onde fica o venv)
# Usuário que roda o serviço. Padrão root (como 'sudo python main/main.py'),
# necessário para ler /dev/input (evdev) e acessar I2C/GPIO. Sobrescreva com RUN_USER=...
RUN_USER="${RUN_USER:-root}"
SERVICE=/etc/systemd/system/robot-dog.service

# Python: usa o venv do projeto (env, venv ou .venv) — é onde estão evdev, adafruit etc.
if [ -z "${PYTHON:-}" ]; then
  for d in env venv .venv; do
    if [ -x "$PROJECT_DIR/$d/bin/python3" ]; then PYTHON="$PROJECT_DIR/$d/bin/python3"; break; fi
  done
fi
PYTHON="${PYTHON:-$(command -v python3)}"
echo "Python: $PYTHON | Usuário: $RUN_USER | Projeto: $PROJECT_DIR"

if [ ! -f "$WORKDIR/main.py" ]; then
  echo "main.py não encontrado em $WORKDIR"; exit 1
fi

# tr remove CR caso os arquivos tenham sido salvos com quebras de linha do Windows
tr -d '\r' < "$SCRIPT_DIR/robot-dog.service.template" \
  | sed -e "s|__USER__|$RUN_USER|g" \
        -e "s|__WORKDIR__|$WORKDIR|g" \
        -e "s|__PYTHON__|$PYTHON|g" \
        -e "s|__PROJECT__|$PROJECT_DIR|g" \
  > "$SERVICE"

systemctl daemon-reload
systemctl enable robot-dog.service
systemctl restart robot-dog.service

echo "Serviço instalado e iniciado."
echo "Status:  systemctl status robot-dog"
echo "Logs:    journalctl -u robot-dog -f"
echo "Parar:   sudo systemctl stop robot-dog"
echo "Desativar boot: sudo systemctl disable robot-dog"
