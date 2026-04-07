#!/usr/bin/env python3
"""
Script para conectar o controle PS3 via Bluetooth usando sixad.
Uso: sudo python3 connect_ps3.py

Também pode ser usado como biblioteca:
    from connect_ps3_control import ensure_connected
    caminho = ensure_connected()
"""

import os
import sys
import subprocess
import threading
import time
import signal

# ── Cores ANSI ──────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

def ok(msg):    print(f"{GREEN}[✓]{RESET} {msg}")
def err(msg):   print(f"{RED}[✗]{RESET} {msg}")
def warn(msg):  print(f"{YELLOW}[!]{RESET} {msg}")
def info(msg):  print(f"{CYAN}[…]{RESET} {msg}")

# ── Estado global ────────────────────────────────────────────
connected      = threading.Event()
sixad_process  = None

# ── Verifica root ────────────────────────────────────────────
def check_root():
    if os.geteuid() != 0:
        err("Execute com sudo:  sudo python3 connect_ps3.py")
        sys.exit(1)

# ── Encontra o binário ───────────────────────────────────────
def find_binary(name, extra_paths=None):
    paths = (extra_paths or []) + [
        f"/usr/sbin/{name}",
        f"/usr/bin/{name}",
        f"/usr/local/bin/{name}",
        os.path.expanduser(f"~/{name}"),
        f"./{name}",
    ]
    for p in paths:
        if os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    return None

# ── Passo 1 – Bluetooth ──────────────────────────────────────
def start_bluetooth():
    info("Iniciando serviço bluetooth…")
    r = subprocess.run(
        ["systemctl", "start", "bluetooth"],
        capture_output=True, text=True
    )
    if r.returncode != 0:
        err(f"Falha ao iniciar bluetooth: {r.stderr.strip()}")
        sys.exit(1)
    time.sleep(2)
    ok("Bluetooth iniciado")

# ── Passo 2 – sixpair ────────────────────────────────────────
def run_sixpair():
    sixpair = find_binary("sixpair")
    if not sixpair:
        warn("sixpair não encontrado — pulando etapa de emparelhamento USB")
        return None

    info(f"Executando sixpair ({sixpair})…")
    r = subprocess.run([sixpair], capture_output=True, text=True)
    output = r.stdout + r.stderr

    if "Unable to retrieve" in output:
        warn("sixpair: Bluetooth precisa estar ativo com controle USB conectado")
        return None

    mac = None
    for line in output.splitlines():
        if "Setting master bd_addr to" in line:
            mac = line.split()[-1]

    if mac:
        ok(f"sixpair executado — endereço gravado no controle")
    else:
        warn("sixpair rodou mas não detectou controle USB (tudo bem se já emparelhou antes)")

    return mac

# ── Passo 3 – SDP ────────────────────────────────────────────
def register_sdp():
    info("Registrando perfil SDP…")
    r = subprocess.run(
        ["sdptool", "add", "SP"],
        capture_output=True, text=True
    )
    if "Serial Port service registered" in r.stdout or r.returncode == 0:
        ok("SDP registrado")
    else:
        warn(f"sdptool: {r.stderr.strip() or 'resposta inesperada'}")

# ── Passo 4 – Para instância anterior do sixad ───────────────
def stop_sixad():
    subprocess.run(
        ["sixad", "--stop"],
        capture_output=True
    )
    time.sleep(1)

# ── Monitor do output do sixad (thread) ──────────────────────
def monitor_sixad(proc):
    global sixad_process
    try:
        for line in proc.stdout:
            line = line.strip()
            if line:
                print(f"  {CYAN}sixad>{RESET} {line}")

            if "Connected" in line and "PLAYSTATION" in line:
                connected.set()
                print()
                ok(f"{GREEN}{BOLD}{line}{RESET}")
                print(f"\n{BOLD}🎮  Pronto para usar!{RESET}\n")

            if "error" in line.lower() or "failed" in line.lower():
                warn(line)
    except Exception:
        pass

# ── Passo 5 – Inicia sixad ───────────────────────────────────
def start_sixad():
    global sixad_process

    sixad = find_binary("sixad")
    if not sixad:
        err("sixad não encontrado. Instale com:  sudo apt install sixad")
        sys.exit(1)

    info("Iniciando sixad…")
    sixad_process = subprocess.Popen(
        [sixad, "--start"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    t = threading.Thread(target=monitor_sixad, args=(sixad_process,), daemon=True)
    t.start()
    ok("sixad iniciado")

# ── Aguarda conexão ───────────────────────────────────────────
def wait_for_connection(timeout=30):
    print()
    print(f"⏳  {BOLD}Aguardando conexão…{RESET} "
          f"{YELLOW}Pressione o botão PS do controle!{RESET}")
    print()

    connected.wait(timeout=timeout)

    if not connected.is_set():
        print()
        warn(f"Timeout de {timeout}s — controle não conectou.")
        warn("Dicas:")
        warn("  1. Pressione o botão PS e aguarde")
        warn("  2. Certifique-se de ter rodado sixpair com o cabo USB primeiro")
        warn("  3. Verifique se o bluetooth está ativo: sudo systemctl status bluetooth")
        return False
    return True

# ── Ctrl+C ────────────────────────────────────────────────────
def handle_exit(sig, frame):
    print()
    warn("Interrompido pelo usuário.")
    if sixad_process:
        warn("sixad ainda rodando em background (use 'sudo sixad --stop' para parar)")
    sys.exit(0)

# ── Detecta o dispositivo PS3 via evdev ──────────────────────
def find_ps3_device():
    """Varre /dev/input/ e retorna o caminho do controle PS3 conectado, ou None."""
    try:
        from evdev import InputDevice, list_devices
        for path in list_devices():
            try:
                dev = InputDevice(path)
                name = dev.name.upper()
                if "PLAYSTATION" in name or "SIXAXIS" in name or "PS3" in name:
                    return path
            except Exception:
                pass
    except ImportError:
        warn("evdev não instalado — instale com: pip install evdev")
    return None


# ── API pública para outros scripts ──────────────────────────
def ensure_connected(timeout=30):
    """
    Garante que o controle PS3 esteja conectado.

    - Se já estiver conectado, retorna o caminho do dispositivo imediatamente.
    - Caso contrário, executa todo o processo de conexão (requer sudo) e
      aguarda até `timeout` segundos pelo controle.

    Retorna o caminho do dispositivo (ex.: '/dev/input/event5') ou None em
    caso de falha.
    """
    device = find_ps3_device()
    if device:
        ok(f"Controle já conectado em {device}")
        return device

    # Precisa conectar
    signal.signal(signal.SIGINT, handle_exit)

    print(f"\n{BOLD}🎮  PS3 Controller Connection Script{RESET}")
    print("=" * 40)
    print()

    check_root()
    start_bluetooth()
    run_sixpair()
    register_sdp()
    stop_sixad()
    start_sixad()

    if not wait_for_connection(timeout=timeout):
        return None

    # Espera o sistema registrar o dispositivo
    for _ in range(10):
        device = find_ps3_device()
        if device:
            ok(f"Dispositivo detectado em {device}")
            return device
        time.sleep(0.5)

    warn("Controle conectou via sixad, mas dispositivo não foi encontrado em /dev/input/")
    return None


# ── Main ──────────────────────────────────────────────────────
def main():
    signal.signal(signal.SIGINT, handle_exit)

    print(f"\n{BOLD}🎮  PS3 Controller Connection Script{RESET}")
    print("=" * 40)
    print()

    check_root()
    start_bluetooth()
    run_sixpair()
    register_sdp()
    stop_sixad()
    start_sixad()
    wait_for_connection(timeout=30)

    device = find_ps3_device()
    if device:
        ok(f"Dispositivo disponível em {device}")

    # Mantém o processo vivo enquanto o sixad roda
    if sixad_process:
        try:
            sixad_process.wait()
        except KeyboardInterrupt:
            handle_exit(None, None)

if __name__ == "__main__":
    main()