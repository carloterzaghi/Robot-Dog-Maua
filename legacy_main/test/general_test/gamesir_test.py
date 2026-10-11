"""
gamesir_test.py
─────────────────────────────────────────────────────────────────────────────
Ferramenta para testar e descobrir os mapeamentos (botões e eixos) do 
controle Gamesir conectado via Dongle (2.4GHz) ou cabo USB.
"""

import sys
import time

try:
    from evdev import InputDevice, list_devices, ecodes, categorize # type: ignore
except ImportError:
    print("Biblioteca evdev não instalada. Execute: pip install evdev")
    sys.exit(1)

def find_gamepad():
    devices = [InputDevice(path) for path in list_devices()]
    gamepads = []
    
    print("Procurando dispositivos de entrada no sistema...")
    for device in devices:
        # print(f" - Verificando: {device.path} | {device.name}")
        cap = device.capabilities()
        # Heurística: checa se tem eventos de tecla e se tem os botões de gamepad comuns
        if ecodes.EV_KEY in cap:
            keys = cap[ecodes.EV_KEY]
            name_lower = device.name.lower()
            
            # Se tiver botões de gamepad ou se o nome indicar Gamesir/Xbox
            if (isinstance(keys, list) and (ecodes.BTN_SOUTH in keys or ecodes.BTN_A in keys or ecodes.BTN_GAMEPAD in keys)) \
               or "gamesir" in name_lower \
               or "xbox" in name_lower \
               or "gamepad" in name_lower:
                gamepads.append(device)
    
    if not gamepads:
        print("\nNenhum controle com cara de Gamepad/Gamesir foi detectado automaticamente.")
        print("Dispositivos disponíveis no momento:")
        for device in devices:
            print(f" - {device.path} : {device.name}")
        return None
        
    print("\nControle(s) detectado(s) automaticamente:")
    for i, gp in enumerate(gamepads):
        print(f" [{i}] {gp.name} ({gp.path})")
        
    # Pega o primeiro por padrão
    return gamepads[0].path

def main():
    print("=" * 70)
    print("  TESTE GERAL - CONTROLE GAMESIR (VIA DONGLE/USB)")
    print("=" * 70)
    
    device_path = find_gamepad()
    if not device_path:
        print("\nCaso não tenha sido detectado, insira o caminho abaixo.")
        device_path = input("Caminho do dispositivo (ex: /dev/input/event0) ou Enter para sair: ").strip()
        if not device_path:
            return

    try:
        gamepad = InputDevice(device_path)
        print(f"\n[SUCESSO] Conectado ao dispositivo: {gamepad.name} ({gamepad.path})")
    except Exception as e:
        print(f"[ERRO] Falha ao acessar o dispositivo {device_path}: {e}")
        return

    print("-" * 70)
    print("  Pronto! Pressione os botões ou mova os analógicos do Gamesir.")
    print("  Qualquer ação será mapeada na tela. Pressione Ctrl+C para sair.")
    print("-" * 70)

    try:
        # Tenta obter acesso exclusivo ao controle, evite interferir com outros apps
        gamepad.grab()
    except Exception:
        print("Aviso: Não foi possível obter acesso exclusivo ao dispositivo (grab).")

    try:
        for event in gamepad.read_loop():
            # Ignora eventos de SYN (sincronização do final do bloco de evento)
            if event.type == ecodes.EV_SYN:
                continue

            ts = time.strftime("%H:%M:%S")
            
            if event.type == ecodes.EV_KEY:
                # É uma tecla ou botão
                key_event = categorize(event)
                
                # Mapeia pressionado/solto
                if key_event.keystate == key_event.key_down:
                    status = "PRESSIONADO"
                elif key_event.keystate == key_event.key_up:
                    status = "SOLTO"
                else:
                    status = "MANTIDO (HOLD)"
                    
                # Exibe a tecla
                keycode = key_event.keycode
                # No evdev, o keycode pode vir como uma lista se houver múltiplos nomes pro mesmo código
                if isinstance(keycode, list):
                    keycode_str = " / ".join(keycode)
                else:
                    keycode_str = str(keycode)
                    
                print(f"[{ts}] [BOTÃO] Código interno: {event.code:<5} Nome: {keycode_str:<20} Estado: {status}")
                
            elif event.type == ecodes.EV_ABS:
                # É um eixo analógico ou gatilho
                abs_name = ecodes.ABS[event.code] if event.code in ecodes.ABS else f"ABS_UNKNOWN_{event.code}"
                
                # Tratamento do nome se também for uma lista (raro em ABS, mas pode acontecer)
                if isinstance(abs_name, list):
                    abs_name = " / ".join(abs_name)
                    
                print(f"[{ts}] [ EIXO] Código interno: {event.code:<5} Nome: {abs_name:<20} Valor atual: {event.value}")
                
            else:
                # Outros tipos de evento (Misc, ForceFeedback, etc)
                print(f"[{ts}] [OUTRO] Tipo: {event.type} Código: {event.code} Valor: {event.value}")

    except KeyboardInterrupt:
        print("\n\n[INFO] Teste encerrado pelo usuário.")
    finally:
        try:
            gamepad.ungrab()
        except Exception:
            pass
        print("Desconectado do controle.")

if __name__ == "__main__":
    main()
