import math
from adafruit_servokit import ServoKit

# ──────────────────────────────────────────────
# Configuração dos servos e dimensões (em mm)
# ──────────────────────────────────────────────
kit = ServoKit(channels=16)
servo_ombro  = kit.servo[0]  # Controla a barra de 120mm (L1)
servo_joelho = kit.servo[4]  # Controla o horn de 25mm (L_horn)

# Constantes Mecânicas
L1 = 120.0        # Fêmur
L2 = 130.0        # Tíbia
L_EXT = 35.0      # Extensão roxa (alavanca da tíbia)
L_HORN = 25.0     # Horn do segundo servo (laranja)
L_HASTE = 120.0   # Barra conectora superior (vermelha)
OFFSET_X = 37.8   # Distância horizontal entre os centros dos servos

def calcular_cinematica_inversa(target_x, target_y):
    try:
        # --- ETAPA 1: Perna Principal (Servo Ombro) ---
        dist_alvo = math.sqrt(target_x**2 + target_y**2)
        
        # Verifica se o alvo é maior que a perna esticada (evita erro matemático)
        if dist_alvo > (L1 + L2):
            return None, None
            
        # Lei dos cossenos para o ângulo interno do quadril
        cos_alpha_h = (L1**2 + dist_alvo**2 - L2**2) / (2 * L1 * dist_alvo)
        alpha_h = math.acos(max(-1, min(1, cos_alpha_h)))
        
        gamma = math.atan2(target_y, target_x)
        theta1_rad = gamma + alpha_h
        
        # --- ETAPA 2: Encontrar Ponto de Ancoragem (Haste Roxa) ---
        xk = L1 * math.cos(theta1_rad)
        yk = L1 * math.sin(theta1_rad)
        
        theta_tibia = math.atan2(target_y - yk, target_x - xk)
        
        xa = xk - L_EXT * math.cos(theta_tibia)
        ya = yk - L_EXT * math.sin(theta_tibia)
        
        # --- ETAPA 3: Segundo Mecanismo (Servo Joelho) ---
        dx = xa - OFFSET_X
        dy = ya - 0 
        dist_servo2_a = math.sqrt(dx**2 + dy**2)
        
        # Verifica se a haste consegue alcançar o ponto de ancoragem
        if dist_servo2_a > (L_HORN + L_HASTE) or dist_servo2_a < abs(L_HASTE - L_HORN):
             return None, None
        
        cos_alpha_s2 = (L_HORN**2 + dist_servo2_a**2 - L_HASTE**2) / (2 * L_HORN * dist_servo2_a)
        alpha_s2 = math.acos(max(-1, min(1, cos_alpha_s2)))
        
        gamma2 = math.atan2(dy, dx)
        theta2_rad = gamma2 - alpha_s2
        
        # Converter para graus
        theta1_deg = math.degrees(theta1_rad)
        theta2_deg = math.degrees(theta2_rad)
        
        return theta1_deg, theta2_deg

    except ValueError:
        return None, None

# ──────────────────────────────────────────────
# Loop Interativo de Controle
# ──────────────────────────────────────────────
print("--- Controle da Perna do Robô Quadrúpede ---")
print("Digite as coordenadas separadas por espaço (ex: 50 180)")
print("Para sair, digite 'q'")

while True:
    comando = input("\nAlvo (X Y): ")
    
    if comando.lower() == 'q':
        print("Encerrando programa...")
        break
        
    try:
        # Separa a string digitada em duas partes e converte para float
        x_str, y_str = comando.split()
        alvo_x = float(x_str)
        alvo_y = float(y_str)
        
        ang_ombro, ang_joelho = calcular_cinematica_inversa(alvo_x, alvo_y)
        
        # Se a matemática falhou (retornou None)
        if ang_ombro is None or ang_joelho is None:
            print(f"❌ Não dá! A posição ({alvo_x}, {alvo_y}) está fora do alcance físico da perna.")
        else:
            # Verifica se os ângulos estão dentro do limite mecânico do servo (0 a 180 graus)
            # Lembre-se de ajustar a lógica se os seus servos precisarem de offsets!
            if 0 <= ang_ombro <= 180 and 0 <= ang_joelho <= 180:
                servo_ombro.angle = ang_ombro
                servo_joelho.angle = ang_joelho
                print(f"✅ Movendo para ({alvo_x}, {alvo_y}) -> Ombro: {ang_ombro:.1f}°, Joelho: {ang_joelho:.1f}°")
            else:
                print(f"❌ Não dá! Os ângulos exigidos (Ombro: {ang_ombro:.1f}°, Joelho: {ang_joelho:.1f}°) ultrapassam o limite de 0° a 180° dos motores.")
                
    except ValueError:
        print("⚠️ Erro: Formato inválido. Digite dois números separados por espaço (ex: 40 160).")