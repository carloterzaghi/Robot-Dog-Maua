"""
Cinemática para uma perna com mecanismo de 4 barras (paralelogramo).

Mecanismo (conforme desenho medido pelo usuário):
  ┌─────────────────────────────────────────────────────────────────┐
  │  Servo 0 (ombro)  ──[barra superior 120mm]──  Pivô A (joelho) │
  │       ↕ ~39mm (chassis)                          ↕ 39mm       │
  │  Servo 4 (joelho) ──[barra inferior 120mm]──  Pivô B (joelho) │
  │                                                   ↓            │
  │                                              Tíbia 130mm       │
  │                                                   ↓            │
  │                                                  Pé (F)        │
  └─────────────────────────────────────────────────────────────────┘

Dimensões do desenho:
  - Barras: 120 mm × 2 (paralelas)
  - Coupler (joelho): 39 mm (distância entre pivôs A e B)
  - Tíbia: 130 mm (extensão rígida do coupler, de B até o pé)
  - Braços dos servos (horns): 25 mm
  - Distância entre eixos dos servos no chassis: ~39 mm

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  IMPORTANTE — LIMITES DE SEGURANÇA
  
  Este mecanismo é um PARALELOGRAMO RÍGIDO. Os dois servos
  NÃO são independentes — eles devem se mover de forma
  COORDENADA para não forçar as peças.
  
  Regra de segurança:
  • Quando os dois servos se movem igualmente em direções
    opostas (servo[0] diminui X° E servo[4] aumenta X°),
    a perna se move para frente/trás SEM FORÇAR o coupler.
  • Quando os servos se movem na MESMA direção, o coupler
    é forçado → RISCO DE QUEBRA!
  
  Limites conservadores definidos abaixo para proteger a peça.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Calibração:
  - Neutro: servo[0] = 50°, servo[4] = 90°
  - Nessa posição: barras paralelas apontando para baixo
"""

import numpy as np
from math import acos, atan2, pi, cos, sin, sqrt
from adafruit_servokit import ServoKit

# ──────────────────────────────────────────────
# Configuração dos servos
# ──────────────────────────────────────────────
kit = ServoKit(channels=16)
servo_ombro  = kit.servo[0]   # Servo 0: ombro (barra superior)
servo_joelho = kit.servo[4]   # Servo 4: joelho (barra inferior)

# ──────────────────────────────────────────────
# Parâmetros físicos (do desenho, em metros)
# ──────────────────────────────────────────────
L_BAR     = 0.120   # comprimento de cada barra [m] (120mm)
L_COUPLE  = 0.039   # coupler entre pivôs do joelho [m] (39mm)
L_TIBIA   = 0.130   # tíbia (extensão rígida do coupler) [m] (130mm)
D_CHASSIS = 0.039   # distância entre pivôs dos servos no chassis [m] (39mm)

# ──────────────────────────────────────────────
# Calibração — Posição neutra
# ──────────────────────────────────────────────
# Quando servo[0]=50° e servo[4]=90°, ambas as barras
# apontam reto para baixo (α=0°, β=0°).
SERVO0_NEUTRO = 50.0
SERVO4_NEUTRO = 90.0

# ──────────────────────────────────────────────
# Limites de segurança dos servos
# ──────────────────────────────────────────────
# CRÍTICO: limitar a diferença |Δα - Δβ| para não forçar o coupler.
# No paralelogramo perfeito, α deve ser sempre == β.
# Na prática, permitimos uma pequena diferença para compensação do IMU.
SERVO0_MIN = 25.0
SERVO0_MAX = 75.0
SERVO4_MIN = 65.0
SERVO4_MAX = 115.0

# Máxima diferença permitida entre os ângulos dos servos
# medidos a partir dos respectivos neutros.
# delta0 = -(servo[0] - 50), delta4 = -(servo[4] - 90)
# |delta0 - delta4| <= MAX_DIFF_DEG
MAX_DIFF_DEG = 10.0   # graus — conservador para não forçar


# ──────────────────────────────────────────────
# Cinemática Direta (FK)
# ──────────────────────────────────────────────
def forward_kinematics(servo0_deg: float, servo4_deg: float):
    """
    Calcula a posição do pé (x, z) a partir dos ângulos dos servos.

    Coordenadas:
      - Origem: pivô do servo 0
      - x positivo = frente do robô
      - z negativo = abaixo do quadril

    Retorna: (x_foot, z_foot) em metros
    """
    # Ângulos geométricos (0° = barra apontando reto para baixo)
    alpha = np.radians(-(servo0_deg - SERVO0_NEUTRO))
    beta  = np.radians(-(servo4_deg - SERVO4_NEUTRO))

    # Ponto A (ponta da barra superior, saindo do servo 0)
    ax = L_BAR * sin(alpha)
    az = -L_BAR * cos(alpha)

    # Ponto B (ponta da barra inferior, saindo do servo 4)
    bx = L_BAR * sin(beta)
    bz = -D_CHASSIS - L_BAR * cos(beta)

    # Direção do coupler (A → B)
    dab = sqrt((bx - ax)**2 + (bz - az)**2)
    if dab > 1e-6:
        dx = (bx - ax) / dab
        dz = (bz - az) / dab
    else:
        dx, dz = 0.0, -1.0

    # Pé = B + tíbia na direção do coupler
    fx = bx + L_TIBIA * dx
    fz = bz + L_TIBIA * dz

    return fx, fz


# ──────────────────────────────────────────────
# Cinemática Inversa (IK) — Segura
# ──────────────────────────────────────────────
def inverse_kinematics(x_foot: float, z_foot: float):
    """
    Calcula os ângulos dos servos para posicionar o pé em (x_foot, z_foot).

    IMPORTANTE: No paralelogramo, o espaço de trabalho do pé é
    essencialmente um ARCO (não uma área 2D completa).
    O pé se move ao longo de um arco circular de raio L_BAR,
    centrado ~39+130 = 169mm abaixo do servo, quando α=β.

    Para movimentos fora desse arco (mudando a altura/joelho),
    é necessário que α ≠ β, o que DEFORMA o paralelogramo.
    Limitamos essa deformação a MAX_DIFF_DEG para segurança.

    Estratégia de solução:
    1. Calcular o ângulo "médio" (swing) que posiciona o pé
       horizontalmente (x).
    2. Calcular a diferença necessária (splay) que ajusta a
       altura (z).
    3. Limitar o splay a MAX_DIFF_DEG para segurança.

    Parâmetros
    ----------
    x_foot : float  — posição horizontal do pé [m]
    z_foot : float  — posição vertical do pé [m] (negativo = abaixo)

    Retorna
    -------
    (servo0_deg, servo4_deg) : ângulos dos servos [graus]
    """
    # ── Distância vertical efetiva do coupler + tíbia ──
    # No paralelogramo perfeito (α=β), o coupler + tíbia formam
    # uma extensão vertical de (D_CHASSIS + L_TIBIA) abaixo do
    # ponto A. Mas na verdade, a tíbia é extensão do coupler (A→B→F).
    # Com α=β: coupler aponta reto para baixo (39mm), tíbia continua (130mm).
    # O pé fica em: (L_BAR*sin(α), -(L_BAR*cos(α) + D_CHASSIS + L_TIBIA))

    # Offset vertical fixo (coupler + tíbia quando α=β)
    z_offset = -(D_CHASSIS + L_TIBIA)   # = -169mm

    # ── Passo 1: Ângulo de swing (α = β) para posição horizontal ──
    # x_foot ≈ L_BAR * sin(α)  quando α=β
    # Resolver: sin(α) = x_foot / L_BAR
    sin_alpha = x_foot / L_BAR
    sin_alpha = max(-1.0, min(1.0, sin_alpha))
    swing_rad = np.arcsin(sin_alpha)
    swing_deg = np.degrees(swing_rad)

    # ── Passo 2: Verificar altura ──
    # Com α=β=swing, z_foot ≈ -(L_BAR*cos(swing) + D_CHASSIS + L_TIBIA)
    z_nominal = -(L_BAR * cos(swing_rad) + D_CHASSIS + L_TIBIA)

    # Diferença de altura que precisa ser compensada
    dz = z_foot - z_nominal   # positivo = precisa subir (agachar mais)

    # ── Passo 3: Splay (diferença α-β) para ajustar altura ──
    # Quando α > β, o coupler inclina e o pé sobe.
    # Quando α < β, o coupler inclina e o pé desce.
    # A relação é aproximadamente linear para pequenas diferenças.
    #
    # Sensibilidade (derivada numérica):
    # dz/d(splay) ≈ calculada a partir do mecanismo
    #
    # Para simplificar e SEGURANÇA, limitamos o splay:
    if abs(dz) > 0.001:  # mais de 1mm de diferença
        # Calcular splay necessário numericamente
        # Teste: aumentar splay de 1° e ver o efeito
        s0_test = SERVO0_NEUTRO - (swing_deg + 0.5)
        s4_test = SERVO4_NEUTRO - (swing_deg - 0.5)
        _, z_test = forward_kinematics(s0_test, s4_test)
        dz_per_splay = (z_test - z_nominal)  # por 1° de splay total
        
        if abs(dz_per_splay) > 1e-6:
            splay_needed = dz / dz_per_splay
        else:
            splay_needed = 0.0

        # LIMITAR para segurança
        splay = max(-MAX_DIFF_DEG, min(MAX_DIFF_DEG, splay_needed))
        
        if abs(splay_needed) > MAX_DIFF_DEG:
            print(f"[AVISO] Ajuste de altura limitado: necessário {splay_needed:.1f}° "
                  f"de splay, limitado a ±{MAX_DIFF_DEG}°")
    else:
        splay = 0.0

    # ── Passo 4: Converter para ângulos dos servos ──
    alpha_deg = swing_deg + splay / 2
    beta_deg  = swing_deg - splay / 2

    servo0 = SERVO0_NEUTRO - alpha_deg
    servo4 = SERVO4_NEUTRO - beta_deg

    # Clamping final com aviso
    if servo0 < SERVO0_MIN or servo0 > SERVO0_MAX:
        print(f"[AVISO] servo[0]={servo0:.1f}° fora dos limites "
              f"[{SERVO0_MIN}°, {SERVO0_MAX}°]. Limitando.")
        servo0 = max(SERVO0_MIN, min(SERVO0_MAX, servo0))

    if servo4 < SERVO4_MIN or servo4 > SERVO4_MAX:
        print(f"[AVISO] servo[4]={servo4:.1f}° fora dos limites "
              f"[{SERVO4_MIN}°, {SERVO4_MAX}°]. Limitando.")
        servo4 = max(SERVO4_MIN, min(SERVO4_MAX, servo4))

    return servo0, servo4


# ──────────────────────────────────────────────
# Funções de conveniência
# ──────────────────────────────────────────────
def move_foot_to(x: float, z: float, verbose: bool = True):
    """Move o pé para a posição (x, z) via cinemática inversa."""
    s0, s4 = inverse_kinematics(x, z)

    if verbose:
        x_check, z_check = forward_kinematics(s0, s4)
        err = sqrt((x_check - x)**2 + (z_check - z)**2)
        print(f"Alvo:     ({x*1000:+.1f}, {z*1000:+.1f}) mm")
        print(f"Servos:   s0={s0:.1f}°, s4={s4:.1f}°")
        print(f"FK real:  ({x_check*1000:+.1f}, {z_check*1000:+.1f}) mm")
        print(f"Erro:     {err*1000:.1f} mm")
        print("-" * 50)

    servo_ombro.angle  = s0
    servo_joelho.angle = s4


def posicao_neutra():
    """Move para a posição neutra (servo[0]=50°, servo[4]=90°)."""
    print(f">> Neutro: servo[0]={SERVO0_NEUTRO}°, servo[4]={SERVO4_NEUTRO}°")
    servo_ombro.angle  = SERVO0_NEUTRO
    servo_joelho.angle = SERVO4_NEUTRO
    x, z = forward_kinematics(SERVO0_NEUTRO, SERVO4_NEUTRO)
    print(f"   Pé em: ({x*1000:+.1f}, {z*1000:+.1f}) mm")
    print("-" * 50)


def swing(delta_deg: float):
    """
    Move a perna para frente/trás SEM alterar a altura.
    Ambos os servos se movem igualmente (paralelogramo puro).

    delta_deg > 0 → perna vai para FRENTE
    delta_deg < 0 → perna vai para TRÁS
    """
    s0 = SERVO0_NEUTRO - delta_deg
    s4 = SERVO4_NEUTRO - delta_deg

    s0 = max(SERVO0_MIN, min(SERVO0_MAX, s0))
    s4 = max(SERVO4_MIN, min(SERVO4_MAX, s4))

    servo_ombro.angle = s0
    servo_joelho.angle = s4

    x, z = forward_kinematics(s0, s4)
    return x, z


def set_servos_safe(servo0_deg: float, servo4_deg: float):
    """
    Envia ângulos para os servos COM verificação de segurança.
    Verifica que a diferença não force o coupler.
    """
    delta0 = -(servo0_deg - SERVO0_NEUTRO)
    delta4 = -(servo4_deg - SERVO4_NEUTRO)
    diff = abs(delta0 - delta4)

    if diff > MAX_DIFF_DEG:
        print(f"[BLOQUEADO] Diferença de {diff:.1f}° entre servos excede "
              f"o limite de {MAX_DIFF_DEG}°!")
        print(f"  Isso FORÇARIA o coupler e pode quebrar a peça.")
        print(f"  Use swing() para movimentos seguros.")
        return False

    servo0_deg = max(SERVO0_MIN, min(SERVO0_MAX, servo0_deg))
    servo4_deg = max(SERVO4_MIN, min(SERVO4_MAX, servo4_deg))

    servo_ombro.angle  = servo0_deg
    servo_joelho.angle = servo4_deg
    return True


# ──────────────────────────────────────────────
# Exemplo de uso
# ──────────────────────────────────────────────
if __name__ == "__main__":
    import time

    print("=== Cinemática — Mecanismo 4 Barras (Paralelogramo) ===\n")
    print(f"Barras:    {L_BAR*1000:.0f} mm (×2)")
    print(f"Coupler:   {L_COUPLE*1000:.0f} mm")
    print(f"Tíbia:     {L_TIBIA*1000:.0f} mm")
    print(f"Chassis:   {D_CHASSIS*1000:.0f} mm")
    print(f"Max swing: ±{SERVO0_NEUTRO - SERVO0_MIN:.0f}°")
    print(f"Max splay: ±{MAX_DIFF_DEG:.0f}° (proteção do coupler)\n")

    # Posição neutra
    posicao_neutra()
    time.sleep(2)

    # Swing para frente (15°)
    print(">> Swing +15° (frente)")
    x, z = swing(15)
    print(f"   Pé em: ({x*1000:+.1f}, {z*1000:+.1f}) mm")
    time.sleep(2)

    # Swing para trás (-15°)
    print(">> Swing -15° (trás)")
    x, z = swing(-15)
    print(f"   Pé em: ({x*1000:+.1f}, {z*1000:+.1f}) mm")
    time.sleep(2)

    # IK: pé à frente
    print(">> IK: pé 40mm à frente")
    move_foot_to(x=0.04, z=-0.28)
    time.sleep(2)

    # IK: pé atrás
    print(">> IK: pé 40mm atrás")
    move_foot_to(x=-0.04, z=-0.28)
    time.sleep(2)

    # Voltar ao neutro
    posicao_neutra()