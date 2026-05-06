"""
teste_perna.py — Teste interativo da perna com cinemática inversa (v2).

Mecanismo de 4 barras (paralelogramo trapezoidal):
  - Servo 0 (ombro): barra superior 120mm
  - Servo 4 (joelho): barra inferior 120mm
  - Coupler: 39mm entre pivôs do joelho
  - Tíbia: 130mm (extensão rígida do coupler)
  - Distância entre eixos no chassis: 39mm

Neutro: servo[0] = 90°, servo[4] = 90°
  → barras apontando reto para baixo.

Limites de segurança obtidos empiricamente (planilha Excel):
  Para cada ângulo do servo 0, existe um range seguro para o servo 4
  que não quebra a perna. Os limites são interpolados linearmente.
"""

from adafruit_servokit import ServoKit
from math import cos, sin, sqrt, pi, acos, atan2
import numpy as np

# ──────────────────────────────────────────────
# Configuração dos servos
# ──────────────────────────────────────────────
kit = ServoKit(channels=16)
servo_ombro  = kit.servo[3]   # Servo 0: ombro (barra superior)
servo_joelho = kit.servo[7]   # Servo 4: joelho (barra inferior)

# ──────────────────────────────────────────────
# Parâmetros físicos (em mm)
# ──────────────────────────────────────────────
L_BAR     = 120.0   # comprimento de cada barra [mm]
L_COUPLE  = 39.0    # coupler entre pivôs do joelho [mm]
L_TIBIA   = 130.0   # tíbia (extensão rígida do coupler) [mm]
D_CHASSIS = 39.0    # distância entre eixos dos servos no chassis [mm]

# ──────────────────────────────────────────────
# Calibração — Posição neutra
# ──────────────────────────────────────────────
SERVO0_NEUTRO = 90.0   # servo 0 neutro [graus]
SERVO4_NEUTRO = 90.0   # servo 4 neutro [graus]

# ──────────────────────────────────────────────
# Limites de segurança (dados do Excel)
# ──────────────────────────────────────────────
# Tabela: [servo0_graus, servo4_min, servo4_max]
# Para cada ângulo do servo 0, o servo 4 pode ir de min até max
# sem quebrar a perna.
LIMITES_TABLE = np.array([
    # servo0,  s4_min,  s4_max
    [  45.0,    10.0,   100.0],
    [  60.0,    10.0,   115.0],
    [  75.0,    10.0,   130.0],
    [  90.0,    15.0,   140.0],
    [ 105.0,    30.0,   155.0],
    [ 120.0,    45.0,   170.0],
    [ 135.0,    70.0,   170.0],
])

# Faixa global dos servos
SERVO0_MIN = float(LIMITES_TABLE[:, 0].min())  # 45°
SERVO0_MAX = float(LIMITES_TABLE[:, 0].max())  # 135°


def servo4_min(servo0_deg: float) -> float:
    """Retorna o ângulo MÍNIMO seguro do servo 4 para dado servo 0 (interpolação linear)."""
    return float(np.interp(servo0_deg, LIMITES_TABLE[:, 0], LIMITES_TABLE[:, 1]))


def servo4_max(servo0_deg: float) -> float:
    """Retorna o ângulo MÁXIMO seguro do servo 4 para dado servo 0 (interpolação linear)."""
    return float(np.interp(servo0_deg, LIMITES_TABLE[:, 0], LIMITES_TABLE[:, 2]))


def clamp_servo4(servo0_deg: float, servo4_deg: float) -> float:
    """Limita o servo 4 ao range seguro para o servo 0 atual."""
    s4_lo = servo4_min(servo0_deg)
    s4_hi = servo4_max(servo0_deg)
    return max(s4_lo, min(s4_hi, servo4_deg))


def is_safe(servo0_deg: float, servo4_deg: float) -> bool:
    """Verifica se a combinação (servo0, servo4) está dentro dos limites seguros."""
    if servo0_deg < SERVO0_MIN or servo0_deg > SERVO0_MAX:
        return False
    return servo4_min(servo0_deg) <= servo4_deg <= servo4_max(servo0_deg)


# ──────────────────────────────────────────────
# Cinemática Direta (FK)
# ──────────────────────────────────────────────
def forward_kinematics(servo0_deg: float, servo4_deg: float):
    """
    Calcula a posição do pé (x, z) em mm a partir dos ângulos dos servos.

    Coordenadas:
      - Origem: pivô do servo 0
      - x positivo = frente do robô
      - z negativo = abaixo do quadril

    Retorna: (x_foot, z_foot) em mm
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
# Cinemática Inversa (IK) — Com limites do Excel
# ──────────────────────────────────────────────
def inverse_kinematics(x_foot: float, z_foot: float):
    """
    Calcula os ângulos dos servos para posicionar o pé em (x_foot, z_foot).
    Usa busca numérica dentro dos limites seguros da tabela Excel.

    Parâmetros
    ----------
    x_foot : float  — posição horizontal do pé [mm]
    z_foot : float  — posição vertical do pé [mm] (negativo = abaixo)

    Retorna
    -------
    (servo0_deg, servo4_deg) : ângulos dos servos [graus]
    """
    best_s0 = SERVO0_NEUTRO
    best_s4 = SERVO4_NEUTRO
    best_err = float('inf')

    # Varredura grosseira: passo de 1°
    for s0 in np.arange(SERVO0_MIN, SERVO0_MAX + 0.5, 1.0):
        s4_lo = servo4_min(s0)
        s4_hi = servo4_max(s0)
        for s4 in np.arange(s4_lo, s4_hi + 0.5, 1.0):
            fx, fz = forward_kinematics(s0, s4)
            err = (fx - x_foot)**2 + (fz - z_foot)**2
            if err < best_err:
                best_err = err
                best_s0 = s0
                best_s4 = s4

    # Refinamento fino: passo de 0.1° ao redor da melhor solução
    s0_lo_r = max(SERVO0_MIN, best_s0 - 2.0)
    s0_hi_r = min(SERVO0_MAX, best_s0 + 2.0)
    for s0 in np.arange(s0_lo_r, s0_hi_r + 0.05, 0.1):
        s4_lo = servo4_min(s0)
        s4_hi = servo4_max(s0)
        s4_lo_r = max(s4_lo, best_s4 - 2.0)
        s4_hi_r = min(s4_hi, best_s4 + 2.0)
        for s4 in np.arange(s4_lo_r, s4_hi_r + 0.05, 0.1):
            fx, fz = forward_kinematics(s0, s4)
            err = (fx - x_foot)**2 + (fz - z_foot)**2
            if err < best_err:
                best_err = err
                best_s0 = s0
                best_s4 = s4

    # Arredonda para 1 casa decimal (resolução prática do servo)
    best_s0 = round(float(best_s0), 1)
    best_s4 = round(float(best_s4), 1)

    # Garante limites após arredondamento
    best_s0 = max(SERVO0_MIN, min(SERVO0_MAX, best_s0))
    best_s4 = clamp_servo4(best_s0, best_s4)

    return best_s0, best_s4


# ──────────────────────────────────────────────
# Funções de conveniência
# ──────────────────────────────────────────────
# Tolerância máxima de erro entre posição desejada e atingível (mm)
MAX_IK_ERROR_MM = 5.0


def move_foot_to(x: float, z: float, verbose: bool = True):
    """
    Move o pé para a posição (x, z) em mm via cinemática inversa.
    BLOQUEIA o movimento se a posição estiver fora do espaço de trabalho seguro.
    """
    s0, s4 = inverse_kinematics(x, z)
    x_check, z_check = forward_kinematics(s0, s4)
    err = sqrt((x_check - x)**2 + (z_check - z)**2)

    if err > MAX_IK_ERROR_MM:
        print(f"[BLOQUEADO] Posição ({x:+.1f}, {z:+.1f}) mm INALCANÇÁVEL dentro dos limites seguros!")
        print(f"  Melhor aproximação: ({x_check:+.1f}, {z_check:+.1f}) mm")
        print(f"  Erro: {err:.1f} mm (máximo permitido: {MAX_IK_ERROR_MM} mm)")
        print(f"  Servos necessários: s0={s0:.1f}°, s4={s4:.1f}°")
        print(f"  Movimento NÃO executado para proteger a estrutura.")
        print("-" * 50)
        return False

    if verbose:
        s4_range = f"[{servo4_min(s0):.0f}°, {servo4_max(s0):.0f}°]"
        print(f"Alvo:      ({x:+.1f}, {z:+.1f}) mm")
        print(f"Servos:    s0={s0:.1f}°, s4={s4:.1f}°  ✓ SEGURO")
        print(f"Range s4:  {s4_range}")
        print(f"FK real:   ({x_check:+.1f}, {z_check:+.1f}) mm")
        print(f"Erro:      {err:.1f} mm")
        print("-" * 50)

    servo_ombro.angle  = s0
    servo_joelho.angle = s4
    return True


def posicao_neutra():
    """Move para a posição neutra (servo[0]=90°, servo[4]=90°)."""
    print(f">> Neutro: servo[0]={SERVO0_NEUTRO}°, servo[4]={SERVO4_NEUTRO}°")
    servo_ombro.angle  = SERVO0_NEUTRO
    servo_joelho.angle = SERVO4_NEUTRO
    x, z = forward_kinematics(SERVO0_NEUTRO, SERVO4_NEUTRO)
    print(f"   Pé em: ({x:+.1f}, {z:+.1f}) mm")
    print("-" * 50)


def move_servos_safe(servo0_deg: float, servo4_deg: float, verbose: bool = True):
    """
    Move os servos COM verificação de limites do Excel.
    BLOQUEIA o movimento se a combinação estiver fora dos limites seguros.
    NÃO ajusta/limita — simplesmente RECUSA o comando inseguro.
    """
    # Verifica servo 0
    if servo0_deg < SERVO0_MIN or servo0_deg > SERVO0_MAX:
        print(f"[BLOQUEADO] servo[0]={servo0_deg:.1f}° está FORA do range permitido "
              f"[{SERVO0_MIN:.0f}°, {SERVO0_MAX:.0f}°]!")
        print(f"  Movimento NÃO executado para proteger a estrutura.")
        return False

    # Verifica servo 4 dentro dos limites para este servo 0
    s4_lo = servo4_min(servo0_deg)
    s4_hi = servo4_max(servo0_deg)
    if servo4_deg < s4_lo or servo4_deg > s4_hi:
        print(f"[BLOQUEADO] servo[4]={servo4_deg:.1f}° está FORA do range seguro "
              f"para s0={servo0_deg:.0f}°!")
        print(f"  Range permitido do servo 4: [{s4_lo:.0f}°, {s4_hi:.0f}°]")
        print(f"  Movimento NÃO executado para proteger a estrutura.")
        return False

    # Tudo seguro — executa
    servo_ombro.angle = servo0_deg
    servo_joelho.angle = servo4_deg

    if verbose:
        x, z = forward_kinematics(servo0_deg, servo4_deg)
        print(f"Servos:  s0={servo0_deg:.1f}°, s4={servo4_deg:.1f}°  ✓ SEGURO")
        print(f"Pé em:   ({x:+.1f}, {z:+.1f}) mm")
        print("-" * 50)
    return True


# ──────────────────────────────────────────────
# Simulação de caminhada (gait)
# ──────────────────────────────────────────────
def gait_walk(ciclos: int = 3, stride_mm: float = 40.0, lift_mm: float = 30.0,
              pontos_stance: int = 10, pontos_swing: int = 10, delay: float = 0.05):
    """
    Simula o movimento de caminhada de uma pata de cachorro.

    A trajetória do pé é uma elipse achatada:
      - Fase de apoio (stance): pé no chão, empurrando para trás (reta horizontal)
      - Fase de balanço (swing): pé levantado, avançando (arco semi-elíptico)

    Parâmetros
    ----------
    ciclos       : int   — número de ciclos completos de caminhada
    stride_mm    : float — comprimento total do passo [mm] (frente + trás)
    lift_mm      : float — altura que o pé levanta na fase de balanço [mm]
    pontos_stance: int   — número de waypoints na fase de apoio
    pontos_swing : int   — número de waypoints na fase de balanço
    delay        : float — tempo entre waypoints [segundos]
    """
    import time

    # Posição neutra do pé (referência)
    x_neutro, z_neutro = forward_kinematics(SERVO0_NEUTRO, SERVO4_NEUTRO)

    half_stride = stride_mm / 2.0

    # ── Gerar trajetória completa de 1 ciclo ──
    waypoints = []

    # Fase de APOIO (stance): pé no chão, move de frente → trás
    for i in range(pontos_stance):
        t = i / pontos_stance
        x = x_neutro + half_stride - (stride_mm * t)
        z = z_neutro
        waypoints.append((x, z, "apoio"))

    # Fase de BALANÇO (swing): pé levantado, move de trás → frente
    for i in range(pontos_swing):
        t = i / pontos_swing
        angle = pi * t
        x = x_neutro - half_stride + (stride_mm * t)
        z = z_neutro + lift_mm * sin(angle)  # levanta (z fica menos negativo)
        waypoints.append((x, z, "balanço"))

    # ── Validar TODOS os waypoints antes de executar ──
    print("\n" + "=" * 55)
    print("  Simulação de Caminhada — Validando trajetória...")
    print("=" * 55)
    print(f"Pé neutro:     ({x_neutro:+.1f}, {z_neutro:+.1f}) mm")
    print(f"Stride:        {stride_mm:.0f} mm")
    print(f"Lift:          {lift_mm:.0f} mm")
    print(f"Ciclos:        {ciclos}")
    print(f"Waypoints:     {len(waypoints)} por ciclo ({pontos_stance} apoio + {pontos_swing} balanço)")
    print(f"Delay:         {delay*1000:.0f} ms entre pontos")
    print()

    all_safe = True
    servos_trajectory = []

    for i, (x, z, fase) in enumerate(waypoints):
        s0, s4 = inverse_kinematics(x, z)
        x_check, z_check = forward_kinematics(s0, s4)
        err = sqrt((x_check - x)**2 + (z_check - z)**2)

        if err > MAX_IK_ERROR_MM:
            print(f"  [FALHA] Ponto {i+1}/{len(waypoints)} ({fase}): "
                  f"({x:+.1f}, {z:+.1f}) mm → erro {err:.1f} mm")
            all_safe = False
        elif not is_safe(s0, s4):
            print(f"  [FALHA] Ponto {i+1}/{len(waypoints)} ({fase}): "
                  f"s0={s0:.1f}°, s4={s4:.1f}° fora dos limites")
            all_safe = False
        else:
            servos_trajectory.append((s0, s4, x_check, z_check, fase))

    if not all_safe:
        print(f"\n[BLOQUEADO] Trajetória contém pontos inseguros!")
        print(f"  Tente reduzir stride ({stride_mm}mm) ou lift ({lift_mm}mm).")
        print(f"  Movimento NÃO executado para proteger a estrutura.")
        print("-" * 55)
        return False

    print(f"  ✓ Todos os {len(waypoints)} waypoints são SEGUROS!")
    print()

    # ── Mostrar trajetória resumida ──
    print("Trajetória (x, z) → (s0, s4):")
    step = max(1, len(servos_trajectory) // 6)
    for i in range(0, len(servos_trajectory), step):
        s0, s4, xc, zc, fase = servos_trajectory[i]
        print(f"  [{fase:7s}] ({xc:+6.1f}, {zc:+7.1f}) mm → s0={s0:5.1f}°, s4={s4:5.1f}°")
    print()

    # ── Executar ciclos ──
    print(f"Executando {ciclos} ciclos... (Ctrl+C para parar)")
    print("-" * 55)

    try:
        for ciclo in range(ciclos):
            print(f"\n── Ciclo {ciclo+1}/{ciclos} ──")
            for i, (s0, s4, xc, zc, fase) in enumerate(servos_trajectory):
                servo_ombro.angle = s0
                servo_joelho.angle = s4
                print(f"  {fase:7s} [{i+1:2d}/{len(servos_trajectory)}] "
                      f"pé=({xc:+6.1f}, {zc:+7.1f}) mm  "
                      f"s0={s0:5.1f}° s4={s4:5.1f}°", end="\r")
                time.sleep(delay)
            print()
    except KeyboardInterrupt:
        print("\n\n[INTERROMPIDO] Parando caminhada...")

    # Volta ao neutro
    print("Voltando ao neutro...")
    posicao_neutra()
    print(f"Caminhada concluída!")
    print("=" * 55)
    return True


def gait_circle(ciclos: int = 3, raio_mm: float = 25.0, pontos: int = 20, delay: float = 0.05):
    """
    Move o pé em uma trajetória circular ao redor da posição neutra.

    Útil para teste de workspace e para simular movimento de rotação.

    Parâmetros
    ----------
    ciclos   : int   — número de voltas completas
    raio_mm  : float — raio do círculo [mm]
    pontos   : int   — número de waypoints por volta
    delay    : float — tempo entre waypoints [segundos]
    """
    import time

    x_neutro, z_neutro = forward_kinematics(SERVO0_NEUTRO, SERVO4_NEUTRO)

    # ── Gerar waypoints do círculo ──
    waypoints = []
    for i in range(pontos):
        angle = 2 * pi * i / pontos
        x = x_neutro + raio_mm * cos(angle)
        z = z_neutro + raio_mm * sin(angle)
        waypoints.append((x, z))

    # ── Validar todos ──
    print("\n" + "=" * 55)
    print("  Movimento Circular — Validando trajetória...")
    print("=" * 55)
    print(f"Centro:    ({x_neutro:+.1f}, {z_neutro:+.1f}) mm")
    print(f"Raio:      {raio_mm:.0f} mm")
    print(f"Ciclos:    {ciclos}")
    print(f"Pontos:    {pontos} por volta")
    print(f"Delay:     {delay*1000:.0f} ms")
    print()

    all_safe = True
    servos_trajectory = []

    for i, (x, z) in enumerate(waypoints):
        s0, s4 = inverse_kinematics(x, z)
        x_check, z_check = forward_kinematics(s0, s4)
        err = sqrt((x_check - x)**2 + (z_check - z)**2)

        if err > MAX_IK_ERROR_MM:
            angle_deg = 360.0 * i / pontos
            print(f"  [FALHA] Ponto {i+1}/{pontos} ({angle_deg:.0f}°): "
                  f"({x:+.1f}, {z:+.1f}) mm → erro {err:.1f} mm")
            all_safe = False
        elif not is_safe(s0, s4):
            angle_deg = 360.0 * i / pontos
            print(f"  [FALHA] Ponto {i+1}/{pontos} ({angle_deg:.0f}°): "
                  f"s0={s0:.1f}°, s4={s4:.1f}° fora dos limites")
            all_safe = False
        else:
            servos_trajectory.append((s0, s4, x_check, z_check))

    if not all_safe:
        print(f"\n[BLOQUEADO] Trajetória circular contém pontos inseguros!")
        print(f"  Tente reduzir o raio ({raio_mm}mm).")
        print(f"  Movimento NÃO executado para proteger a estrutura.")
        print("-" * 55)
        return False

    print(f"  ✓ Todos os {pontos} waypoints são SEGUROS!")
    print()

    # ── Trajetória resumida ──
    print("Trajetória circular:")
    step = max(1, len(servos_trajectory) // 8)
    for i in range(0, len(servos_trajectory), step):
        s0, s4, xc, zc = servos_trajectory[i]
        angle_deg = 360.0 * i / pontos
        print(f"  [{angle_deg:5.0f}°] ({xc:+6.1f}, {zc:+7.1f}) mm → s0={s0:5.1f}°, s4={s4:5.1f}°")
    print()

    # ── Executar ──
    print(f"Executando {ciclos} voltas... (Ctrl+C para parar)")
    print("-" * 55)

    try:
        for ciclo in range(ciclos):
            print(f"\n── Volta {ciclo+1}/{ciclos} ──")
            for i, (s0, s4, xc, zc) in enumerate(servos_trajectory):
                servo_ombro.angle = s0
                servo_joelho.angle = s4
                angle_deg = 360.0 * i / pontos
                print(f"  {angle_deg:5.0f}° [{i+1:2d}/{len(servos_trajectory)}] "
                      f"pé=({xc:+6.1f}, {zc:+7.1f}) mm  "
                      f"s0={s0:5.1f}° s4={s4:5.1f}°", end="\r")
                time.sleep(delay)
            print()
    except KeyboardInterrupt:
        print("\n\n[INTERROMPIDO] Parando movimento circular...")

    print("Voltando ao neutro...")
    posicao_neutra()
    print("Movimento circular concluído!")
    print("=" * 55)
    return True


def find_max_stride():
    """
    Encontra o stride máximo seguro para caminhada e o raio máximo para círculo.
    Testa incrementalmente até encontrar o limite.
    """
    x_neutro, z_neutro = forward_kinematics(SERVO0_NEUTRO, SERVO4_NEUTRO)

    print("\n" + "=" * 55)
    print("  Buscando limites máximos seguros...")
    print("=" * 55)
    print(f"Pé neutro: ({x_neutro:+.1f}, {z_neutro:+.1f}) mm")
    print()

    # ── Stride máximo (caminhada) ──
    max_stride = 0
    for stride in range(5, 200, 5):
        half = stride / 2.0
        safe = True
        # Testa pontos extremos + intermediários
        for t in np.linspace(0, 1, 20):
            # Stance
            x = x_neutro + half - stride * t
            s0, s4 = inverse_kinematics(x, z_neutro)
            xc, zc = forward_kinematics(s0, s4)
            if sqrt((xc - x)**2 + (zc - z_neutro)**2) > MAX_IK_ERROR_MM:
                safe = False
                break
            # Swing (com lift de 30mm)
            angle = pi * t
            x_sw = x_neutro - half + stride * t
            z_sw = z_neutro + 30.0 * sin(angle)
            s0, s4 = inverse_kinematics(x_sw, z_sw)
            xc, zc = forward_kinematics(s0, s4)
            if sqrt((xc - x_sw)**2 + (zc - z_sw)**2) > MAX_IK_ERROR_MM:
                safe = False
                break
        if safe:
            max_stride = stride
        else:
            break

    print(f"  Stride máximo (caminhada, lift=30mm): ~{max_stride} mm")

    # ── Raio máximo (círculo) ──
    max_raio = 0
    for raio in range(5, 100, 5):
        safe = True
        for i in range(20):
            angle = 2 * pi * i / 20
            x = x_neutro + raio * cos(angle)
            z = z_neutro + raio * sin(angle)
            s0, s4 = inverse_kinematics(x, z)
            xc, zc = forward_kinematics(s0, s4)
            if sqrt((xc - x)**2 + (zc - z)**2) > MAX_IK_ERROR_MM:
                safe = False
                break
        if safe:
            max_raio = raio
        else:
            break

    print(f"  Raio máximo (círculo):                ~{max_raio} mm")
    print()
    print(f"Sugestões:")
    print(f"  w 3 {max_stride} 30 50     — caminhada no stride máximo")
    print(f"  c 3 {max_raio} 20 50       — círculo no raio máximo")
    print("=" * 55)


# ──────────────────────────────────────────────
# Programa principal — modo interativo
# ──────────────────────────────────────────────
if __name__ == "__main__":
    import time

    print("=" * 55)
    print("  Teste de Perna — Mecanismo 4 Barras (v2)")
    print("=" * 55)
    print(f"Barras:    {L_BAR:.0f} mm (×2)")
    print(f"Coupler:   {L_COUPLE:.0f} mm")
    print(f"Tíbia:     {L_TIBIA:.0f} mm")
    print(f"Chassis:   {D_CHASSIS:.0f} mm")
    print(f"Neutro:    s0={SERVO0_NEUTRO:.0f}°, s4={SERVO4_NEUTRO:.0f}°")
    print(f"Range s0:  [{SERVO0_MIN:.0f}°, {SERVO0_MAX:.0f}°]")
    print()
    print("Limites de segurança do servo 4 (por ângulo do servo 0):")
    for row in LIMITES_TABLE:
        print(f"  s0={row[0]:5.0f}°  →  s4 ∈ [{row[1]:5.0f}°, {row[2]:5.0f}°]")
    print()
    print("Modos:")
    print("  1) Cinemática inversa  — digite: x z  (pé em mm)")
    print("  2) Controle direto     — digite: s s0 s4  (graus)")
    print("  3) Caminhada           — digite: w [ciclos] [stride] [lift] [delay_ms]")
    print("  4) Círculo             — digite: c [ciclos] [raio] [pontos] [delay_ms]")
    print("  5) Limites máximos     — digite: max")
    print("  6) Neutro              — digite: n")
    print("  7) Sair                — digite: q")
    print("-" * 55)

    # Inicia na posição neutra
    posicao_neutra()
    time.sleep(1)

    while True:
        entrada = input("\n>> ").strip().lower()

        if entrada == "q":
            print("Voltando ao neutro e encerrando...")
            posicao_neutra()
            break

        if entrada == "n":
            posicao_neutra()
            continue

        partes = entrada.split()

        # Modo caminhada: w [ciclos] [stride_mm] [lift_mm] [delay_ms]
        if partes[0] == "w":
            try:
                ciclos    = int(partes[1])     if len(partes) > 1 else 3
                stride    = float(partes[2])   if len(partes) > 2 else 40.0
                lift      = float(partes[3])   if len(partes) > 3 else 30.0
                delay_ms  = float(partes[4])   if len(partes) > 4 else 50.0
                gait_walk(ciclos=ciclos, stride_mm=stride, lift_mm=lift, delay=delay_ms/1000.0)
            except (ValueError, IndexError):
                print("Formato: w [ciclos] [stride_mm] [lift_mm] [delay_ms]")
                print("Exemplo: w 5 40 30 50")
            continue

        # Modo círculo: c [ciclos] [raio_mm] [pontos] [delay_ms]
        if partes[0] == "c":
            try:
                ciclos    = int(partes[1])     if len(partes) > 1 else 3
                raio      = float(partes[2])   if len(partes) > 2 else 25.0
                pontos    = int(partes[3])     if len(partes) > 3 else 20
                delay_ms  = float(partes[4])   if len(partes) > 4 else 50.0
                gait_circle(ciclos=ciclos, raio_mm=raio, pontos=pontos, delay=delay_ms/1000.0)
            except (ValueError, IndexError):
                print("Formato: c [ciclos] [raio_mm] [pontos] [delay_ms]")
                print("Exemplo: c 3 25 20 50")
            continue

        # Buscar limites máximos
        if partes[0] == "max":
            find_max_stride()
            continue

        # Modo servo direto: s <servo0> <servo4>
        if partes[0] == "s" and len(partes) == 3:
            try:
                s0 = float(partes[1])
                s4 = float(partes[2])
                move_servos_safe(s0, s4)
            except ValueError:
                print("Formato inválido. Use: s <ombro_graus> <joelho_graus>")
            continue

        # Modo IK: x z
        if len(partes) == 2:
            try:
                x = float(partes[0])
                z = float(partes[1])
                move_foot_to(x, z)
            except ValueError:
                print("Formato inválido. Use: x z (posição em mm)")
            continue

        print("Comando não reconhecido. Use:")
        print("  x z              — cinemática inversa (mm)")
        print("  s s0 s4          — controle direto (graus)")
        print("  w [c] [s] [l] [d]  — caminhada")
        print("  c [c] [r] [p] [d]  — círculo")
        print("  max              — mostra limites máximos")
        print("  n                — posição neutra")
        print("  q                — sair")
