"""
y_move.py — Movimento vertical (eixo Y) reto da perna do Robot Dog Mauá.

Move o pé para cima e para baixo mantendo a posição X constante,
usando cinemática inversa para o mecanismo de cadeia serial RR
(modelo simplificado do mecanismo de 5 barras).

GEOMETRIA (mesma do 5bars.py):
  - Fêmur: L_F = 120 mm, pivô na origem (0, 0).
  - Tíbia: L_T = 130 mm, pivô no joelho.
  - Servo fêmur: canal 3 — ângulo_servo = ângulo_físico + 90°.
  - Servo tíbia:  canal 7 — ângulo_servo = ângulo_físico + 90°.

CINEMÁTICA INVERSA (cadeia RR):
  Dado (Px, Py) do pé, calcula θ_f e θ_t (ângulos físicos).

  d² = Px² + Py²
  cos θ_t_rel = (d² − L_F² − L_T²) / (2 · L_F · L_T)    [lei dos cossenos]
  θ_t_rel = atan2(−√(1 − cos²), cos)                       [elbow-down]
  θ_f = atan2(Py, Px) − atan2(L_T·sin θ_t_rel, L_F + L_T·cos θ_t_rel)
  θ_t = θ_f + θ_t_rel                                      [ângulo absoluto]

MOVIMENTO:
  X fixo (ex: 180 mm), Y varia entre Y_MIN e Y_MAX (ex: −100 a −200 mm).
  O pé sobe e desce em linha reta vertical.

Restrições mecânicas: mesmas equações de regressão do 5bars.py.

Hardware: Raspberry Pi + PCA9685 (via Adafruit ServoKit, I2C).
"""

import math
import time
from adafruit_servokit import ServoKit        # type: ignore

# ── Inicialização do controlador de servos (PCA9685, 16 canais, I2C) ──────────
kit = ServoKit(channels=16)
servo_femur = kit.servo[3]   # Servo fêmur — braço 120 mm
servo_tibia = kit.servo[7]   # Servo tíbia — braço 130 mm

# ── Geometria (mm) ────────────────────────────────────────────────────────────
L_F = 120.0   # comprimento do braço do fêmur [mm]
L_T = 130.0   # comprimento do braço da tíbia [mm]

# ── Limites de servo (graus) ──────────────────────────────────────────────────
FEMUR_CENTRO  = 90
FEMUR_MIN     = 45     # −45° físico
FEMUR_MAX     = 135    # +45° físico

TIBIA_CENTRO  = 90
TIBIA_MIN_ABS = 15     # −75° físico
TIBIA_MAX_ABS = 140    # +50° físico

# ── Parâmetros de movimento vertical ─────────────────────────────────────────
STEP_DELAY       = 0.015   # Intervalo (s) entre cada passo
HOLD_DELAY       = 0.5     # Pausa (s) nos extremos e posição inicial
PASSOS_POR_CICLO = 80      # Resolução de cada subida/descida

# Posição X fixa do pé durante o movimento vertical [mm]
# (180 mm é um valor seguro dentro do espaço de trabalho, frente do robô)
X_FIXO = 180.0

# Faixa de Y para o movimento vertical [mm]
# Y negativo = para baixo (convenção: Y positivo = para cima)
Y_MIN = -200.0   # ponto mais baixo (pé mais estendido)
Y_MAX = -100.0    # ponto mais alto  (pé mais recolhido)

# Margem de segurança (graus) em relação aos limites mecânicos
TIBIA_MARGEM = 8


# ── Restrições mecânicas (mesmas do 5bars.py) ─────────────────────────────────

def tibia_limites_servo(femur_servo: float) -> tuple:
    """
    Retorna (tibia_min_servo, tibia_max_servo) para dado ângulo de fêmur.

    Equações de regressão:
      offset = femur_servo − 45
      tibia_max_físico = min( 50,  offset + 10,333)
      tibia_min_físico = max(−75,  0,9733 × offset − 99,4)
    """
    offset = femur_servo - 45.0
    tibia_max_fis = min(50.0,  offset + 10.333)
    tibia_min_fis = max(-75.0, 0.9733 * offset - 99.4)

    tibia_min_srv = max(float(TIBIA_MIN_ABS), TIBIA_CENTRO + tibia_min_fis)
    tibia_max_srv = min(float(TIBIA_MAX_ABS), TIBIA_CENTRO + tibia_max_fis)

    return tibia_min_srv, tibia_max_srv


# ── Cinemática inversa (cadeia RR) ────────────────────────────────────────────

def ik(px: float, py: float) -> tuple:
    """
    Cinemática inversa para a cadeia serial RR.

    Dado o ponto do pé (px, py) em mm, retorna (servo_femur, servo_tibia)
    em graus de servo. Usa configuração elbow-down (cotovelo para baixo).

    Retorna None se o ponto está fora do alcance.

    Parâmetros:
        px: coordenada X do pé [mm]
        py: coordenada Y do pé [mm]

    Retorna:
        (servo_femur, servo_tibia) em graus, ou None se inalcançável.
    """
    d_sq = px * px + py * py
    d = math.sqrt(d_sq)

    # Verificar se está dentro do alcance
    if d > L_F + L_T or d < abs(L_F - L_T):
        return None

    # Lei dos cossenos para ângulo relativo do "cotovelo"
    cos_theta_t_rel = (d_sq - L_F * L_F - L_T * L_T) / (2.0 * L_F * L_T)
    cos_theta_t_rel = max(-1.0, min(1.0, cos_theta_t_rel))  # clamp numérico

    # Configuração elbow-down: sin negativo → cotovelo para baixo
    sin_theta_t_rel = -math.sqrt(1.0 - cos_theta_t_rel * cos_theta_t_rel)
    theta_t_rel = math.atan2(sin_theta_t_rel, cos_theta_t_rel)

    # Ângulo do fêmur (absoluto)
    theta_f = math.atan2(py, px) - math.atan2(
        L_T * sin_theta_t_rel,
        L_F + L_T * cos_theta_t_rel
    )

    # Ângulo absoluto da tíbia
    theta_t = theta_f + theta_t_rel

    # Converter de radianos para graus de servo (servo = físico + 90)
    servo_f = math.degrees(theta_f) + 90.0
    servo_t = math.degrees(theta_t) + 90.0

    return servo_f, servo_t


def ik_seguro(px: float, py: float) -> tuple:
    """
    Cinemática inversa com verificação de restrições mecânicas.

    Retorna (servo_femur, servo_tibia) se o ponto é alcançável E seguro,
    ou None caso contrário.
    """
    resultado = ik(px, py)
    if resultado is None:
        return None

    servo_f, servo_t = resultado

    # Verificar limites do servo do fêmur
    if servo_f < FEMUR_MIN or servo_f > FEMUR_MAX:
        return None

    # Verificar limites dinâmicos da tíbia
    t_min, t_max = tibia_limites_servo(servo_f)
    if servo_t < (t_min + TIBIA_MARGEM) or servo_t > (t_max - TIBIA_MARGEM):
        return None

    # Verificar limites absolutos
    if servo_t < TIBIA_MIN_ABS or servo_t > TIBIA_MAX_ABS:
        return None

    return servo_f, servo_t


# ── Funções de movimento ──────────────────────────────────────────────────────

def mover_suave(servo, angulo_inicio: float, angulo_fim: float,
                delay: float = STEP_DELAY) -> None:
    """Move um servo grau a grau de angulo_inicio até angulo_fim."""
    inicio = int(angulo_inicio)
    fim    = int(angulo_fim)
    passo  = 1 if fim > inicio else -1
    for ang in range(inicio, fim + passo, passo):
        servo.angle = ang
        time.sleep(delay)


def posicao_inicial() -> None:
    """Leva a perna à posição de repouso: fêmur e tíbia em 90°."""
    print("Movendo para posição inicial (fêmur: 90°, tíbia: 90°)...")
    i_f = int(servo_femur.angle) if servo_femur.angle is not None else FEMUR_CENTRO
    i_t = int(servo_tibia.angle) if servo_tibia.angle is not None else TIBIA_CENTRO
    mover_suave(servo_femur, i_f, FEMUR_CENTRO)
    mover_suave(servo_tibia, i_t, TIBIA_CENTRO)
    time.sleep(HOLD_DELAY)


def encontrar_segmentos_y(x_fixo: float, y_min: float, y_max: float,
                          resolucao: int = 500) -> list:
    """
    Varre o eixo Y e retorna uma lista de SEGMENTOS CONTÍGUOS alcançáveis.

    O espaço de trabalho é uma faixa curvada (meia-lua). Uma linha vertical
    pode cruzar esta faixa em vários trechos separados com buracos no meio.
    Esta função detecta cada trecho contíguo.

    Retorna:
        Lista de tuplas (y_min, y_max, curso) para cada segmento contíguo,
        ordenada do segmento com maior curso para o menor.
        Lista vazia se nenhum ponto for alcançável.
    """
    # Tamanho máximo de um "buraco" que ainda consideramos como contínuo (mm)
    # Isso evita que ruído numérico fragmente um segmento real.
    tolerancia_gap = (y_max - y_min) / resolucao * 3.0

    segmentos = []
    seg_inicio = None
    seg_fim = None

    for i in range(resolucao + 1):
        y = y_min + (y_max - y_min) * i / resolucao
        if ik_seguro(x_fixo, y) is not None:
            if seg_inicio is None:
                seg_inicio = y
            seg_fim = y
        else:
            if seg_inicio is not None:
                # Verificar se é um buraco real ou ruído numérico
                # Olhar para frente para ver se retoma logo
                if i + 3 < resolucao:
                    y_next = y_min + (y_max - y_min) * (i + 3) / resolucao
                    if ik_seguro(x_fixo, y_next) is not None:
                        continue  # Provavelmente ruído, continuar segmento

                curso = seg_fim - seg_inicio
                if curso > 0.5:  # Ignorar segmentos menores que 0.5mm
                    segmentos.append((seg_inicio, seg_fim, curso))
                seg_inicio = None
                seg_fim = None

    # Fechar último segmento se existir
    if seg_inicio is not None:
        curso = seg_fim - seg_inicio
        if curso > 0.5:
            segmentos.append((seg_inicio, seg_fim, curso))

    # Ordenar por curso (maior primeiro)
    segmentos.sort(key=lambda s: s[2], reverse=True)
    return segmentos


def encontrar_faixa_y(x_fixo: float, y_min: float, y_max: float,
                      resolucao: int = 500) -> tuple:
    """
    Retorna (y_min_real, y_max_real) do MAIOR segmento contíguo alcançável.

    Usa encontrar_segmentos_y() para detectar segmentos separados e
    retorna apenas o maior — evita reportar ranges falsos quando a
    linha vertical cruza a faixa curvada do workspace em dois pontos.
    """
    segmentos = encontrar_segmentos_y(x_fixo, y_min, y_max, resolucao)
    if not segmentos:
        return None
    # Retorna o maior segmento contíguo
    melhor = segmentos[0]
    return melhor[0], melhor[1]


def ciclo_vertical(n_ciclos: int = 3) -> None:
    """
    Executa n_ciclos completos de movimento vertical (sobe e desce).

    O pé se move em linha reta no eixo Y, com X fixo.
    Cada ciclo: desce de Y_MAX até Y_MIN, depois sobe de Y_MIN até Y_MAX.

    Usa interpolação senoidal para aceleração/desaceleração suave
    nos extremos do curso.
    """
    # Encontrar faixa real alcançável
    print(f"Verificando espaço de trabalho para X = {X_FIXO:.1f} mm...")
    faixa = encontrar_faixa_y(X_FIXO, Y_MIN - 20, Y_MAX + 20)

    if faixa is None:
        print(f"ERRO: Nenhum ponto alcançável em X = {X_FIXO} mm!")
        print("  Tente ajustar X_FIXO para um valor dentro do espaço de trabalho.")
        return

    y_bottom, y_top = faixa
    # Aplicar margem de segurança de 5 mm nos extremos
    y_bottom = max(Y_MIN, y_bottom + 5.0)
    y_top    = min(Y_MAX, y_top - 5.0)

    if y_bottom >= y_top:
        print(f"ERRO: Faixa Y muito pequena! ({y_bottom:.1f} a {y_top:.1f} mm)")
        return

    print(f"  Faixa Y alcançável: {y_bottom:.1f} a {y_top:.1f} mm")
    print(f"  X fixo: {X_FIXO:.1f} mm")
    print(f"  Curso vertical: {y_top - y_bottom:.1f} mm")
    print()

    # Ir para a posição inicial do movimento (topo)
    print("Movendo para posição inicial do eixo Y...")
    resultado = ik_seguro(X_FIXO, y_top)
    if resultado is None:
        print("ERRO: Posição inicial inalcançável!")
        return

    i_f = int(servo_femur.angle) if servo_femur.angle is not None else FEMUR_CENTRO
    i_t = int(servo_tibia.angle) if servo_tibia.angle is not None else TIBIA_CENTRO
    mover_suave(servo_femur, i_f, round(resultado[0]))
    mover_suave(servo_tibia, i_t, round(resultado[1]))
    time.sleep(HOLD_DELAY)

    print(f"Iniciando {n_ciclos} ciclo(s) de movimento vertical.")
    print()

    for ciclo in range(n_ciclos):
        # ── Descida: y_top → y_bottom (senoidal para suavidade) ──────────
        for i in range(PASSOS_POR_CICLO):
            # Interpolação senoidal: começa e termina devagar
            t = i / PASSOS_POR_CICLO
            # Função senoidal: 0→1 com aceleração/desaceleração suave
            s = 0.5 * (1.0 - math.cos(math.pi * t))
            y = y_top + (y_bottom - y_top) * s

            resultado = ik_seguro(X_FIXO, y)
            if resultado is None:
                # Se o ponto exato não é alcançável, pula
                continue

            f_srv, t_srv = resultado
            f_final = max(FEMUR_MIN, min(FEMUR_MAX, round(f_srv)))
            t_final = max(TIBIA_MIN_ABS, min(TIBIA_MAX_ABS, round(t_srv)))

            servo_femur.angle = f_final
            servo_tibia.angle = t_final

            time.sleep(STEP_DELAY)

        time.sleep(0.1)  # Breve pausa no ponto mais baixo

        # ── Subida: y_bottom → y_top (senoidal para suavidade) ───────────
        for i in range(PASSOS_POR_CICLO):
            t = i / PASSOS_POR_CICLO
            s = 0.5 * (1.0 - math.cos(math.pi * t))
            y = y_bottom + (y_top - y_bottom) * s

            resultado = ik_seguro(X_FIXO, y)
            if resultado is None:
                continue

            f_srv, t_srv = resultado
            f_final = max(FEMUR_MIN, min(FEMUR_MAX, round(f_srv)))
            t_final = max(TIBIA_MIN_ABS, min(TIBIA_MAX_ABS, round(t_srv)))

            servo_femur.angle = f_final
            servo_tibia.angle = t_final

            time.sleep(STEP_DELAY)

        time.sleep(0.1)  # Breve pausa no ponto mais alto

        print(f"  Ciclo {ciclo + 1}/{n_ciclos} concluído.")

    print()


# ── Mapeamento do espaço de trabalho ─────────────────────────────────────────

def mapear_workspace() -> None:
    """
    Varre todo o espaço de trabalho e imprime:
      1. Tabela com faixa de Y alcançável para cada valor de X.
      2. Mapa ASCII mostrando a área segura.
      3. Destaca o X com maior curso vertical.
    """
    print()
    print("=" * 62)
    print("  MAPEAMENTO DO ESPAÇO DE TRABALHO SEGURO")
    print("=" * 62)
    print()
    print("  Varrendo... (pode demorar alguns segundos)")
    print()

    # ── 1. Tabela: para cada X, encontrar segmentos de Y ────────────────
    x_inicio = -50
    x_fim    = 260
    x_passo  = 10
    y_scan_min = -260.0
    y_scan_max =  260.0
    y_resolucao = 600

    # Lista de (x, segmentos) onde segmentos = [(y_min, y_max, curso), ...]
    tabela = []
    melhor_x = None
    melhor_curso = 0
    melhor_seg = None

    for x in range(x_inicio, x_fim + 1, x_passo):
        segs = encontrar_segmentos_y(float(x), y_scan_min, y_scan_max,
                                     resolucao=y_resolucao)
        if segs:
            tabela.append((x, segs))
            # O maior segmento contíguo é o primeiro (já está ordenado)
            if segs[0][2] > melhor_curso:
                melhor_curso = segs[0][2]
                melhor_x = x
                melhor_seg = segs[0]

    if not tabela:
        print("  ERRO: Nenhum ponto alcançável encontrado!")
        return

    # ── Imprimir tabela ───────────────────────────────────────────────────
    print("  ┌────────┬─────┬────────────┬────────────┬──────────┐")
    print("  │  X(mm) │ Seg │ Y min (mm) │ Y max (mm) │ Curso(mm)│")
    print("  ├────────┼─────┼────────────┼────────────┼──────────┤")
    for x, segs in tabela:
        for idx, (y_lo, y_hi, curso) in enumerate(segs):
            marca = " ◀ MELHOR" if x == melhor_x and idx == 0 else ""
            seg_label = f"{idx + 1}/{len(segs)}"
            if idx == 0:
                print(f"  │ {x:5d}  │ {seg_label:>3} │ {y_lo:9.1f}  │ {y_hi:9.1f}  │ {curso:7.1f}  │{marca}")
            else:
                print(f"  │   ↑    │ {seg_label:>3} │ {y_lo:9.1f}  │ {y_hi:9.1f}  │ {curso:7.1f}  │")
        if len(segs) > 1:
            print("  ├────────┼─────┼────────────┼────────────┼──────────┤")
    print("  └────────┴─────┴────────────┴────────────┴──────────┘")
    print()

    if melhor_x is not None and melhor_seg is not None:
        print(f"  ★ Melhor X para movimento vertical CONTÍGUO: X = {melhor_x} mm")
        print(f"    Y de {melhor_seg[0]:.1f} a {melhor_seg[1]:.1f} mm "
              f"(curso contíguo: {melhor_seg[2]:.1f} mm)")
    print()

    # ── 2. Mapa ASCII do espaço de trabalho ───────────────────────────────
    print("  MAPA DO ESPAÇO DE TRABALHO (vista lateral):")
    print("  (O = pivô do fêmur / origem)")
    print()

    # Resolução mais fina para o mapa
    x_map_min = min(t[0] for t in tabela) - 10
    x_map_max = max(t[0] for t in tabela) + 10
    y_map_min = min(seg[0] for _, segs in tabela for seg in segs) - 10
    y_map_max = max(seg[1] for _, segs in tabela for seg in segs) + 10

    # Grid do mapa ASCII
    cols = 72
    rows = 30

    grid = [[' ' for _ in range(cols)] for _ in range(rows)]

    # Preencher grid com pontos alcançáveis (varredura fina)
    for xi in range(cols):
        px = x_map_min + (x_map_max - x_map_min) * xi / (cols - 1)
        for yi in range(rows):
            # Y invertido: linha 0 = topo = Y mais positivo
            py = y_map_max - (y_map_max - y_map_min) * yi / (rows - 1)
            if ik_seguro(px, py) is not None:
                grid[yi][xi] = '░'

    # Marcar a origem (0, 0) no grid
    ox = round((0 - x_map_min) / (x_map_max - x_map_min) * (cols - 1))
    oy = round((y_map_max - 0) / (y_map_max - y_map_min) * (rows - 1))
    if 0 <= ox < cols and 0 <= oy < rows:
        grid[oy][ox] = 'O'

    # Marcar linha do X fixo atual
    x_col = round((X_FIXO - x_map_min) / (x_map_max - x_map_min) * (cols - 1))
    if 0 <= x_col < cols:
        for yi in range(rows):
            py = y_map_max - (y_map_max - y_map_min) * yi / (rows - 1)
            if grid[yi][x_col] == '░':
                grid[yi][x_col] = '█'
            elif grid[yi][x_col] == ' ':
                grid[yi][x_col] = '│'

    # Marcar melhor X
    if melhor_x is not None:
        best_col = round((melhor_x - x_map_min) / (x_map_max - x_map_min) * (cols - 1))
        if 0 <= best_col < cols and best_col != x_col:
            for yi in range(rows):
                if grid[yi][best_col] == '░':
                    grid[yi][best_col] = '▓'

    # Imprimir grid com eixos
    print(f"  Y(mm)")
    for yi in range(rows):
        py = y_map_max - (y_map_max - y_map_min) * yi / (rows - 1)
        if yi % 5 == 0:
            label = f"{py:6.0f}"
        else:
            label = "      "
        print(f"  {label} │{''.join(grid[yi])}│")

    # Eixo X
    print(f"         └{'─' * cols}┘")
    # Labels do eixo X
    x_labels = f"        {x_map_min:<10.0f}"
    x_mid = (x_map_min + x_map_max) / 2
    x_labels += f"{' ' * (cols // 2 - 15)}{x_mid:.0f}"
    x_labels += f"{' ' * (cols // 2 - 10)}{x_map_max:.0f}"
    print(x_labels)
    print(f"  {'':>8}{'X (mm) →':^{cols}}")
    print()
    print("  Legenda:")
    print("    ░ = espaço seguro alcançável")
    print(f"    █ = X fixo atual ({X_FIXO:.0f} mm)")
    if melhor_x is not None:
        print(f"    ▓ = melhor X para curso vertical contíguo ({melhor_x} mm)")
    print("    O = origem (pivô do fêmur)")
    print()

    # ── 3. Sugestões ──────────────────────────────────────────────────────
    print("  ─── SUGESTÕES ───")
    print()
    # Encontrar Xs cujo maior segmento contíguo tem curso > 20mm
    bons = []
    for x, segs in tabela:
        # segs[0] é o maior segmento contíguo
        y_lo, y_hi, curso = segs[0]
        if curso > 20:
            bons.append((x, y_lo, y_hi, curso))
    if bons:
        print("  Valores de X com curso vertical contíguo > 20 mm:")
        for x, y_lo, y_hi, curso in bons:
            print(f"    X={x:4d} mm → Y: {y_lo:.0f} a {y_hi:.0f} mm "
                  f"(curso contíguo: {curso:.0f} mm)")
    else:
        print("  Nenhum X com curso vertical contíguo > 20 mm encontrado.")
        print("  O mecanismo tem amplitude vertical limitada nesta configuração.")
    print()


# ── Menu interativo ──────────────────────────────────────────────────────────

def menu():
    """Menu interativo para ajustar parâmetros e executar o movimento."""
    global X_FIXO, Y_MIN, Y_MAX, PASSOS_POR_CICLO, STEP_DELAY

    print("=" * 60)
    print("  MOVIMENTO VERTICAL (EIXO Y) — Robot Dog Mauá")
    print("=" * 60)
    print()

    while True:
        print(f"  Configuração atual:")
        print(f"    X fixo       : {X_FIXO:.1f} mm")
        print(f"    Y min (baixo): {Y_MIN:.1f} mm")
        print(f"    Y max (cima) : {Y_MAX:.1f} mm")
        print(f"    Passos/ciclo : {PASSOS_POR_CICLO}")
        print(f"    Delay/passo  : {STEP_DELAY*1000:.1f} ms")
        print()
        print("  Comandos:")
        print("    [1] Executar movimento vertical (3 ciclos)")
        print("    [2] Executar movimento vertical (N ciclos)")
        print("    [3] Alterar X fixo")
        print("    [4] Alterar Y min / Y max")
        print("    [5] Alterar velocidade (passos e delay)")
        print("    [6] Ir para posição inicial (90°, 90°)")
        print("    [7] Testar ponto específico (X, Y)")
        print("    [8] Mapear espaço de trabalho seguro")
        print("    [q] Sair")
        print()

        cmd = input("  > ").strip().lower()

        if cmd == '1':
            ciclo_vertical(n_ciclos=3)

        elif cmd == '2':
            try:
                n = int(input("    Número de ciclos: "))
                ciclo_vertical(n_ciclos=n)
            except ValueError:
                print("    Valor inválido.")

        elif cmd == '3':
            try:
                X_FIXO = float(input(f"    Novo X fixo (atual: {X_FIXO:.1f}): "))
            except ValueError:
                print("    Valor inválido.")

        elif cmd == '4':
            try:
                Y_MIN = float(input(f"    Novo Y min (atual: {Y_MIN:.1f}): "))
                Y_MAX = float(input(f"    Novo Y max (atual: {Y_MAX:.1f}): "))
            except ValueError:
                print("    Valor inválido.")

        elif cmd == '5':
            try:
                PASSOS_POR_CICLO = int(input(f"    Passos/ciclo (atual: {PASSOS_POR_CICLO}): "))
                delay_ms = float(input(f"    Delay/passo em ms (atual: {STEP_DELAY*1000:.1f}): "))
                STEP_DELAY = delay_ms / 1000.0
            except ValueError:
                print("    Valor inválido.")

        elif cmd == '6':
            posicao_inicial()

        elif cmd == '7':
            try:
                px = float(input("    X (mm): "))
                py = float(input("    Y (mm): "))
                resultado = ik_seguro(px, py)
                if resultado is None:
                    print(f"    ERRO: ({px:.1f}, {py:.1f}) está fora do espaço seguro!")
                    # Mostrar resultado da IK pura (sem restrições) para debug
                    ik_puro = ik(px, py)
                    if ik_puro is None:
                        print("    (Fora do alcance cinemático)")
                    else:
                        sf, st = ik_puro
                        print(f"    IK pura: fêmur={sf:.1f}°, tíbia={st:.1f}°")
                        t_min, t_max = tibia_limites_servo(sf)
                        print(f"    Limites tíbia: {t_min:.1f}° a {t_max:.1f}° "
                              f"(com margem: {t_min+TIBIA_MARGEM:.1f}° a {t_max-TIBIA_MARGEM:.1f}°)")
                else:
                    sf, st = resultado
                    print(f"    OK! Fêmur: {sf:.1f}° servo, Tíbia: {st:.1f}° servo")
                    resp = input("    Mover para esta posição? (s/n): ").strip().lower()
                    if resp == 's':
                        i_f = int(servo_femur.angle) if servo_femur.angle is not None else FEMUR_CENTRO
                        i_t = int(servo_tibia.angle) if servo_tibia.angle is not None else TIBIA_CENTRO
                        mover_suave(servo_femur, i_f, round(sf))
                        mover_suave(servo_tibia, i_t, round(st))
                        print("    Movido!")
            except ValueError:
                print("    Valor inválido.")

        elif cmd == '8':
            mapear_workspace()

        elif cmd == 'q':
            break

        print()


# ── Execução principal ────────────────────────────────────────────────────────
try:
    posicao_inicial()
    menu()
    print("Programa encerrado.")
    posicao_inicial()

except KeyboardInterrupt:
    print("\nInterrompido pelo usuário.")
    posicao_inicial()
