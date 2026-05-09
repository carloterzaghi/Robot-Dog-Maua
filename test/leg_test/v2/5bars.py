"""
5bars.py — Teste do mecanismo de 5 barras v2 para uma perna do Robot Dog Mauá.

MECANISMO DE 5 BARRAS (conforme desenho medido):
  - Servo 0 (fêmur) : braço de 120 mm — elevação e avanço/recuo da perna.
  - Servo 4 (tíbia) : braço de 130 mm — extensão da perna.
  - Offset entre eixos dos servos: 35 mm.
  - Ângulo entre eixos dos servos: 90°.
  - Montagem do servo: 32,53 mm horiz. + 23 mm vert. + 23 mm horiz.

Restrições mecânicas (tabela de dados medidos):
  ┌──────────────────────────────────────────────────────────────────┐
  │ Fêmur físico (°) │ Tíbia máx (físico °) │ Tíbia mín (físico °) │
  │──────────────────│──────────────────────│──────────────────────│
  │       -45        │          10          │         -75          │
  │       -30        │          26          │         -75          │
  │       -15        │          40          │         -70          │
  │         0        │          50          │         -55          │
  │        15        │          50          │         -43          │
  │        30        │          50          │         -26          │
  │        45        │          50          │         -12          │
  └──────────────────────────────────────────────────────────────────┘

  Equações lineares de restrição (obtidas por regressão — gráficos):
    offset = femur_servo − 45   (0 quando fêmur = −45°, 90 quando fêmur = +45°)

    Tíbia máx (físico °) = min( 50 ,  offset + 10,333)
    Tíbia mín (físico °) = max(−75 ,  0,9733 × offset − 99,4)

  Ambas já convertidas para ângulo de servo somando 90°
  (tíbia_servo = tíbia_físico + 90).

Ciclo de passada:
  - Fase de balanço [t ∈ 0, π]  — pé no ar:
      Fêmur avança (−30° → +30°), tíbia próxima do máximo permitido.
  - Fase de apoio   [t ∈ π, 2π] — pé no chão:
      Fêmur recua (+30° → −30°), tíbia próxima do mínimo + margem de segurança.

Como executar:
    python 5bars.py

Hardware: Raspberry Pi + PCA9685 (via Adafruit ServoKit, I2C).
"""

import math
import time
from adafruit_servokit import ServoKit        # type: ignore

# ── Inicialização do controlador de servos (PCA9685, 16 canais, I2C) ──────────
kit = ServoKit(channels=16)
servo_femur = kit.servo[3]   # Servo 0: fêmur — braço 120 mm
servo_tibia = kit.servo[7]   # Servo 4: tíbia — braço 130 mm

# ── Ângulos de referência (graus servo; centro em 90°) ────────────────────────
#
# Convenção:
#   ângulo_servo = ângulo_físico + 90°
#   Portanto, servo em 90° → membro em posição neutra (0° físico).
#
# FÊMUR (Servo 0):
#   FEMUR_CENTRO : membro neutro, 0° físico.
#   FEMUR_MIN    : membro em −45° físico (perna mais recuada).
#   FEMUR_MAX    : membro em +45° físico (perna mais avançada).
FEMUR_CENTRO = 90
FEMUR_MIN    = 45    # −45° físico
FEMUR_MAX    = 135   # +45° físico

# TÍBIA (Servo 4):
#   TIBIA_CENTRO  : membro neutro, 0° físico.
#   TIBIA_MIN_ABS : limite inferior absoluto do servo (−75° físico).
#   TIBIA_MAX_ABS : limite superior absoluto do servo (+50° físico).
TIBIA_CENTRO  = 90
TIBIA_MIN_ABS = 15   # −75° físico
TIBIA_MAX_ABS = 140  # +50° físico

# ── Parâmetros de movimento ───────────────────────────────────────────────────
STEP_DELAY       = 0.01   # Intervalo (s) entre cada passo do ciclo
HOLD_DELAY       = 0.5    # Pausa (s) ao atingir a posição inicial
PASSOS_POR_CICLO = 100    # Resolução do ciclo — mais passos = mais suave

# Amplitude do balanço do fêmur em torno de FEMUR_CENTRO
FEMUR_AMP = 30   # ± 30° físico → servo oscila entre 60° e 120°

# Margem de segurança (°) em relação aos limites calculados da tíbia
TIBIA_MARGEM = 8


# ── Restrições mecânicas ──────────────────────────────────────────────────────

def tibia_limites_servo(femur_servo: float) -> tuple:
    """
    Retorna (tibia_min_servo, tibia_max_servo) para dado ângulo de fêmur.

    Usa as equações de regressão linear extraídas dos gráficos da tabela
    de dados mecânicos:

      offset = femur_servo − 45
      tibia_max_físico = min( 50,  offset + 10,333)
      tibia_min_físico = max(−75,  0,9733 × offset − 99,4)

    Os valores são convertidos para ângulo de servo (centro = 90°) antes
    de serem retornados.

    Parâmetros:
        femur_servo: ângulo atual do servo do fêmur (graus, 45 a 135).

    Retorna:
        (tibia_min_servo, tibia_max_servo): limites válidos da tíbia em
        graus de servo, já clampeados aos limites absolutos do hardware.
    """
    offset = femur_servo - 45.0

    tibia_max_fis = min(50.0,  offset + 10.333)
    tibia_min_fis = max(-75.0, 0.9733 * offset - 99.4)

    tibia_min_srv = max(float(TIBIA_MIN_ABS), TIBIA_CENTRO + tibia_min_fis)
    tibia_max_srv = min(float(TIBIA_MAX_ABS), TIBIA_CENTRO + tibia_max_fis)

    return tibia_min_srv, tibia_max_srv


# ── Funções de movimento ──────────────────────────────────────────────────────

def mover_suave(servo, angulo_inicio: float, angulo_fim: float,
                delay: float = STEP_DELAY) -> None:
    """
    Move um servo grau a grau de angulo_inicio até angulo_fim.

    Parâmetros:
        servo        : instância do servo (adafruit_servokit.Servo).
        angulo_inicio: ângulo de partida em graus.
        angulo_fim   : ângulo de destino em graus.
        delay        : intervalo em segundos entre cada passo.
    """
    inicio = int(angulo_inicio)
    fim    = int(angulo_fim)
    passo  = 1 if fim > inicio else -1
    for ang in range(inicio, fim + passo, passo):
        servo.angle = ang
        time.sleep(delay)


def posicao_inicial() -> None:
    """
    Leva a perna à posição de repouso: fêmur e tíbia centralizados em 90°.

    Move os dois servos sequencialmente a partir de seus ângulos atuais,
    garantindo transição suave independentemente do estado anterior.
    """
    print("Movendo para posição inicial (fêmur: 90°, tíbia: 90°)...")
    i_f = int(servo_femur.angle) if servo_femur.angle is not None else FEMUR_CENTRO
    i_t = int(servo_tibia.angle) if servo_tibia.angle is not None else TIBIA_CENTRO
    mover_suave(servo_femur, i_f, FEMUR_CENTRO)
    mover_suave(servo_tibia, i_t, TIBIA_CENTRO)
    time.sleep(HOLD_DELAY)


def ciclo_passada(n_ciclos: int = 5) -> None:
    """
    Executa n_ciclos completos de passada usando o mecanismo de 5 barras.

    Trajetória (parâmetro t, 0 a 2π por ciclo):

      Fêmur: onda cossenoidal, ampltude ± FEMUR_AMP graus.
        t = 0   → fêmur na posição mais avançada (+FEMUR_AMP físico).
        t = π   → fêmur na posição mais recuada  (−FEMUR_AMP físico).

      Tíbia:
        Fase de balanço [sin(t) ≥ 0, t ∈ 0..π]:
          Pé no ar — tíbia posicionada próxima ao MÁXIMO permitido para o
          ângulo atual do fêmur (levanta o pé e avança).
        Fase de apoio [sin(t) < 0, t ∈ π..2π]:
          Pé no chão — tíbia posicionada próxima ao MÍNIMO permitido + margem
          (mantém contato com o solo, empurra o corpo para frente).

    Em ambas as fases os ângulos são clampeados pelos limites calculados por
    tibia_limites_servo() e pelos limites absolutos de hardware.

    Parâmetros:
        n_ciclos: número de ciclos completos de passada.
    """
    print(f"Iniciando {n_ciclos} ciclo(s) de passada — mecanismo 5 barras.")
    print(f"  Fêmur: {FEMUR_CENTRO - FEMUR_AMP}° a {FEMUR_CENTRO + FEMUR_AMP}° servo")
    print(f"  Tíbia: limitada dinamicamente pelas equações de restrição")

    for ciclo in range(n_ciclos):
        for i in range(PASSOS_POR_CICLO):
            t = 2.0 * math.pi * i / PASSOS_POR_CICLO

            # ── Fêmur: cossenoidal ────────────────────────────────────────────
            # t=0  → cos(0)=1  → fêmur em +FEMUR_AMP (frente)
            # t=π  → cos(π)=-1 → fêmur em −FEMUR_AMP (trás)
            femur_fisico = FEMUR_AMP * math.cos(t)
            femur_servo  = FEMUR_CENTRO + femur_fisico

            # ── Limites dinâmicos da tíbia para este ângulo de fêmur ─────────
            t_min, t_max = tibia_limites_servo(femur_servo)

            # ── Tíbia: chaveamento por fase ───────────────────────────────────
            if math.sin(t) >= 0:
                # Balanço: perto do máximo (pé levantado e avançando)
                tibia_alvo = t_max - TIBIA_MARGEM
            else:
                # Apoio: perto do mínimo + margem (pé no chão, empurrando)
                tibia_alvo = t_min + TIBIA_MARGEM

            # ── Arredonda e clampeia aos limites absolutos de hardware ────────
            femur_final = max(FEMUR_MIN,     min(FEMUR_MAX,     round(femur_servo)))
            tibia_final = max(TIBIA_MIN_ABS, min(TIBIA_MAX_ABS, round(tibia_alvo)))

            servo_femur.angle = femur_final
            servo_tibia.angle = tibia_final

            time.sleep(STEP_DELAY)

        print(f"  Ciclo {ciclo + 1}/{n_ciclos} concluído.")


# ── Execução principal ────────────────────────────────────────────────────────
try:
    posicao_inicial()
    ciclo_passada(n_ciclos=5)
    print("Movimento concluído.")
    posicao_inicial()

except KeyboardInterrupt:
    print("\nInterrompido pelo usuário.")
    posicao_inicial()
