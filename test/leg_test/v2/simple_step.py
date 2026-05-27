"""
simple_step.py — Movimento simples de passada para uma perna do Robot Dog Mauá.

Ponto de partida: fêmur = 90° e tíbia = 90° (posição neutra).

Trajetória de passada (parâmetro t, 0 a 2π por ciclo):
  Fêmur  : onda cossenoidal ± FEMUR_AMP em torno de 90°.
    t = 0  → fêmur avançado (+FEMUR_AMP físico)
    t = π  → fêmur recuado  (−FEMUR_AMP físico)

  Tíbia : chaveada pelos limites mecânicos dinâmicos:
    Balanço [sin(t) ≥ 0] — pé no ar: tíbia próxima do máximo permitido.
    Apoio   [sin(t) < 0] — pé no chão: tíbia próxima do mínimo + margem.

Convenção de ângulos:
  ângulo_servo = ângulo_físico + 90°  →  90° = posição neutra.

Hardware: Raspberry Pi + PCA9685 (via Adafruit ServoKit, I2C).
  Servo fêmur : canal 3  (braço 120 mm)
  Servo tíbia : canal 7  (braço 130 mm)
"""

import math
import time
from adafruit_servokit import ServoKit        # type: ignore

# ── Inicialização ──────────────────────────────────────────────────────────────
kit = ServoKit(channels=16)
servo_femur = kit.servo[3]
servo_tibia = kit.servo[7]

# ── Limites de hardware (graus de servo) ──────────────────────────────────────
FEMUR_CENTRO  = 90
FEMUR_MIN     = 45    # −45° físico
FEMUR_MAX     = 135   # +45° físico

TIBIA_CENTRO  = 90
TIBIA_MIN_ABS = 15    # −75° físico
TIBIA_MAX_ABS = 140   # +50° físico

# ── Parâmetros de gait ────────────────────────────────────────────────────────
FEMUR_AMP        = 30     # amplitude do balanço do fêmur (graus físicos)
TIBIA_MARGEM     = 8      # margem de segurança nos limites da tíbia (graus)
PASSOS_POR_CICLO = 100    # resolução do ciclo — mais passos = mais suave
STEP_DELAY       = 0.008  # segundos entre cada passo
HOLD_DELAY       = 0.4    # pausa na posição neutra antes de iniciar

# ── Número de ciclos a executar ───────────────────────────────────────────────
N_CICLOS = 4


# ── Restrições mecânicas (equações de regressão medidas) ─────────────────────

def tibia_limites_servo(femur_servo: float) -> tuple:
    """
    Retorna (tibia_min_servo, tibia_max_servo) para o ângulo de fêmur dado.

    Equações de regressão linear:
      offset          = femur_servo − 45
      tibia_max_físico = min( 50,  offset + 10.333)
      tibia_min_físico = max(−75,  0.9733 × offset − 99.4)
    """
    offset = femur_servo - 45.0
    tibia_max_fis = min(50.0,  offset + 10.333)
    tibia_min_fis = max(-75.0, 0.9733 * offset - 99.4)

    tibia_min_srv = max(float(TIBIA_MIN_ABS), TIBIA_CENTRO + tibia_min_fis)
    tibia_max_srv = min(float(TIBIA_MAX_ABS), TIBIA_CENTRO + tibia_max_fis)
    return tibia_min_srv, tibia_max_srv


# ── Posição inicial ───────────────────────────────────────────────────────────

def posicao_neutra() -> None:
    """Leva ambos os servos à posição neutra (90°/90°) de forma suave."""
    print("  → Posição neutra (fêmur: 90° | tíbia: 90°)")
    f = servo_femur.angle if servo_femur.angle is not None else FEMUR_CENTRO
    t = servo_tibia.angle if servo_tibia.angle is not None else TIBIA_CENTRO
    passos = max(abs(int(f) - FEMUR_CENTRO), abs(int(t) - TIBIA_CENTRO), 1)
    for i in range(passos + 1):
        frac = i / passos
        servo_femur.angle = round(f + (FEMUR_CENTRO - f) * frac)
        servo_tibia.angle = round(t + (TIBIA_CENTRO - t) * frac)
        time.sleep(STEP_DELAY)
    time.sleep(HOLD_DELAY)


# ── Ciclo de passada ──────────────────────────────────────────────────────────

def ciclo_passada() -> None:
    """
    Executa um ciclo completo de passada com trajetória suave.

    Fêmur segue uma cossenoide. A tíbia é posicionada dinamicamente
    nos limites mecânicos: máximo na fase de balanço (pé no ar) e
    mínimo + margem na fase de apoio (pé no chão).
    """
    for i in range(PASSOS_POR_CICLO):
        t = 2.0 * math.pi * i / PASSOS_POR_CICLO

        # Fêmur: cossenoidal — avança na primeira metade, recua na segunda
        femur_servo = FEMUR_CENTRO + FEMUR_AMP * math.cos(t)
        femur_servo = max(FEMUR_MIN, min(FEMUR_MAX, round(femur_servo)))

        # Limites dinâmicos da tíbia para este ângulo de fêmur
        t_min, t_max = tibia_limites_servo(femur_servo)

        if math.sin(t) >= 0:
            # Balanço: pé no ar — tíbia próxima do máximo (pé levantado)
            tibia_servo = t_max - TIBIA_MARGEM
        else:
            # Apoio: pé no chão — tíbia próxima do mínimo (empurrando)
            tibia_servo = t_min + TIBIA_MARGEM

        tibia_servo = max(TIBIA_MIN_ABS, min(TIBIA_MAX_ABS, round(tibia_servo)))

        servo_femur.angle = femur_servo
        servo_tibia.angle = tibia_servo
        time.sleep(STEP_DELAY)


# ── Ponto de entrada ──────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=== simple_step.py — Movimento simples de perna ===")
    print(f"  Servo fêmur : canal 3  |  Servo tíbia : canal 7")
    print(f"  Amplitude   : ±{FEMUR_AMP}° físico  |  Ciclos: {N_CICLOS}\n")

    try:
        posicao_neutra()

        for ciclo in range(1, N_CICLOS + 1):
            print(f"  Ciclo {ciclo}/{N_CICLOS}...")
            ciclo_passada()

        print("\nMovimento concluído. Retornando à posição neutra...")
        posicao_neutra()

    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário. Voltando à posição neutra...")
        posicao_neutra()
