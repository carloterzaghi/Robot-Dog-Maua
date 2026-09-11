"""
Configuração central do Robô Quadrúpede — Robot-Dog-Maua.

Todas as constantes de hardware, geometria e parâmetros de marcha ficam aqui.
Qualquer ajuste (canal, ângulo, velocidade) deve ser feito apenas neste arquivo.
"""

# ── Mapeamento de canais PCA9685 (placa v4) ───────────────────────────────────
# Cada chave é "<posição>_<articulação>_<lado>", valor é o canal do PCA9685.
SERVO_CHANNELS: dict[str, int] = {
    # Pata Frontal Direita
    "frente_femur_dir":    10,
    "frente_angular_dir":   9,
    "frente_tibia_dir":     8,
    # Pata Frontal Esquerda
    "frente_femur_esq":    13,
    "frente_angular_esq":  14,
    "frente_tibia_esq":    15,
    # Pata Traseira Direita
    "tras_femur_dir":       7,
    "tras_angular_dir":     6,
    "tras_tibia_dir":       5,
    # Pata Traseira Esquerda
    "tras_femur_esq":       2,
    "tras_angular_esq":     1,
    "tras_tibia_esq":       0,
}

# ── Ângulos base da posição default (referência para cálculo de offsets) ──────
DEFAULT_ANGLES: dict[str, float] = {
    "frente_femur_dir":    90.0,
    "frente_angular_dir": 100.0,
    "frente_tibia_dir":    90.0,
    "frente_femur_esq":    90.0,
    "frente_angular_esq": 105.0,
    "frente_tibia_esq":    90.0,
    "tras_femur_dir":      90.0,
    "tras_angular_dir":   100.0,
    "tras_tibia_dir":      90.0,
    "tras_femur_esq":      90.0,
    "tras_angular_esq":   95.0,
    "tras_tibia_esq":      90.0,
}

# ── Geometria das pernas (mm) ─────────────────────────────────────────────────
UPPER_LEG:  int = 120   # comprimento do segmento superior (fêmur)
LOWER_LEG:  int = 130   # comprimento do segmento inferior (tíbia)
MAX_RADIUS: int = 220   # alcance máximo da perna (raio)
MAX_Z:      int = -80   # teto de Z (mais próximo do corpo)
TIBIA_OFFSET: float = -8.0  # correção angular aplicada à tíbia após IK

# ── Parâmetros de marcha ──────────────────────────────────────────────────────
# Z_APOIO: altura do pé no chão durante a fase de apoio.
#   Diminuir (ex: -220) = perna mais esticada (corpo mais alto).
#   Aumentar (ex: -195) = perna mais dobrada (corpo mais baixo).
#   Z_APOIO ajustado para -195 para evitar violação do alcance máximo (MAX_RADIUS=220)
#   durante a passada completa de X=90.
GAIT_PARAMS: dict[str, dict] = {
    "frente": {
        "z_apoio":  -220,   # mm — altura de apoio das pernas dianteiras
        "z_swing":  -140,   # mm — altura máxima durante o swing
        "x_frente":   90,   # mm — posição X na frente do ombro
        "x_atras":   -90,   # mm — posição X atrás do ombro
        "n_pontos":   20,   # pontos por fase (swing/apoio)
        "delay":    0.015,  # s  — delay entre pontos
    },
    "tras": {
        "z_apoio":  -220,   # mm — altura de apoio das pernas traseiras
        "z_swing":  -140,   # mm — altura máxima durante o swing
        "x_frente":   90,
        "x_atras":   -90,
        "n_pontos":   20,
        "delay":    0.015,
    },
}

# ── Escala de Giro (Turn Scale) ───────────────────────────────────────────────
# Define a amplitude do giro para as pernas. Como z_apoio foi rebaixado para -195,
# as pernas conseguem realizar o percurso total (-90 a 90) sem a IK estourar o limite,
# permitindo o uso de 1.0 (sincronia perfeita) ou valores menores para giros mais lentos.
TURN_SCALE: float = 1.0

# ── Configuração individual de cada perna ─────────────────────────────────────
# mirror: True = ângulos espelhados (180° - valor), para pernas do lado esquerdo
# phase:  "swing_first" | "stance_first" — qual fase inicia o ciclo de marcha
# ang_min/ang_max: faixa de operação do servo angular desta perna
LEG_CONFIG: dict[str, dict] = {
    "frente_dir": {
        "group":  "frente",
        "mirror": False,
        "phase":  "swing_first",
        "ang_min": 70,
        "ang_max": 120,
        "angular_fixed_angle": 100,    # ângulo do angular quando use_angular=False
        "turn_scale": TURN_SCALE,
    },
    "frente_esq": {
        "group":  "frente",
        "mirror": True,
        "phase":  "stance_first",      # defasada 180° em relação à frente_dir
        "ang_min": 90,
        "ang_max": 135,
        "angular_fixed_angle": 110,
        "turn_scale": TURN_SCALE,
    },
    "tras_dir": {
        "group":  "tras",
        "mirror": False,
        "phase":  "stance_first",      # diagonal com frente_esq
        "ang_min": 90,
        "ang_max": 135,
        "angular_fixed_angle": 100,
        "turn_scale": TURN_SCALE,
        # Usa os defaults do grupo "tras" (z_apoio=-195, z_swing=-115)
    },
    "tras_esq": {
        "group":  "tras",
        "mirror": True,
        "phase":  "swing_first",       # diagonal com frente_dir
        "ang_min": 70,
        "ang_max": 120,
        "angular_fixed_angle": 100,
        "turn_scale": TURN_SCALE,
        # Usa os defaults do grupo "tras"
    },
}

# ── Parâmetros de rampa de inicialização ──────────────────────────────────────
N_RAMP:     int   = 40      # número de passos da rampa
RAMP_DELAY: float = 0.025   # s — delay entre passos da rampa

# ── Poses predefinidas ────────────────────────────────────────────────────────
# Ângulos absolutos (sem calibração) para transições rápidas.
# A transição final passa pela calibração dentro de ServoManager.smooth_move().

POSE_STAND: dict[str, float] = {
    # Tíbias e fêmures
    "frente_femur_dir":   30.0,
    "frente_tibia_dir":   60.0,
    "frente_femur_esq":  150.0,
    "frente_tibia_esq":  120.0,
    "tras_femur_dir":     20.0,
    "tras_tibia_dir":     40.0,
    "tras_femur_esq":    150.0,
    "tras_tibia_esq":    135.0,
    # Angulares
    "frente_angular_dir": 101.0,
    "frente_angular_esq": 110.0,
    "tras_angular_dir":    98.0,
    "tras_angular_esq":   93.0,
}

POSE_SLEEP: dict[str, float] = {
    # Fase 1 — posição intermediária (fêmures/tíbias)
    "frente_femur_dir":   80.0,
    "frente_tibia_dir":  120.0,
    "frente_femur_esq":  100.0,
    "frente_tibia_esq":   70.0,
    "tras_femur_dir":     80.0,
    "tras_tibia_dir":    120.0,
    "tras_femur_esq":    100.0,
    "tras_tibia_esq":     70.0,
    # Fase 2 — angulares deitados
    "frente_angular_dir":  70.0,
    "frente_angular_esq": 135.0,
    "tras_angular_dir":   135.0,
    "tras_angular_esq":    70.0,
}

# Pose intermediária antes de deitar (fêmures e tíbias dobram primeiro)
POSE_SLEEP_STRUCT: dict[str, float] = {
    "frente_femur_dir":   30.0,
    "frente_tibia_dir":   60.0,
    "frente_femur_esq":  150.0,
    "frente_tibia_esq":  120.0,
    "tras_femur_dir":     30.0,
    "tras_tibia_dir":     60.0,
    "tras_femur_esq":    150.0,
    "tras_tibia_esq":    120.0,
}

# ── Parâmetros suaves de movimentação ─────────────────────────────────────────
SMOOTH_N_STEPS: int   = 60     # passos da interpolação smooth_move
SMOOTH_DELAY:   float = 0.02   # s — delay entre passos
