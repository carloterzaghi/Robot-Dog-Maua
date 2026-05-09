"""
workspace_plot.py — Visualização do espaço de trabalho seguro da perna.

Gera um gráfico mostrando todas as posições que o pé pode alcançar
sem danificar o mecanismo, com base nas restrições mecânicas medidas.

Modelo cinemático (cadeia serial RR):
  - Pivô do fêmur em O = (0, 0).
  - Braço do fêmur: L_F = 120 mm, ângulo θ_f (absoluto).
  - Joelho K = (L_F·cos θ_f, L_F·sin θ_f).
  - Braço da tíbia: L_T = 130 mm, ângulo θ_t (absoluto), a partir de K.
  - Pé P = K + (L_T·cos θ_t, L_T·sin θ_t).

Convenção de ângulos:
  ângulo_físico = ângulo_servo − 90°
  θ = 0°  → braço horizontal para frente (+x)
  θ = −90° → braço vertical para baixo (−y) → servo em 0°

Restrições (equações de regressão da tabela de dados):
  offset = servo_femur − 45
  tíbia máx (físico°) = min(50,  offset + 10,333)
  tíbia mín (físico°) = max(−75, 0,9733·offset − 99,4)

Rodar:  python workspace_plot.py
Saída:  workspace.png (mesmo diretório)
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines

# ── Geometria física (mm) ─────────────────────────────────────────────────────
L_F = 120.0   # comprimento do braço do fêmur [mm]
L_T = 130.0   # comprimento do braço da tíbia [mm]

# ── Limites de servo ──────────────────────────────────────────────────────────
FEMUR_MIN_SRV  =  45    # servo mín do fêmur (físico −45°)
FEMUR_MAX_SRV  = 135    # servo máx do fêmur (físico +45°)
TIBIA_MIN_ABS  =  15    # servo mín absoluto da tíbia (físico −75°)
TIBIA_MAX_ABS  = 140    # servo máx absoluto da tíbia (físico +50°)
TIBIA_MARGEM   =   8    # margem de segurança já usada no x_move.py

# Parâmetros do x_move.py (valores de referência para sobrepor no gráfico)
Y_SOLO = -150.0
X_MIN  =  170.0
X_MAX  =  190.0


# ── Funções de restrição e cinemática ─────────────────────────────────────────

def tibia_limites_servo(femur_servo: float) -> tuple:
    """Retorna (servo_min, servo_max) da tíbia para dado ângulo do fêmur."""
    offset = femur_servo - 45.0
    t_max_fis = min(50.0,   offset + 10.333)
    t_min_fis = max(-75.0,  0.9733 * offset - 99.4)
    return (max(float(TIBIA_MIN_ABS), 90.0 + t_min_fis),
            min(float(TIBIA_MAX_ABS), 90.0 + t_max_fis))


def fk(servo_f: float, servo_t: float) -> tuple:
    """
    Cinemática direta serial RR.
    Retorna (Kx, Ky, Px, Py) — joelho e pé em mm.
    """
    tf = math.radians(servo_f - 90.0)
    tt = math.radians(servo_t - 90.0)
    Kx = L_F * math.cos(tf)
    Ky = L_F * math.sin(tf)
    Px = Kx + L_T * math.cos(tt)
    Py = Ky + L_T * math.sin(tt)
    return Kx, Ky, Px, Py


# ── Varredura completa do espaço de trabalho ──────────────────────────────────
STEPS_F = 400    # resolução do fêmur
STEPS_T = 200    # resolução da tíbia para cada posição de fêmur

# Coleções para o polígono de contorno
edge_max_pts = []   # pé na tíbia máxima (contorno "externo")
edge_min_pts = []   # pé na tíbia mínima (contorno "interno")

# Todos os pontos do pé (preenche a região)
all_foot_x = []
all_foot_y = []

# Posições do joelho (para mostrar o alcance do fêmur)
knee_arc_x = []
knee_arc_y = []

for i in range(STEPS_F + 1):
    sf = FEMUR_MIN_SRV + (FEMUR_MAX_SRV - FEMUR_MIN_SRV) * i / STEPS_F
    t_min, t_max = tibia_limites_servo(sf)

    if t_max < t_min:
        continue

    # Contorno externo (tíbia no máximo)
    _, _, px, py = fk(sf, t_max)
    edge_max_pts.append((px, py))

    # Contorno interno (tíbia no mínimo)
    _, _, px, py = fk(sf, t_min)
    edge_min_pts.append((px, py))

    # Arco do joelho (extremidade do braço do fêmur)
    Kx, Ky, _, _ = fk(sf, t_min)
    knee_arc_x.append(Kx)
    knee_arc_y.append(Ky)

    # Pontos internos para preenchimento
    for j in range(STEPS_T + 1):
        st = t_min + (t_max - t_min) * j / STEPS_T
        _, _, px, py = fk(sf, st)
        all_foot_x.append(px)
        all_foot_y.append(py)

edge_max_pts = np.array(edge_max_pts)
edge_min_pts = np.array(edge_min_pts)

# Polígono fechado: máx (frente→trás) + mín (trás→frente)
poly_x = np.concatenate([edge_max_pts[:, 0], edge_min_pts[::-1, 0]])
poly_y = np.concatenate([edge_max_pts[:, 1], edge_min_pts[::-1, 1]])


# ── Figuras de configuração exemplar ─────────────────────────────────────────
# 5 posições do fêmur × 3 posições da tíbia (mín, médio, máx)
EXAMPLE_FEMURS = [45, 60, 90, 120, 135]
example_arms = []
for sf in EXAMPLE_FEMURS:
    t_min, t_max = tibia_limites_servo(sf)
    t_mid = (t_min + t_max) / 2.0
    for st in [t_min, t_mid, t_max]:
        Kx, Ky, Px, Py = fk(sf, st)
        example_arms.append((sf, st, Kx, Ky, Px, Py))


# ── Plot ──────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(11, 13))
fig.patch.set_facecolor('#f8f9fa')
ax.set_facecolor('#f0f4f8')

# Espaço de trabalho preenchido
ax.fill(poly_x, poly_y, color='#4a90d9', alpha=0.20, zorder=1)

# Contornos de limite
ax.plot(edge_max_pts[:, 0], edge_max_pts[:, 1],
        color='#2980b9', linewidth=2.0, linestyle='--',
        zorder=3, label='Limite tíbia máx.')
ax.plot(edge_min_pts[:, 0], edge_min_pts[:, 1],
        color='#c0392b', linewidth=2.0, linestyle='--',
        zorder=3, label='Limite tíbia mín.')

# Arco do joelho (extremidade do fêmur)
ax.plot(knee_arc_x, knee_arc_y,
        color='#8e44ad', linewidth=1.2, linestyle=':',
        zorder=3, label='Arco do joelho (extremidade do fêmur)')

# Configurações de exemplo (braços)
for sf, st, Kx, Ky, Px, Py in example_arms:
    # Escolhe cor pelo fêmur
    frac = (sf - FEMUR_MIN_SRV) / (FEMUR_MAX_SRV - FEMUR_MIN_SRV)
    color = plt.cm.RdYlGn(0.1 + 0.8 * frac)
    ax.plot([0, Kx, Px], [0, Ky, Py],
            '-', color=color, alpha=0.25, linewidth=1.0, zorder=2)

# Margem de segurança (região com margem TIBIA_MARGEM)
margin_max_pts = []
margin_min_pts = []
for i in range(STEPS_F + 1):
    sf = FEMUR_MIN_SRV + (FEMUR_MAX_SRV - FEMUR_MIN_SRV) * i / STEPS_F
    t_min, t_max = tibia_limites_servo(sf)
    if t_max - t_min < 2 * TIBIA_MARGEM:
        continue
    _, _, px, py = fk(sf, t_max - TIBIA_MARGEM)
    margin_max_pts.append((px, py))
    _, _, px, py = fk(sf, t_min + TIBIA_MARGEM)
    margin_min_pts.append((px, py))

if margin_max_pts and margin_min_pts:
    margin_max_pts = np.array(margin_max_pts)
    margin_min_pts = np.array(margin_min_pts)
    mpoly_x = np.concatenate([margin_max_pts[:, 0], margin_min_pts[::-1, 0]])
    mpoly_y = np.concatenate([margin_max_pts[:, 1], margin_min_pts[::-1, 1]])
    ax.fill(mpoly_x, mpoly_y, color='#27ae60', alpha=0.18, zorder=2,
            label=f'Zona de operação (margem ±{TIBIA_MARGEM}°)')
    ax.plot(margin_max_pts[:, 0], margin_max_pts[:, 1],
            color='#27ae60', linewidth=1.0, linestyle='-', alpha=0.6, zorder=3)
    ax.plot(margin_min_pts[:, 0], margin_min_pts[:, 1],
            color='#27ae60', linewidth=1.0, linestyle='-', alpha=0.6, zorder=3)

# ── Linha de movimento do x_move.py ──────────────────────────────────────────
ax.axhline(Y_SOLO, color='#7f8c8d', linewidth=0.8, linestyle=':', alpha=0.7)
ax.plot([X_MIN, X_MAX], [Y_SOLO, Y_SOLO],
        color='#e67e22', linewidth=3.5, solid_capstyle='round',
        zorder=5, label=f'Movimento X (x_move.py)  y = {Y_SOLO:.0f} mm')
ax.annotate('',
            xy=(X_MAX + 5, Y_SOLO), xytext=(X_MIN - 5, Y_SOLO),
            arrowprops=dict(arrowstyle='<->', color='#e67e22', lw=2.0))
ax.text((X_MIN + X_MAX) / 2, Y_SOLO + 8,
        f'{X_MIN:.0f} ↔ {X_MAX:.0f} mm',
        ha='center', va='bottom', fontsize=8.5, color='#e67e22', fontweight='bold')

# ── Pivô do fêmur ─────────────────────────────────────────────────────────────
ax.plot(0, 0, 'ko', markersize=12, zorder=6)
ax.plot(0, 0, 'wo', markersize=6,  zorder=7)
ax.annotate('Eixo fêmur\n(origem)', xy=(0, 0),
            xytext=(12, 12), textcoords='offset points',
            fontsize=8, color='#2c3e50',
            arrowprops=dict(arrowstyle='->', color='#2c3e50', lw=1.0))

# ── Dimensões indicativas ─────────────────────────────────────────────────────
# Ângulo de fêmur neutro (servo 90° → θ_f = 0° → aponta para +x)
Kx0, Ky0, _, _ = fk(90, 90)
ax.annotate('',
            xy=(Kx0, Ky0), xytext=(0, 0),
            arrowprops=dict(arrowstyle='->', color='#95a5a6', lw=1.5, linestyle='dashed'))
ax.text(Kx0 / 2, Ky0 / 2 + 5, f'L_F={L_F:.0f} mm',
        ha='center', va='bottom', fontsize=8, color='#7f8c8d')

_, _, Px0, Py0 = fk(90, 90)
ax.annotate('',
            xy=(Px0, Py0), xytext=(Kx0, Ky0),
            arrowprops=dict(arrowstyle='->', color='#bdc3c7', lw=1.5, linestyle='dashed'))
ax.text((Kx0 + Px0) / 2 + 5, (Ky0 + Py0) / 2, f'L_T={L_T:.0f} mm',
        ha='left', va='center', fontsize=8, color='#95a5a6')

# ── Estatísticas do workspace ─────────────────────────────────────────────────
all_x = np.array(all_foot_x)
all_y = np.array(all_foot_y)
info = (
    f"X:  {all_x.min():.0f} a {all_x.max():.0f} mm\n"
    f"Y:  {all_y.min():.0f} a {all_y.max():.0f} mm"
)
ax.text(0.02, 0.03, info,
        transform=ax.transAxes, fontsize=8.5,
        verticalalignment='bottom',
        bbox=dict(boxstyle='round,pad=0.4', facecolor='white',
                  edgecolor='#bdc3c7', alpha=0.9))

# ── Eixos, grade e rótulos ────────────────────────────────────────────────────
ax.axhline(0, color='#2c3e50', linewidth=0.6, alpha=0.5)
ax.axvline(0, color='#2c3e50', linewidth=0.6, alpha=0.5)
ax.set_aspect('equal')
ax.grid(True, alpha=0.25, linestyle='--', linewidth=0.6)
ax.set_xlabel('X  [mm]  →  frente', fontsize=11)
ax.set_ylabel('Y  [mm]  →  cima', fontsize=11)
ax.set_title(
    'Espaço de Trabalho Seguro da Perna\nRobot Dog Mauá — Mecanismo 5 barras',
    fontsize=13, fontweight='bold', pad=14
)

# Legenda extra: cores dos exemplos de configuração
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
sm = ScalarMappable(cmap='RdYlGn',
                    norm=Normalize(vmin=FEMUR_MIN_SRV, vmax=FEMUR_MAX_SRV))
sm.set_array([])
cbar = fig.colorbar(sm, ax=ax, shrink=0.35, pad=0.01, aspect=20)
cbar.set_label('Servo fêmur [°]', fontsize=8)
cbar.ax.tick_params(labelsize=7)

ax.legend(loc='upper right', fontsize=8.5,
          framealpha=0.92, edgecolor='#bdc3c7')

plt.tight_layout()
out_path = 'workspace.png'
plt.savefig(out_path, dpi=150, bbox_inches='tight')
print(f"Gráfico salvo em: {out_path}")
plt.show()
