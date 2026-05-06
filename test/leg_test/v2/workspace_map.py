"""
workspace_map.py — Gera o mapa do workspace (espaço alcançável) do pé.

Varre todas as combinações seguras de servo 0 e servo 4 (dentro dos limites
da planilha Excel) e calcula a posição do pé via cinemática direta.
Salva o resultado como imagem e como tabela CSV.
"""

from math import cos, sin, sqrt, pi
import numpy as np
import matplotlib
matplotlib.use('Agg')  # backend sem GUI
import matplotlib.pyplot as plt

# ── Parâmetros físicos (mm) ──
L_BAR     = 120.0
L_COUPLE  = 39.0
L_TIBIA   = 130.0
D_CHASSIS = 39.0

SERVO0_NEUTRO = 90.0
SERVO4_NEUTRO = 90.0

# ── Limites de segurança (Excel) ──
LIMITES_TABLE = np.array([
    [  45.0,   10.0,  100.0],
    [  60.0,   10.0,  115.0],
    [  75.0,   10.0,  130.0],
    [  90.0,   15.0,  140.0],
    [ 105.0,   30.0,  155.0],
    [ 120.0,   45.0,  170.0],
    [ 135.0,   70.0,  170.0],
])

SERVO0_MIN = float(LIMITES_TABLE[:, 0].min())
SERVO0_MAX = float(LIMITES_TABLE[:, 0].max())


def servo4_min(s0):
    return float(np.interp(s0, LIMITES_TABLE[:, 0], LIMITES_TABLE[:, 1]))

def servo4_max(s0):
    return float(np.interp(s0, LIMITES_TABLE[:, 0], LIMITES_TABLE[:, 2]))


def forward_kinematics(servo0_deg, servo4_deg):
    alpha = np.radians(-(servo0_deg - SERVO0_NEUTRO))
    beta  = np.radians(-(servo4_deg - SERVO4_NEUTRO))

    ax = L_BAR * sin(alpha)
    az = -L_BAR * cos(alpha)

    bx = L_BAR * sin(beta)
    bz = -D_CHASSIS - L_BAR * cos(beta)

    dab = sqrt((bx - ax)**2 + (bz - az)**2)
    if dab > 1e-6:
        dx = (bx - ax) / dab
        dz = (bz - az) / dab
    else:
        dx, dz = 0.0, -1.0

    fx = bx + L_TIBIA * dx
    fz = bz + L_TIBIA * dz

    return fx, fz


# ── Varrer todo o espaço seguro ──
print("Calculando workspace...")

points_x = []
points_z = []
points_s0 = []
points_s4 = []

step = 1.0  # resolução de 1°

for s0 in np.arange(SERVO0_MIN, SERVO0_MAX + step/2, step):
    s4_lo = servo4_min(s0)
    s4_hi = servo4_max(s0)
    for s4 in np.arange(s4_lo, s4_hi + step/2, step):
        fx, fz = forward_kinematics(s0, s4)
        points_x.append(fx)
        points_z.append(fz)
        points_s0.append(s0)
        points_s4.append(s4)

points_x = np.array(points_x)
points_z = np.array(points_z)
points_s0 = np.array(points_s0)
points_s4 = np.array(points_s4)

# Ponto neutro
x_neutro, z_neutro = forward_kinematics(SERVO0_NEUTRO, SERVO4_NEUTRO)

print(f"Total de pontos: {len(points_x)}")
print(f"Range X: [{points_x.min():+.1f}, {points_x.max():+.1f}] mm")
print(f"Range Z: [{points_z.min():+.1f}, {points_z.max():+.1f}] mm")
print(f"Neutro:  ({x_neutro:+.1f}, {z_neutro:+.1f}) mm")

# ── Gerar visualização ──
fig, axes = plt.subplots(1, 2, figsize=(16, 8))

# --- Plot 1: Workspace colorido por servo 0 ---
ax1 = axes[0]
sc1 = ax1.scatter(points_x, points_z, c=points_s0, cmap='coolwarm', s=1, alpha=0.6)
ax1.plot(x_neutro, z_neutro, 'k*', markersize=15, label=f'Neutro ({x_neutro:.0f}, {z_neutro:.0f})')
ax1.set_xlabel('X (mm) — frente/trás', fontsize=12)
ax1.set_ylabel('Z (mm) — cima/baixo', fontsize=12)
ax1.set_title('Workspace do Pé — colorido por Servo 0', fontsize=14)
ax1.legend(fontsize=10)
ax1.set_aspect('equal')
ax1.grid(True, alpha=0.3)
ax1.invert_yaxis()  # Z negativo = abaixo
cbar1 = plt.colorbar(sc1, ax=ax1)
cbar1.set_label('Servo 0 (°)', fontsize=10)

# --- Plot 2: Workspace colorido por servo 4 ---
ax2 = axes[1]
sc2 = ax2.scatter(points_x, points_z, c=points_s4, cmap='viridis', s=1, alpha=0.6)
ax2.plot(x_neutro, z_neutro, 'k*', markersize=15, label=f'Neutro ({x_neutro:.0f}, {z_neutro:.0f})')
ax2.set_xlabel('X (mm) — frente/trás', fontsize=12)
ax2.set_ylabel('Z (mm) — cima/baixo', fontsize=12)
ax2.set_title('Workspace do Pé — colorido por Servo 4', fontsize=14)
ax2.legend(fontsize=10)
ax2.set_aspect('equal')
ax2.grid(True, alpha=0.3)
ax2.invert_yaxis()
cbar2 = plt.colorbar(sc2, ax=ax2)
cbar2.set_label('Servo 4 (°)', fontsize=10)

plt.suptitle('Espaço de Trabalho Alcançável (Limites Seguros da Planilha Excel)', fontsize=16, fontweight='bold')
plt.tight_layout()

output_path = r'd:\Carlo\Documentos\Raspberry Pi 4\Robot-Dog-Maua\test\leg_test\v2\workspace_map.png'
plt.savefig(output_path, dpi=150, bbox_inches='tight')
print(f"\nImagem salva em: {output_path}")

# ── Plot 3: Contorno do workspace (borda) ──
fig2, ax3 = plt.subplots(1, 1, figsize=(10, 8))

# Para cada s0 fixo, pegar os pontos extremos (s4_min e s4_max)
border_x = []
border_z = []

# Borda superior (s4_min para cada s0)
for s0 in np.arange(SERVO0_MIN, SERVO0_MAX + 0.5, 1.0):
    fx, fz = forward_kinematics(s0, servo4_min(s0))
    border_x.append(fx)
    border_z.append(fz)

# Borda inferior (s4_max para cada s0, em ordem reversa)
for s0 in np.arange(SERVO0_MAX, SERVO0_MIN - 0.5, -1.0):
    fx, fz = forward_kinematics(s0, servo4_max(s0))
    border_x.append(fx)
    border_z.append(fz)

# Fechar o contorno
border_x.append(border_x[0])
border_z.append(border_z[0])

ax3.fill(border_x, border_z, alpha=0.2, color='dodgerblue', label='Área alcançável')
ax3.plot(border_x, border_z, 'b-', linewidth=1.5)
ax3.plot(x_neutro, z_neutro, 'r*', markersize=20, label=f'Neutro ({x_neutro:.0f}, {z_neutro:.0f}) mm', zorder=5)

# Marcar linhas de s0 constante
for s0 in [45, 60, 75, 90, 105, 120, 135]:
    line_x = []
    line_z = []
    for s4 in np.arange(servo4_min(s0), servo4_max(s0) + 0.5, 1.0):
        fx, fz = forward_kinematics(s0, s4)
        line_x.append(fx)
        line_z.append(fz)
    ax3.plot(line_x, line_z, '--', alpha=0.5, linewidth=0.8, label=f's0={s0}°')

ax3.set_xlabel('X (mm) — frente(+) / trás(-)', fontsize=13)
ax3.set_ylabel('Z (mm) — abaixo(-)', fontsize=13)
ax3.set_title('Contorno do Workspace + Linhas de Servo 0 Constante', fontsize=14, fontweight='bold')
ax3.legend(loc='upper right', fontsize=9)
ax3.set_aspect('equal')
ax3.grid(True, alpha=0.3)
ax3.invert_yaxis()

output_path2 = r'd:\Carlo\Documentos\Raspberry Pi 4\Robot-Dog-Maua\test\leg_test\v2\workspace_contour.png'
plt.savefig(output_path2, dpi=150, bbox_inches='tight')
print(f"Contorno salvo em: {output_path2}")

# ── Salvar tabela dos pontos de borda ──
print("\n" + "=" * 60)
print("  RESUMO DO WORKSPACE")
print("=" * 60)
print(f"  X range:  {points_x.min():+.1f} mm  a  {points_x.max():+.1f} mm")
print(f"  Z range:  {points_z.min():+.1f} mm  a  {points_z.max():+.1f} mm")
print(f"  Neutro:   ({x_neutro:+.1f}, {z_neutro:+.1f}) mm")
print(f"  Total de combinações seguras: {len(points_x)}")
print()

# Pontos extremos
idx_max_x = np.argmax(points_x)
idx_min_x = np.argmin(points_x)
idx_max_z = np.argmax(points_z)
idx_min_z = np.argmin(points_z)

print("  Extremos:")
print(f"    Mais à frente:  ({points_x[idx_max_x]:+.1f}, {points_z[idx_max_x]:+.1f}) mm  [s0={points_s0[idx_max_x]:.0f}°, s4={points_s4[idx_max_x]:.0f}°]")
print(f"    Mais atrás:     ({points_x[idx_min_x]:+.1f}, {points_z[idx_min_x]:+.1f}) mm  [s0={points_s0[idx_min_x]:.0f}°, s4={points_s4[idx_min_x]:.0f}°]")
print(f"    Mais acima:     ({points_x[idx_max_z]:+.1f}, {points_z[idx_max_z]:+.1f}) mm  [s0={points_s0[idx_max_z]:.0f}°, s4={points_s4[idx_max_z]:.0f}°]")
print(f"    Mais abaixo:    ({points_x[idx_min_z]:+.1f}, {points_z[idx_min_z]:+.1f}) mm  [s0={points_s0[idx_min_z]:.0f}°, s4={points_s4[idx_min_z]:.0f}°]")
print("=" * 60)
