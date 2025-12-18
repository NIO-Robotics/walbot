#!/usr/bin/env python3
#=============================================================================
#     Exemples avec orientation horizontale et rayon de virage
#=============================================================================

from wall_painting_trajectory import WallPaintingTrajectory
import numpy as np
import matplotlib.pyplot as plt

print("="*70)
print("COMPARAISON: ORIENTATIONS VERTICALE vs HORIZONTALE")
print("="*70)

# ==================== TRAJECTOIRE VERTICALE ====================
print("\n1. Trajectoire VERTICALE (passages de haut en bas)")
print("-" * 70)

traj_vertical = WallPaintingTrajectory(
    wall_width=1.0,
    wall_height=3.0,
    tool_width=0.05,
    overlap_ratio=0.10,
    orientation='vertical',  # Passages verticaux
    turning_radius=0.1       # Pas de virage
)

trajectory_vert = traj_vertical.generate()
stats_vert = traj_vertical.get_statistics()


# ==================== TRAJECTOIRE HORIZONTALE ====================
print("\n2. Trajectoire HORIZONTALE (passages de gauche à droite)")
print("-" * 70)

traj_horizontal = WallPaintingTrajectory(
    wall_width=1.0,
    wall_height=3.0,
    tool_width=0.05,
    overlap_ratio=0.10,
    orientation='horizontal',  # Passages horizontaux
    turning_radius=0.0         # Pas de virage
)

trajectory_horiz = traj_horizontal.generate()
stats_horiz = traj_horizontal.get_statistics()


# ==================== VISUALISATION COMPARAISON ORIENTATIONS ====================
fig, axes = plt.subplots(1, 2, figsize=(14, 8))

# --- Verticale ---
ax = axes[0]
ax.set_aspect('equal')
ax.set_xlim(-0.1, 1.1)
ax.set_ylim(-0.1, 3.1)
ax.set_xlabel('Largeur (m)', fontsize=11)
ax.set_ylabel('Hauteur (m)', fontsize=11)
ax.set_title(f'VERTICALE\n{stats_vert["num_passes"]} passages - {stats_vert["total_distance"]:.1f}m',
            fontsize=12, fontweight='bold')
ax.grid(True, alpha=0.3)

# Contour du mur
wall_x = [0, 1, 1, 0, 0]
wall_y = [0, 0, 3, 3, 0]
ax.plot(wall_x, wall_y, 'k-', linewidth=2)

# Trajectoire avec gradient
n_segments = len(trajectory_vert) - 1
colors = plt.cm.viridis(np.linspace(0, 1, n_segments))
for i in range(n_segments):
    ax.plot(trajectory_vert[i:i+2, 0], trajectory_vert[i:i+2, 1],
           color=colors[i], linewidth=2, alpha=0.8)

ax.plot(trajectory_vert[0, 0], trajectory_vert[0, 1], 'go', markersize=12, label='Départ', zorder=5)
ax.plot(trajectory_vert[-1, 0], trajectory_vert[-1, 1], 'ro', markersize=12, label='Arrivée', zorder=5)
ax.legend()

# --- Horizontale ---
ax = axes[1]
ax.set_aspect('equal')
ax.set_xlim(-0.1, 1.1)
ax.set_ylim(-0.1, 3.1)
ax.set_xlabel('Largeur (m)', fontsize=11)
ax.set_ylabel('Hauteur (m)', fontsize=11)
ax.set_title(f'HORIZONTALE\n{stats_horiz["num_passes"]} passages - {stats_horiz["total_distance"]:.1f}m',
            fontsize=12, fontweight='bold')
ax.grid(True, alpha=0.3)

# Contour du mur
ax.plot(wall_x, wall_y, 'k-', linewidth=2)

# Trajectoire avec gradient
n_segments = len(trajectory_horiz) - 1
colors = plt.cm.plasma(np.linspace(0, 1, n_segments))
for i in range(n_segments):
    ax.plot(trajectory_horiz[i:i+2, 0], trajectory_horiz[i:i+2, 1],
           color=colors[i], linewidth=2, alpha=0.8)

ax.plot(trajectory_horiz[0, 0], trajectory_horiz[0, 1], 'go', markersize=12, label='Départ', zorder=5)
ax.plot(trajectory_horiz[-1, 0], trajectory_horiz[-1, 1], 'ro', markersize=12, label='Arrivée', zorder=5)
ax.legend()

plt.tight_layout()
plt.savefig('comparaison_orientations.png', dpi=300, bbox_inches='tight')
print("\nGraphique sauvegardé: comparaison_orientations.png")


# ==================== EFFET DU RAYON DE VIRAGE ====================
print("\n" + "="*70)
print("EFFET DU RAYON DE VIRAGE (orientation horizontale)")
print("="*70)

rayons = [0.0, 0.05, 0.10, 0.15]  # en mètres
trajectoires = []
stats_list = []

for rayon in rayons:
    print(f"\nRayon de virage: {rayon*100}cm")
    print("-" * 70)

    traj = WallPaintingTrajectory(
        wall_width=1.0,
        wall_height=3.0,
        tool_width=0.05,
        overlap_ratio=0.10,
        orientation='horizontal',
        turning_radius=rayon
    )

    trajectory = traj.generate()
    stats = traj.get_statistics()

    trajectoires.append(trajectory)
    stats_list.append(stats)

    traj.save(f"horizontal_virage_{int(rayon*100)}cm")


# ==================== VISUALISATION EFFET RAYON DE VIRAGE ====================
fig, axes = plt.subplots(2, 2, figsize=(14, 14))
axes = axes.flatten()

for idx, (rayon, traj, stats) in enumerate(zip(rayons, trajectoires, stats_list)):
    ax = axes[idx]
    ax.set_aspect('equal')
    ax.set_xlim(-0.15, 1.15)
    ax.set_ylim(-0.15, 3.15)
    ax.set_xlabel('Largeur (m)', fontsize=10)
    ax.set_ylabel('Hauteur (m)', fontsize=10)

    title = f'Rayon: {rayon*100:.0f}cm'
    if rayon == 0:
        title += ' (transitions directes)'
    ax.set_title(f'{title}\n{stats["num_points"]} points - {stats["total_distance"]:.1f}m',
                fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3)

    # Contour du mur
    ax.plot(wall_x, wall_y, 'k-', linewidth=2)

    # Trajectoire
    n_segments = len(traj) - 1
    colors = plt.cm.plasma(np.linspace(0, 1, n_segments))
    for i in range(n_segments):
        ax.plot(traj[i:i+2, 0], traj[i:i+2, 1],
               color=colors[i], linewidth=1.5, alpha=0.7)

    # Points de départ et d'arrivée
    ax.plot(traj[0, 0], traj[0, 1], 'go', markersize=10, zorder=5)
    ax.plot(traj[-1, 0], traj[-1, 1], 'ro', markersize=10, zorder=5)

    # Zoomer sur un virage si rayon > 0
    if rayon > 0 and len(traj) > 10:
        # Petit graphique inséré montrant un zoom sur un virage
        from mpl_toolkits.axes_grid1.inset_locator import inset_axes
        axins = inset_axes(ax, width="30%", height="30%", loc='upper right')

        # Trouver la zone de transition (grand changement en X ou Y)
        diffs = np.linalg.norm(np.diff(traj, axis=0), axis=1)
        if len(diffs) > 0:
            # Prendre une zone autour du milieu de la trajectoire
            mid_idx = len(traj) // 2
            zoom_start = max(0, mid_idx - 5)
            zoom_end = min(len(traj), mid_idx + 5)

            zoom_traj = traj[zoom_start:zoom_end]
            axins.plot(zoom_traj[:, 0], zoom_traj[:, 1], 'r-', linewidth=2)
            axins.set_aspect('equal')
            axins.grid(True, alpha=0.3)
            axins.set_xticks([])
            axins.set_yticks([])
            axins.set_title('Zoom virage', fontsize=8)

plt.tight_layout()
plt.savefig('effet_rayon_virage.png', dpi=300, bbox_inches='tight')
print("\nGraphique sauvegardé: effet_rayon_virage.png")


# ==================== COMPARAISON DÉTAILLÉE ====================
print("\n" + "="*70)
print("TABLEAU COMPARATIF")
print("="*70)

print("\n1. Comparaison Vertical vs Horizontal (rayon=0):")
print(f"{'Métrique':<30} {'Vertical':<15} {'Horizontal':<15}")
print("-" * 60)
print(f"{'Nombre de passages':<30} {stats_vert['num_passes']:<15} {stats_horiz['num_passes']:<15}")
print(f"{'Nombre de points':<30} {stats_vert['num_points']:<15} {stats_horiz['num_points']:<15}")
print(f"{'Distance totale (m)':<30} {stats_vert['total_distance']:<15.2f} {stats_horiz['total_distance']:<15.2f}")

print("\n2. Effet du rayon de virage (orientation horizontale):")
print(f"{'Rayon (cm)':<20} {'Points':<15} {'Distance (m)':<15}")
print("-" * 50)
for rayon, stats in zip(rayons, stats_list):
    print(f"{rayon*100:<20.0f} {stats['num_points']:<15} {stats['total_distance']:<15.2f}")


print("\n" + "="*70)
print("RÉSUMÉ")
print("="*70)
print("\nFichiers générés:")
print("  - comparaison_orientations.png (vertical vs horizontal)")
print("  - effet_rayon_virage.png (différents rayons)")
print("  - horizontal_virage_0cm.npy/csv")
print("  - horizontal_virage_5cm.npy/csv")
print("  - horizontal_virage_10cm.npy/csv")
print("  - horizontal_virage_15cm.npy/csv")

print("\nObservations:")
print("  1. Orientation verticale: plus de passages, distance similaire")
print("  2. Orientation horizontale: moins de passages (hauteur > largeur)")
print("  3. Rayon de virage > 0: augmente le nombre de points et la distance")
print("     (virages arrondis au lieu de transitions directes)")

print("\nUtilisation:")
print("  # Trajectoire verticale:")
print("  traj = WallPaintingTrajectory(orientation='vertical')")
print("  ")
print("  # Trajectoire horizontale:")
print("  traj = WallPaintingTrajectory(orientation='horizontal')")
print("  ")
print("  # Avec virages arrondis (10cm de rayon):")
print("  traj = WallPaintingTrajectory(orientation='horizontal', turning_radius=0.10)")
print("="*70)
