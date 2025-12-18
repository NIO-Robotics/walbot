#!/usr/bin/env python3
#=============================================================================
#     Exemple d'utilisation de WallPaintingTrajectory
#=============================================================================

from wall_painting_trajectory import WallPaintingTrajectory
import numpy as np
import matplotlib.pyplot as plt

print("="*70)
print("EXEMPLE 1: Configuration de base (30% de recouvrement)")
print("="*70)

# Créer une trajectoire avec 10% de recouvrement
traj1 = WallPaintingTrajectory(
    wall_width=1.0,
    wall_height=3.0,
    tool_width=0.20,
    tool_height=0.05,
    overlap_ratio=0.30,
    orientation = 'horizontal',
)

# Générer la trajectoire
trajectory1 = traj1.generate()

# Afficher les statistiques
stats1 = traj1.get_statistics()
print(f"\nRésultats:")
print(f"  Nombre de passages: {stats1['num_passes']}")
print(f"  Distance totale: {stats1['total_distance']:.2f}m")

# Sauvegarder
traj1.save("exemple_overlap_10")
traj1.plot(save_path="exemple_overlap_10.png", show=False)


print("\n" + "="*70)
print("EXEMPLE 2: Test avec 15% de recouvrement")
print("="*70)

# Créer une trajectoire avec 15% de recouvrement
traj2 = WallPaintingTrajectory(
    wall_width=1.0,
    wall_height=3.0,
    tool_width=0.05,
    tool_height=0.20,
    overlap_ratio=0.15,
    orientation = 'horizontal',
)

# Générer la trajectoire
trajectory2 = traj2.generate()

# Afficher les statistiques
stats2 = traj2.get_statistics()
print(f"\nRésultats:")
print(f"  Nombre de passages: {stats2['num_passes']}")
print(f"  Distance totale: {stats2['total_distance']:.2f}m")

# Sauvegarder
traj2.save("exemple_overlap_15")
traj2.plot(save_path="exemple_overlap_15.png", show=False)


print("\n" + "="*70)
print("EXEMPLE 3: Test avec 5% de recouvrement")
print("="*70)

# Créer une trajectoire avec 5% de recouvrement
traj3 = WallPaintingTrajectory(
    wall_width=1.0,
    wall_height=3.0,
    tool_width=0.05,
    tool_height=0.20,
    overlap_ratio=0.05,
    orientation = 'horizontal',
)

# Générer la trajectoire
trajectory3 = traj3.generate()

# Afficher les statistiques
stats3 = traj3.get_statistics()
print(f"\nRésultats:")
print(f"  Nombre de passages: {stats3['num_passes']}")
print(f"  Distance totale: {stats3['total_distance']:.2f}m")

# Sauvegarder
traj3.save("exemple_overlap_05")
traj3.plot(save_path="exemple_overlap_05.png", show=False)


print("\n" + "="*70)
print("COMPARAISON DES TROIS CONFIGURATIONS")
print("="*70)

# Créer un graphique comparatif
fig, axes = plt.subplots(1, 3, figsize=(18, 6))

configs = [
    (trajectory1, stats1, "10% recouvrement", traj1),
    (trajectory2, stats2, "15% recouvrement", traj2),
    (trajectory3, stats3, "5% recouvrement", traj3)
]

for idx, (traj, stats, title, traj_obj) in enumerate(configs):
    ax = axes[idx]
    ax.set_aspect('equal')
    ax.set_xlim(-0.05, 1.05)
    ax.set_ylim(-0.1, 3.1)
    ax.set_xlabel('Largeur (m)', fontsize=10)
    ax.set_ylabel('Hauteur (m)', fontsize=10)
    ax.set_title(f'{title}\n{stats["num_passes"]} passages - {stats["total_distance"]:.1f}m',
                fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3)

    # Contour du mur
    wall_x = [0, 1, 1, 0, 0]
    wall_y = [0, 0, 3, 3, 0]
    ax.plot(wall_x, wall_y, 'k-', linewidth=2)

    # Trajectoire
    n_segments = len(traj) - 1
    colors = plt.cm.viridis(np.linspace(0, 1, n_segments))

    for i in range(n_segments):
        ax.plot(traj[i:i+2, 0], traj[i:i+2, 1],
               color=colors[i], linewidth=1.5, alpha=0.7)

    # Points de départ et d'arrivée
    ax.plot(traj[0, 0], traj[0, 1], 'go', markersize=10, zorder=5)
    ax.plot(traj[-1, 0], traj[-1, 1], 'ro', markersize=10, zorder=5)

plt.tight_layout()
plt.savefig('comparaison_recouvrements.png', dpi=300, bbox_inches='tight')
print("Graphique comparatif sauvegardé: comparaison_recouvrements.png")


print("\n" + "="*70)
print("EXEMPLE 4: Accéder aux données de trajectoire")
print("="*70)

# Accéder à la trajectoire sous forme de numpy array
trajectory = traj1.get_trajectory()

print(f"\nFormat des données:")
print(f"  Type: {type(trajectory)}")
print(f"  Shape: {trajectory.shape}")
print(f"  Dtype: {trajectory.dtype}")

print(f"\nPremiers 10 points (X, Y) en mètres:")
for i in range(min(10, len(trajectory))):
    print(f"  Point {i+1}: X={trajectory[i,0]:.4f}m, Y={trajectory[i,1]:.4f}m")

print(f"\n...")
print(f"\nDerniers 3 points:")
for i in range(max(0, len(trajectory)-3), len(trajectory)):
    print(f"  Point {i+1}: X={trajectory[i,0]:.4f}m, Y={trajectory[i,1]:.4f}m")


print("\n" + "="*70)
print("EXEMPLE 5: Calcul de distances entre points")
print("="*70)

# Calculer les distances entre chaque point successif
distances = np.linalg.norm(np.diff(trajectory, axis=0), axis=1)

print(f"\nStatistiques des distances entre points:")
print(f"  Distance min: {np.min(distances):.4f}m")
print(f"  Distance max: {np.max(distances):.4f}m")
print(f"  Distance moyenne: {np.mean(distances):.4f}m")
print(f"  Distance médiane: {np.median(distances):.4f}m")

# Identifier les transitions (grands sauts entre passages)
threshold = 0.1  # 10cm
transitions = np.where(distances > threshold)[0]
print(f"\nNombre de transitions entre passages: {len(transitions)}")
if len(transitions) > 0:
    print(f"  Première transition: entre point {transitions[0]} et {transitions[0]+1}")
    print(f"  Distance: {distances[transitions[0]]:.4f}m")


print("\n" + "="*70)
print("RÉSUMÉ")
print("="*70)
print("\nFichiers générés:")
print("  - exemple_overlap_10.npy/csv (10% recouvrement)")
print("  - exemple_overlap_15.npy/csv (15% recouvrement)")
print("  - exemple_overlap_05.npy/csv (5% recouvrement)")
print("  - comparaison_recouvrements.png (graphique comparatif)")
print("\nVous pouvez maintenant:")
print("  1. Charger les trajectoires: np.load('exemple_overlap_10.npy')")
print("  2. Tester d'autres valeurs de recouvrement")
print("  3. Utiliser les données pour calculer vitesses et accélérations")
print("="*70)
