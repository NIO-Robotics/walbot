#=============================================================================
#     Trajectoire de peinture pour mur avec Fields2Cover
#     Classe réutilisable pour générer des trajectoires de peinture
#=============================================================================

import numpy as np
import matplotlib.pyplot as plt
import fields2cover as f2c
from typing import Tuple, Optional


class WallPaintingTrajectory:
    """
    Classe pour générer des trajectoires de peinture pour un mur rectangulaire.

    Utilise Fields2Cover pour créer des passages verticaux optimisés.
    """

    def __init__(self,
                 wall_width: float = 1.0,
                 wall_height: float = 3.0,
                 tool_width: float = 0.05,
                 tool_height: float = 0.20,
                 overlap_ratio: float = 0.10,
                 orientation: str = 'vertical',
                 turning_radius: float = 0.0):
        """
        Initialise le générateur de trajectoire.

        Args:
            wall_width: Largeur du mur en mètres (axe X)
            wall_height: Hauteur du mur en mètres (axe Y)
            tool_width: Largeur de l'outil de peinture en mètres
            tool_height: Hauteur de l'outil de peinture en mètres (pour info)
            overlap_ratio: Ratio de recouvrement entre passages (0.0 à 1.0)
            orientation: 'vertical' (haut-bas) ou 'horizontal' (gauche-droite)
            turning_radius: Rayon de virage en mètres (0 = pas de virage arrondi)
        """
        self.wall_width = wall_width
        self.wall_height = wall_height
        self.tool_width = tool_width
        self.tool_height = tool_height
        self.overlap_ratio = overlap_ratio
        self.orientation = orientation.lower()
        self.turning_radius = turning_radius

        # Validation
        if self.orientation not in ['vertical', 'horizontal']:
            raise ValueError("orientation doit être 'vertical' ou 'horizontal'")
        if self.turning_radius < 0:
            raise ValueError("turning_radius doit être >= 0")

        # Largeur effective avec recouvrement
        self.effective_width = tool_width * (1.0 - overlap_ratio)

        # Données générées
        self.wall_cell = None
        self.swaths = None
        self.trajectory = None
        self.path = None  # Path avec virages si turning_radius > 0

        print(f"WallPaintingTrajectory initialisée:")
        print(f"  - Mur: {wall_width}m x {wall_height}m")
        print(f"  - Outil: {tool_width*100}cm de large")
        print(f"  - Recouvrement: {overlap_ratio*100}%")
        print(f"  - Largeur effective: {self.effective_width*100}cm")
        print(f"  - Orientation: {self.orientation}")
        print(f"  - Rayon de virage: {turning_radius*100}cm" if turning_radius > 0 else "  - Rayon de virage: Aucun (transitions directes)")
        print(f"  - Passages estimés: {self._estimate_num_passes()}\n")

    def _estimate_num_passes(self) -> int:
        """Estime le nombre de passages nécessaires."""
        if self.orientation == 'vertical':
            # Passages verticaux: on couvre la largeur
            return int(np.ceil(self.wall_width / self.effective_width))
        else:
            # Passages horizontaux: on couvre la hauteur
            return int(np.ceil(self.wall_height / self.effective_width))

    def _create_wall_cell(self) -> f2c.Cell:
        """Crée la géométrie du mur sous forme de Cell."""
        wall_ring = f2c.LinearRing()
        wall_points = [
            f2c.Point(0, 0),                           # bas-gauche
            f2c.Point(self.wall_width, 0),             # bas-droite
            f2c.Point(self.wall_width, self.wall_height), # haut-droite
            f2c.Point(0, self.wall_height),            # haut-gauche
            f2c.Point(0, 0)                            # fermeture
        ]
        [wall_ring.addGeometry(p) for p in wall_points]

        wall_cell = f2c.Cell()
        wall_cell.addRing(wall_ring)

        return wall_cell

    def _generate_swaths(self) -> tuple:
        """
        Génère les swaths (passages) selon l'orientation.

        Returns:
            Tuple (robot, swaths)
        """
        # Angle selon l'orientation
        if self.orientation == 'vertical':
            angle = np.pi / 2.0  # Passages verticaux (90°)
        else:
            angle = 0.0  # Passages horizontaux (0°)

        # Robot avec largeur effective et rayon de virage
        robot = f2c.Robot(self.effective_width, 1.0)
        if self.turning_radius > 0:
            robot.setMinTurningRadius(self.turning_radius)

        # Générateur de swaths
        swath_generator = f2c.SG_BruteForce()
        swaths = swath_generator.generateSwaths(angle, self.effective_width, self.wall_cell)

        return robot, swaths

    def _reorder_swaths(self) -> list:
        """
        Réordonne les swaths selon l'orientation.
        - Vertical: de gauche à droite (tri par X)
        - Horizontal: de haut en bas (tri par Y, ordre décroissant)

        Returns:
            Liste de tuples (position, swath, index)
        """
        swath_list = []
        for i in range(self.swaths.size()):
            swath = self.swaths.at(i)
            path = swath.getPath()
            start_point = path.getGeometry(0)

            if self.orientation == 'vertical':
                # Passages verticaux: trier par X (gauche à droite)
                pos = start_point.getX()
            else:
                # Passages horizontaux: trier par Y (haut en bas)
                pos = start_point.getY()

            swath_list.append((pos, swath, i))

        # Trier selon l'orientation
        if self.orientation == 'vertical':
            swath_list.sort(key=lambda x: x[0])  # Croissant (gauche à droite)
        else:
            swath_list.sort(key=lambda x: -x[0])  # Décroissant (haut en bas)

        return swath_list

    def _create_trajectory(self, robot: f2c.Robot, swath_list: list) -> np.ndarray:
        """
        Crée la trajectoire ordonnée à partir des swaths.

        Pour orientation verticale: chaque passage descend de haut en bas
        Pour orientation horizontale: chaque passage va de gauche à droite

        Args:
            robot: Robot avec configuration de virage
            swath_list: Liste ordonnée de swaths

        Returns:
            Array numpy de shape (N, 2) contenant les points (x, y)
        """
        # Si on a un rayon de virage, utiliser le path planning
        if self.turning_radius > 0:
            return self._create_trajectory_with_turns(robot, swath_list)
        else:
            return self._create_trajectory_simple(swath_list)

    def _create_trajectory_simple(self, swath_list: list) -> np.ndarray:
        """
        Crée une trajectoire simple sans virages arrondis.

        Args:
            swath_list: Liste ordonnée de swaths

        Returns:
            Array numpy de shape (N, 2) contenant les points (x, y)
        """
        trajectory_points = []

        for pos, swath, original_idx in swath_list:
            path = swath.getPath()

            if self.orientation == 'vertical':
                # Passages verticaux: on veut toujours descendre (haut -> bas)
                start_y = path.getGeometry(0).getY()
                end_y = path.getGeometry(path.size() - 1).getY()

                # Si le swath monte, inverser l'ordre
                if start_y < end_y:
                    for i in range(path.size() - 1, -1, -1):
                        point = path.getGeometry(i)
                        trajectory_points.append([point.getX(), point.getY()])
                else:
                    for i in range(path.size()):
                        point = path.getGeometry(i)
                        trajectory_points.append([point.getX(), point.getY()])

            else:
                # Passages horizontaux: on veut aller de gauche à droite
                start_x = path.getGeometry(0).getX()
                end_x = path.getGeometry(path.size() - 1).getX()

                # Si le swath va de droite à gauche, inverser
                if start_x > end_x:
                    for i in range(path.size() - 1, -1, -1):
                        point = path.getGeometry(i)
                        trajectory_points.append([point.getX(), point.getY()])
                else:
                    for i in range(path.size()):
                        point = path.getGeometry(i)
                        trajectory_points.append([point.getX(), point.getY()])

        return np.array(trajectory_points)

    def _create_trajectory_with_turns(self, robot: f2c.Robot, swath_list: list) -> np.ndarray:
        """
        Crée une trajectoire avec virages arrondis en utilisant le path planning.

        Args:
            robot: Robot avec rayon de virage configuré
            swath_list: Liste ordonnée de swaths

        Returns:
            Array numpy de shape (N, 2) contenant les points (x, y)
        """
        # Reconstruire les swaths dans l'ordre avec bonne direction
        ordered_swaths = f2c.Swaths()

        for pos, swath, original_idx in swath_list:
            path = swath.getPath()

            # Déterminer si on doit inverser le swath
            should_reverse = False

            if self.orientation == 'vertical':
                # Vertical: on veut descendre (Y décroissant)
                start_y = path.getGeometry(0).getY()
                end_y = path.getGeometry(path.size() - 1).getY()
                should_reverse = (start_y < end_y)
            else:
                # Horizontal: on veut aller vers la droite (X croissant)
                start_x = path.getGeometry(0).getX()
                end_x = path.getGeometry(path.size() - 1).getX()
                should_reverse = (start_x > end_x)

            # Créer le swath avec la bonne direction
            if should_reverse:
                # Inverser les points du path
                reversed_line = f2c.LineString()
                for i in range(path.size() - 1, -1, -1):
                    point = path.getGeometry(i)
                    reversed_line.addPoint(point.getX(), point.getY())
                new_swath = f2c.Swath(reversed_line, swath.getWidth())
                ordered_swaths.push_back(new_swath)
            else:
                ordered_swaths.push_back(swath)

        # Utiliser le path planning avec courbes de Dubins
        path_planner = f2c.PP_PathPlanning()
        dubins = f2c.PP_DubinsCurvesCC()  # Dubins avec courbure continue (plus lisse)
        self.path = path_planner.planPath(robot, ordered_swaths, dubins)

        # Extraire les points de la trajectoire
        trajectory_points = []
        for i in range(self.path.size()):
            state = self.path.getState(i)
            point = state.point
            trajectory_points.append([point.getX(), point.getY()])

        return np.array(trajectory_points)

    def generate(self) -> np.ndarray:
        """
        Génère la trajectoire complète.

        Returns:
            Array numpy de shape (N, 2) contenant les points (x, y) de la trajectoire
        """
        print("Génération de la trajectoire...")

        # 1. Créer le mur
        self.wall_cell = self._create_wall_cell()
        print(f"  - Surface du mur: {self.wall_cell.area()} m²")

        # 2. Générer les swaths
        robot, self.swaths = self._generate_swaths()
        print(f"  - Swaths générés: {self.swaths.size()}")

        # 3. Réordonner les swaths
        swath_list = self._reorder_swaths()

        # 4. Créer la trajectoire
        self.trajectory = self._create_trajectory(robot, swath_list)

        # 5. Afficher les infos
        print(f"  - Points dans la trajectoire: {len(self.trajectory)}")

        start_desc = "haut-gauche" if self.orientation == 'vertical' else "haut-gauche"
        print(f"  - Départ: X={self.trajectory[0,0]:.3f}m, Y={self.trajectory[0,1]:.3f}m ({start_desc})")
        print(f"  - Arrivée: X={self.trajectory[-1,0]:.3f}m, Y={self.trajectory[-1,1]:.3f}m")

        distance = np.sum(np.linalg.norm(np.diff(self.trajectory, axis=0), axis=1))
        print(f"  - Distance totale: {distance:.2f}m")

        if self.turning_radius > 0:
            print(f"  - Trajectoire avec virages arrondis (rayon: {self.turning_radius*100}cm)\n")
        else:
            print(f"  - Trajectoire avec transitions directes\n")

        return self.trajectory

    def get_trajectory(self) -> np.ndarray:
        """
        Retourne la trajectoire générée.

        Returns:
            Array numpy de shape (N, 2)
        """
        if self.trajectory is None:
            raise ValueError("La trajectoire n'a pas encore été générée. Appelez generate() d'abord.")
        return self.trajectory

    def plot(self, save_path: Optional[str] = None, show: bool = True):
        """
        Visualise la trajectoire avec matplotlib.

        Args:
            save_path: Chemin pour sauvegarder l'image (optionnel)
            show: Si True, affiche le graphique
        """
        if self.trajectory is None:
            raise ValueError("La trajectoire n'a pas encore été générée. Appelez generate() d'abord.")

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 8))

        # --- Graphique 1: Vue d'ensemble avec les swaths ---
        ax1.set_aspect('equal')
        ax1.set_xlim(-0.1, self.wall_width + 0.1)
        ax1.set_ylim(-0.1, self.wall_height + 0.1)
        ax1.set_xlabel('Largeur (m)', fontsize=12)
        ax1.set_ylabel('Hauteur (m)', fontsize=12)
        ax1.set_title('Vue d\'ensemble - Passages de peinture', fontsize=14, fontweight='bold')
        ax1.grid(True, alpha=0.3)

        # Contour du mur
        wall_x = [0, self.wall_width, self.wall_width, 0, 0]
        wall_y = [0, 0, self.wall_height, self.wall_height, 0]
        ax1.plot(wall_x, wall_y, 'k-', linewidth=2, label='Mur')

        # Dessiner chaque swath
        for i in range(self.swaths.size()):
            swath = self.swaths.at(i)
            path = swath.getPath()
            x_coords = [path.getGeometry(j).getX() for j in range(path.size())]
            y_coords = [path.getGeometry(j).getY() for j in range(path.size())]
            ax1.plot(x_coords, y_coords, 'b-', linewidth=1.5, alpha=0.6)

        # Points de départ et d'arrivée
        ax1.plot(self.trajectory[0,0], self.trajectory[0,1], 'go',
                markersize=12, label='Départ', zorder=5)
        ax1.plot(self.trajectory[-1,0], self.trajectory[-1,1], 'ro',
                markersize=12, label='Arrivée', zorder=5)
        ax1.legend(fontsize=10)

        # --- Graphique 2: Trajectoire ordonnée ---
        ax2.set_aspect('equal')
        ax2.set_xlim(-0.1, self.wall_width + 0.1)
        ax2.set_ylim(-0.1, self.wall_height + 0.1)
        ax2.set_xlabel('Largeur (m)', fontsize=12)
        ax2.set_ylabel('Hauteur (m)', fontsize=12)
        ax2.set_title('Trajectoire ordonnée', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3)

        # Contour du mur
        ax2.plot(wall_x, wall_y, 'k-', linewidth=2, label='Mur')

        # Trajectoire avec gradient de couleur
        n_segments = len(self.trajectory) - 1
        colors = plt.cm.viridis(np.linspace(0, 1, n_segments))

        for i in range(n_segments):
            ax2.plot(self.trajectory[i:i+2, 0], self.trajectory[i:i+2, 1],
                    color=colors[i], linewidth=2, alpha=0.8)

        # Marquer les changements de direction
        x_diff = np.diff(self.trajectory[:, 0])
        direction_changes = np.where(np.abs(x_diff) > self.effective_width * 0.5)[0] + 1
        direction_changes = np.concatenate([[0], direction_changes, [len(self.trajectory) - 1]])

        for i, idx in enumerate(direction_changes):
            if i == 0:
                ax2.plot(self.trajectory[idx, 0], self.trajectory[idx, 1],
                        'go', markersize=12, label='Départ', zorder=5)
                ax2.text(self.trajectory[idx, 0] - 0.05, self.trajectory[idx, 1] + 0.1,
                        '1', fontsize=10, fontweight='bold', ha='right')
            elif i == len(direction_changes) - 1:
                ax2.plot(self.trajectory[idx, 0], self.trajectory[idx, 1],
                        'ro', markersize=12, label='Arrivée', zorder=5)
            else:
                ax2.plot(self.trajectory[idx, 0], self.trajectory[idx, 1],
                        'yo', markersize=8, zorder=5)
                ax2.text(self.trajectory[idx, 0], self.trajectory[idx, 1] + 0.1,
                        f'{i+1}', fontsize=9, fontweight='bold', ha='center')

        ax2.legend(fontsize=10)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Graphique sauvegardé: {save_path}")

        if show:
            plt.show()
        else:
            plt.close()

    def plot_f2c(self, save_path: str = "wall_painting_f2c.png"):
        """
        Visualise avec l'outil natif de Fields2Cover.

        Args:
            save_path: Chemin pour sauvegarder l'image
        """
        if self.swaths is None:
            raise ValueError("Les swaths n'ont pas été générés. Appelez generate() d'abord.")

        wall_cells = f2c.Cells()
        wall_cells.addGeometry(self.wall_cell)

        f2c.Visualizer.figure()
        f2c.Visualizer.plot(wall_cells)
        f2c.Visualizer.plot(self.swaths)
        f2c.Visualizer.save(save_path)
        print(f"Visualisation Fields2Cover sauvegardée: {save_path}")

    def save(self, prefix: str = "wall_painting_trajectory"):
        """
        Sauvegarde la trajectoire dans des fichiers.

        Args:
            prefix: Préfixe pour les noms de fichiers
        """
        if self.trajectory is None:
            raise ValueError("La trajectoire n'a pas encore été générée. Appelez generate() d'abord.")

        # Sauvegarder en .npy
        npy_path = f"{prefix}.npy"
        np.save(npy_path, self.trajectory)
        print(f"Trajectoire sauvegardée: {npy_path}")

        # Sauvegarder en .csv
        csv_path = f"{prefix}.csv"
        np.savetxt(csv_path, self.trajectory, delimiter=',',
                  header='X(m),Y(m)', comments='', fmt='%.6f')
        print(f"Trajectoire sauvegardée: {csv_path}")

    def get_statistics(self) -> dict:
        """
        Retourne des statistiques sur la trajectoire.

        Returns:
            Dictionnaire avec les statistiques
        """
        if self.trajectory is None:
            raise ValueError("La trajectoire n'a pas encore été générée. Appelez generate() d'abord.")

        distance = np.sum(np.linalg.norm(np.diff(self.trajectory, axis=0), axis=1))

        return {
            'wall_width': self.wall_width,
            'wall_height': self.wall_height,
            'wall_area': self.wall_width * self.wall_height,
            'tool_width': self.tool_width,
            'overlap_ratio': self.overlap_ratio,
            'effective_width': self.effective_width,
            'orientation': self.orientation,
            'turning_radius': self.turning_radius,
            'num_passes': self.swaths.size() if self.swaths else 0,
            'num_points': len(self.trajectory),
            'total_distance': distance,
            'start_point': tuple(self.trajectory[0]),
            'end_point': tuple(self.trajectory[-1])
        }


# ===================== EXEMPLE D'UTILISATION =====================
# if __name__ == "__main__":
#     # Créer le générateur de trajectoire
#     traj_gen = WallPaintingTrajectory(
#         wall_width=1.0,      # 1m de large
#         wall_height=3.0,     # 3m de haut
#         tool_width=0.05,     # 5cm de large
#         tool_height=0.20,    # 20cm de haut
#         overlap_ratio=0.10   # 10% de recouvrement
#     )

#     # Générer la trajectoire
#     trajectory = traj_gen.generate()

#     # Afficher les statistiques
#     stats = traj_gen.get_statistics()
#     print("\n" + "="*60)
#     print("STATISTIQUES:")
#     print(f"  - Nombre de passages: {stats['num_passes']}")
#     print(f"  - Nombre de points: {stats['num_points']}")
#     print(f"  - Distance totale: {stats['total_distance']:.2f}m")
#     print(f"  - Surface couverte: {stats['wall_area']:.2f}m²")
#     print("="*60 + "\n")

#     # Visualiser
#     traj_gen.plot(save_path="wall_painting_trajectory.png", show=True)
#     traj_gen.plot_f2c()

#     # Sauvegarder
#     traj_gen.save()

#     print("\n" + "="*60)
#     print("UTILISATION:")
#     print("  # Modifier le recouvrement pour tests:")
#     print("  traj_gen = WallPaintingTrajectory(overlap_ratio=0.15)")
#     print("  trajectory = traj_gen.generate()")
#     print("  ")
#     print("  # Accéder à la trajectoire:")
#     print("  trajectory = traj_gen.get_trajectory()  # numpy array (N, 2)")
#     print("="*60)
