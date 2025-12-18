# walbot_coverage_planning

Package ROS2 pour la planification et l'exécution de trajectoires de peinture murale avec cuRobo.

## Description

Ce package génère des trajectoires de couverture 2D pour peindre un mur, les transforme en poses 3D contraintes dans le plan YZ, et les exécute sur le robot en utilisant les planificateurs de cuRobo.

## Utilisation

### 1. Sourcer l'environnement

```bash
source /home/ros2_ws/install/setup.bash
source /opt/ros/humble/setup.bash
```

### 2. Lancer cuRobo

```bash
ros2 launch curobo_ros gen_traj.launch.py
```
Attendre que le warmup de curobo soit fini.
### 3. Lancer le nœud de coverage

```bash
ros2 launch walbot_coverage_planning wall_coverage.launch.py
```

### 4. Déclencher l'exécution

```bash
ros2 topic pub --once /generate_and_execute std_msgs/msg/Bool "data: true"
```

## Configuration

### Paramètres du mur
- `wall_width`: Largeur du mur (m)
- `wall_height`: Hauteur du mur (m)
- `tool_width`: Largeur de l'outil (m)
- `tool_height`: Hauteur de l'outil (m)
- `overlap_ratio`: Ratio de recouvrement (0.0-1.0)
- `orientation`: Orientation de la trajectoire ('horizontal' ou 'vertical')

### Paramètres du robot
- `distance_to_wall`: Distance du TCP au mur (m)
- `target_orientation_qw/qx/qy/qz`: Orientation cible (quaternion)
- `home_joint_positions`: Position home en espace articulaire

### Modification des paramètres

**Option 1 - Via launch file:**
1. Modifier les valeurs dans `launch/wall_coverage.launch.py`
2. Recompiler le package:
   ```bash
   cd /home/ros2_ws
   colcon build --packages-select walbot_coverage_planning
   ```

**Option 2 - Via ROS params (runtime):**
```bash
ros2 param set /wall_coverage_node wall_width 1.0
ros2 param set /wall_coverage_node wall_height 2.0
```

## Visualisation RViz

Le nœud publie des marqueurs sur le topic `/wall_markers` qui affichent les 4 coins du mur en rouge.

Pour visualiser:
1. Ouvrir RViz
2. Ajouter un MarkerArray display
3. Topic: `/wall_markers`

## Workflow d'exécution

1. Déplacement à la position home (joint_space planner)
2. Ajout de l'obstacle ground
3. Publication des marqueurs des coins du mur
4. Génération de la trajectoire 2D avec Fields2Cover
5. Transformation en poses 3D avec contraintes YZ
6. Déplacement au premier waypoint (classic planner)
7. Planification de la trajectoire complète (multipoint planner)
8. Exécution de la trajectoire
