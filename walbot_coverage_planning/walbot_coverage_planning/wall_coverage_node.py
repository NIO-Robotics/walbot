#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Vector3, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from curobo_msgs.srv import TrajectoryGeneration, SetPlanner, AddObject
from curobo_msgs.action import SendTrajectory
from walbot_coverage_planning.wall_painting_trajectory import WallPaintingTrajectory
import numpy as np


class WallCoverageNode(Node):
    def __init__(self, executor=None):
        super().__init__('wall_coverage_node')

        # Stocker l'executor pour l'utiliser dans spin_until_future_complete
        self.executor = executor

        # Créer les callback groups
        # ReentrantCallbackGroup permet l'exécution simultanée de callbacks
        self.client_cb_group = ReentrantCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        # Déclarer tous les paramètres
        self._declare_parameters()

        # Créer les clients
        self._setup_curobo_clients()

        # Créer le publisher pour les marqueurs RViz
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'wall_markers',
            10
        )

        # Créer le subscriber pour déclencher la génération
        self.trigger_sub = self.create_subscription(
            Bool,
            'generate_and_execute',
            self.generate_and_execute_callback,
            10,
            callback_group=self.service_cb_group
        )

        self.get_logger().info('Wall Coverage Node initialized')

    def _declare_parameters(self):
        """Déclarer tous les paramètres ROS2"""
        # Wall parameters
        self.declare_parameter('wall_width', 1.0)
        self.declare_parameter('wall_height', 3.0)
        self.declare_parameter('tool_width', 0.05)
        self.declare_parameter('tool_height', 0.20)
        self.declare_parameter('overlap_ratio', 0.10)
        self.declare_parameter('orientation', 'vertical')
        self.declare_parameter('turning_radius', 0.0)

        # Robot parameters
        self.declare_parameter('distance_to_wall', 0.5)
        self.declare_parameter('target_orientation_qw', 1.0)
        self.declare_parameter('target_orientation_qx', 0.0)
        self.declare_parameter('target_orientation_qy', 0.0)
        self.declare_parameter('target_orientation_qz', 0.0)

        # Curobo parameters
        self.declare_parameter('curobo_namespace', 'unified_planner')
        self.declare_parameter('service_timeout', 60.0)
        self.declare_parameter('action_timeout', 120.0)
        self.declare_parameter('connect_waypoints', True)
        self.declare_parameter('time_dilation_factor', 1.0)

        # Home position (joint space)
        self.declare_parameter('home_joint_positions', [
            -0.07784457504749298,
            -0.18099723756313324,
            1.6212745904922485,
            -2.5986220836639404,
            -0.1518784463405609,
            -0.537718653678894
        ])

    def _setup_curobo_clients(self):
        """Créer les clients pour communiquer avec curobo_ros"""
        namespace = self.get_parameter('curobo_namespace').value

        # Service client pour planification
        self.traj_client = self.create_client(
            TrajectoryGeneration,
            f'/{namespace}/generate_trajectory',
            callback_group=self.client_cb_group
        )

        # Service client pour sélectionner le planner multipoint
        self.set_planner_client = self.create_client(
            SetPlanner,
            f'/{namespace}/set_planner',
            callback_group=self.client_cb_group
        )

        # Service client pour ajouter des objets/obstacles
        self.add_object_client = self.create_client(
            AddObject,
            f'/{namespace}/add_object',
            callback_group=self.client_cb_group
        )

        # Action client pour exécution (créé à la demande pour éviter les erreurs RCL)
        self.execute_client = None

    def generate_and_execute_callback(self, msg):
        """Callback du topic: génère et exécute la trajectoire"""
        if not msg.data:
            self.get_logger().info("Received False on trigger topic, ignoring")
            return

        self.get_logger().info("=== Starting trajectory generation and execution ===")

        try:
            # 1. Aller à la position home
            if not self._move_to_home():
                self.get_logger().error("Failed to move to home position")
                return

            # 2. Ajouter l'obstacle ground
            if not self._add_ground_obstacle():
                self.get_logger().error("Failed to add ground obstacle")
                return

            # 2b. Publier les marqueurs des coins du mur
            self._publish_wall_corners()

            # 3. Générer trajectoire 2D
            trajectory_2d = self._generate_2d_trajectory()
            if trajectory_2d is None:
                self.get_logger().error("Failed to generate 2D trajectory")
                return

            # 4. Transformer en poses 3D avec contraintes
            poses_3d, constraints = self._transform_to_3d_poses(trajectory_2d)

            # 5. Déplacer le robot au premier waypoint avec le planner classic
            if not self._move_to_first_waypoint(poses_3d[0]):
                self.get_logger().error("Failed to move to first waypoint")
                return

            # 6. Sélectionner le planner multipoint pour le reste de la trajectoire
            if not self._select_multipoint_planner():
                self.get_logger().error("Failed to select multi_point planner")
                return

            # 7. Envoyer à curobo pour planning (tous les waypoints)
            if not self._plan_trajectory(poses_3d, constraints):
                self.get_logger().error("Failed to plan trajectory with curobo")
                return

            # 8. Exécuter la trajectoire
            if not self._execute_trajectory():
                self.get_logger().error("Failed to execute trajectory")
                return

            self.get_logger().info(f"=== Successfully executed {len(poses_3d)} waypoints ===")

        except Exception as e:
            self.get_logger().error(f"Exception during execution: {e}")

    def _move_to_home(self) -> bool:
        """Déplacer le robot à la position home en utilisant joint_space planner"""
        self.get_logger().info("Moving to home position...")

        # 1. Sélectionner le planner joint_space
        if not self.set_planner_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("set_planner service not available")
            return False

        planner_request = SetPlanner.Request()
        planner_request.planner_type = 5  # JOINT_SPACE

        future = self.set_planner_client.call_async(planner_request)

        # Attendre le future avec une boucle Rate (évite les race conditions)
        rate = self.create_rate(100)  # 100 Hz
        timeout = 5.0
        start_time = self.get_clock().now()
        while not future.done():
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().error("set_planner (joint_space) timeout")
                return False
            rate.sleep()

        if not future.done():
            self.get_logger().error("set_planner (joint_space) timeout")
            return False

        planner_response = future.result()
        if not planner_response.success:
            self.get_logger().error(f"Failed to select joint_space planner: {planner_response.message}")
            return False

        self.get_logger().info("Joint space planner selected")

        # 2. Planifier vers la position home
        home_positions = self.get_parameter('home_joint_positions').value

        if not self.traj_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("generate_trajectory service not available")
            return False

        traj_request = TrajectoryGeneration.Request()
        traj_request.target_joint_positions = home_positions

        self.get_logger().info(f"Planning to home position: {home_positions}")
        traj_future = self.traj_client.call_async(traj_request)

        # Attendre avec Rate
        rate = self.create_rate(100)
        timeout = self.get_parameter('service_timeout').value
        start_time = self.get_clock().now()
        while not traj_future.done():
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().error("Home trajectory planning timeout")
                return False
            rate.sleep()

        if not traj_future.done():
            self.get_logger().error("Home trajectory planning timeout")
            return False

        traj_response = traj_future.result()
        if not traj_response.success:
            self.get_logger().error(f"Failed to plan to home: {traj_response.message}")
            return False

        self.get_logger().info("Home trajectory planned successfully")

        # 3. Exécuter le mouvement vers home
        if not self._execute_trajectory():
            self.get_logger().error("Failed to execute home trajectory")
            return False

        self.get_logger().info("Robot moved to home position successfully")
        return True

    def _move_to_first_waypoint(self, first_pose: Pose) -> bool:
        """Déplacer le robot au premier waypoint avec le planner classic"""
        self.get_logger().info("Moving to first waypoint with classic planner...")

        # 1. Sélectionner le planner classic
        if not self.set_planner_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("set_planner service not available")
            return False

        planner_request = SetPlanner.Request()
        planner_request.planner_type = 0  # CLASSIC

        future = self.set_planner_client.call_async(planner_request)

        # Attendre le future avec une boucle Rate
        rate = self.create_rate(100)
        timeout = 5.0
        start_time = self.get_clock().now()
        while not future.done():
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().error("set_planner (classic) timeout")
                return False
            rate.sleep()

        if not future.done():
            self.get_logger().error("set_planner (classic) timeout")
            return False

        planner_response = future.result()
        if not planner_response.success:
            self.get_logger().error(f"Failed to select classic planner: {planner_response.message}")
            return False

        self.get_logger().info("Classic planner selected")

        # 2. Planifier vers le premier waypoint
        if not self.traj_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("generate_trajectory service not available")
            return False

        traj_request = TrajectoryGeneration.Request()
        traj_request.target_pose = first_pose

        self.get_logger().info(f"Planning to first waypoint: x={first_pose.position.x:.3f}, y={first_pose.position.y:.3f}, z={first_pose.position.z:.3f}")
        traj_future = self.traj_client.call_async(traj_request)

        # Attendre avec Rate
        rate = self.create_rate(100)
        timeout = self.get_parameter('service_timeout').value
        start_time = self.get_clock().now()
        while not traj_future.done():
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().error("First waypoint trajectory planning timeout")
                return False
            rate.sleep()

        if not traj_future.done():
            self.get_logger().error("First waypoint trajectory planning timeout")
            return False

        traj_response = traj_future.result()
        if not traj_response.success:
            self.get_logger().error(f"Failed to plan to first waypoint: {traj_response.message}")
            return False

        self.get_logger().info("First waypoint trajectory planned successfully")

        # 3. Exécuter le mouvement vers le premier waypoint
        if not self._execute_trajectory():
            self.get_logger().error("Failed to execute first waypoint trajectory")
            return False

        self.get_logger().info("Robot moved to first waypoint successfully")
        return True

    def _add_ground_obstacle(self) -> bool:
        """Ajouter un obstacle ground (sol) de 10m x 10m x 20cm"""
        if not self.add_object_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("add_object service not available")
            return False

        request = AddObject.Request()
        request.type = 0  # CUBOID
        request.name = "ground"

        # Position: centré en x,y et à -10cm en z pour que le dessus soit à z=0
        request.pose.position.x = 0.0
        request.pose.position.y = 0.0
        request.pose.position.z = -0.3

        # Orientation: identité (pas de rotation)
        request.pose.orientation.w = 1.0
        request.pose.orientation.x = 0.0
        request.pose.orientation.y = 0.0
        request.pose.orientation.z = 0.0

        # Dimensions: 10m x 10m x 20cm
        request.dimensions.x = 10.0
        request.dimensions.y = 10.0
        request.dimensions.z = 0.2

        # Couleur: gris
        request.color.r = 0.5
        request.color.g = 0.5
        request.color.b = 0.5
        request.color.a = 1.0

        future = self.add_object_client.call_async(request)

        # Attendre avec Rate
        rate = self.create_rate(100)
        timeout = 5.0
        start_time = self.get_clock().now()
        while not future.done():
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().error("add_object service timeout")
                return False
            rate.sleep()

        if not future.done():
            self.get_logger().error("add_object service timeout")
            return False

        response = future.result()
        if response.success:
            self.get_logger().info("Ground obstacle added successfully")
            return True
        else:
            self.get_logger().error(f"Failed to add ground: {response.message}")
            return False

    def _publish_wall_corners(self):
        """Publier les marqueurs des 4 coins du mur en rouge pour RViz"""
        wall_width = self.get_parameter('wall_width').value
        wall_height = self.get_parameter('wall_height').value
        distance_to_wall = self.get_parameter('distance_to_wall').value

        # Calculer les 4 coins du mur (même référentiel que la trajectoire)
        # Le mur est dans le plan YZ, centré en Y
        z_offset = 0.5  # Même offset que dans _transform_to_3d_poses
        corners = [
            Point(x=distance_to_wall, y=-wall_width/2, z=z_offset),                    # Coin bas-gauche
            Point(x=distance_to_wall, y=wall_width/2, z=z_offset),                     # Coin bas-droit
            Point(x=distance_to_wall, y=wall_width/2, z=wall_height + z_offset),       # Coin haut-droit
            Point(x=distance_to_wall, y=-wall_width/2, z=wall_height + z_offset),      # Coin haut-gauche
        ]

        # Créer les marqueurs
        marker_array = MarkerArray()

        for i, corner in enumerate(corners):
            marker = Marker()
            marker.header.frame_id = "base_0"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "wall_corners"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position du coin
            marker.pose.position = corner
            marker.pose.orientation.w = 1.0

            # Taille du cube (5cm x 5cm x 5cm)
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            # Couleur rouge
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.lifetime.sec = 0  # Permanent

            marker_array.markers.append(marker)

        # Publier
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Published {len(corners)} wall corner markers in RViz")

    def _select_multipoint_planner(self) -> bool:
        """Sélectionner le planner 'multi_point' sur curobo_ros"""
        if not self.set_planner_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("set_planner service not available")
            return False

        request = SetPlanner.Request()
        request.planner_type = 4

        future = self.set_planner_client.call_async(request)

        # Attendre avec Rate
        rate = self.create_rate(100)
        timeout = 5.0
        start_time = self.get_clock().now()
        while not future.done():
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().error("set_planner service timeout")
                return False
            rate.sleep()

        if not future.done():
            self.get_logger().error("set_planner service timeout")
            return False

        response = future.result()
        if response.success:
            self.get_logger().info("MultiPoint planner selected")
            return True
        else:
            self.get_logger().error(f"Failed to select planner: {response.message}")
            return False

    def _generate_2d_trajectory(self):
        """Générer trajectoire 2D avec WallPaintingTrajectory"""
        # Récupérer les paramètres
        wall_width = self.get_parameter('wall_width').value
        wall_height = self.get_parameter('wall_height').value
        tool_width = self.get_parameter('tool_width').value
        tool_height = self.get_parameter('tool_height').value
        overlap_ratio = self.get_parameter('overlap_ratio').value
        orientation = self.get_parameter('orientation').value
        turning_radius = self.get_parameter('turning_radius').value

        try:
            # Créer générateur de trajectoire
            traj_gen = WallPaintingTrajectory(
                wall_width=wall_width,
                wall_height=wall_height,
                tool_width=tool_width,
                tool_height=tool_height,
                overlap_ratio=overlap_ratio,
                orientation=orientation,
                turning_radius=turning_radius
            )

            # Générer
            trajectory = traj_gen.generate()
            stats = traj_gen.get_statistics()

            self.get_logger().info(
                f"Generated 2D trajectory: {stats['num_points']} points, "
                f"{trajectory} passes, {stats['total_distance']:.2f}m"
            )

            return trajectory

        except Exception as e:
            self.get_logger().error(f"Failed to generate trajectory: {e}")
            return None

    def _transform_to_3d_poses(self, trajectory_2d: np.ndarray) -> tuple:
        """Transformer coordonnées 2D en poses 3D pour le robot et générer les contraintes"""
        distance_to_wall = self.get_parameter('distance_to_wall').value
        wall_width = self.get_parameter('wall_width').value
        qw = self.get_parameter('target_orientation_qw').value
        qx = self.get_parameter('target_orientation_qx').value
        qy = self.get_parameter('target_orientation_qy').value
        qz = self.get_parameter('target_orientation_qz').value

        poses = []
        constraints = []

        for point in trajectory_2d:
            x_traj, y_traj = point[0], point[1]

            # Transformation: mur dans plan YZ, x fixe
            pose = Pose()
            pose.position.x = distance_to_wall                  # X fixe (distance au mur)
            pose.position.y = x_traj - wall_width / 2           # Y variable (largeur du mur, centré)
            pose.position.z = y_traj + 0.5                            # Z variable (hauteur du mur)

            # Orientation fixe
            pose.orientation.w = qw
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz

            poses.append(pose)

            # Contraintes pour ce waypoint: [theta_x, theta_y, theta_z, x, y, z]
            # 1 = contraint (doit rester égal entre start et goal)
            # 0 = libre (peut bouger)
            # Contraindre X (fixe), laisser Y et Z libres pour peindre le mur
            constraints.extend([
                1,  # theta_x: orientation libre
                1,  # theta_y: orientation libre
                1,  # theta_z: orientation libre
                0,  # x: CONTRAINT (ne bouge pas - plan YZ)
                0,  # y: LIBRE (bouge le long du mur)
                0   # z: LIBRE (bouge le long du mur)
            ])

        self.get_logger().info(f"Transformed to {len(poses)} 3D poses with YZ plane constraints")
        return poses, constraints

    def _plan_trajectory(self, poses: list, constraints: list = None) -> bool:
        """Envoyer les poses à curobo pour planning avec contraintes"""
        timeout = self.get_parameter('service_timeout').value

        if not self.traj_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("generate_trajectory service not available")
            return False

        # Créer requête
        request = TrajectoryGeneration.Request()
        request.target_poses = poses  # Array de Pose pour multipoint!

        # Ajouter les contraintes si disponibles
        if constraints is not None:
            request.trajectories_contraints = constraints
            self.get_logger().info(f"Adding {len(constraints)} constraint values (6 per waypoint)")

        # Appeler service
        self.get_logger().info(f"Planning trajectory with {len(poses)} waypoints...")
        future = self.traj_client.call_async(request)

        # Attendre avec Rate
        rate = self.create_rate(100)
        start_time = self.get_clock().now()
        while not future.done():
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().error("Trajectory planning timeout")
                return False
            rate.sleep()

        if not future.done():
            self.get_logger().error("Trajectory planning timeout")
            return False

        response = future.result()
        if response.success:
            self.get_logger().info(f"Trajectory planned: {response.message}")
            return True
        else:
            self.get_logger().error(f"Planning failed: {response.message}")
            return False

    def _execute_trajectory(self) -> bool:
        """Exécuter la trajectoire via l'action"""
        timeout = self.get_parameter('action_timeout').value

        # Créer l'action client à la demande si nécessaire
        if self.execute_client is None:
            namespace = self.get_parameter('curobo_namespace').value
            self.execute_client = ActionClient(
                self,
                SendTrajectory,
                f'/{namespace}/execute_trajectory',
                callback_group=self.client_cb_group
            )
            self.get_logger().info("Action client created on demand")

        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("execute_trajectory action not available")
            return False

        # Créer goal
        goal = SendTrajectory.Goal()

        self.get_logger().info("Executing trajectory...")
        goal_future = self.execute_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )

        # Attendre l'acceptation du goal avec Rate
        rate = self.create_rate(100)
        timeout_acceptance = 10.0
        start_time = self.get_clock().now()
        while not goal_future.done():
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_acceptance:
                self.get_logger().error("Goal acceptance timeout")
                return False
            rate.sleep()

        if not goal_future.done():
            self.get_logger().error("Goal acceptance timeout")
            return False

        goal_handle = goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Execution goal rejected")
            return False

        # Attendre résultat avec Rate
        result_future = goal_handle.get_result_async()

        rate = self.create_rate(100)
        start_time = self.get_clock().now()
        while not result_future.done():
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                self.get_logger().error("Execution timeout")
                goal_handle.cancel_goal_async()
                return False
            rate.sleep()

        if not result_future.done():
            self.get_logger().error("Execution timeout")
            return False

        result = result_future.result().result
        if result.success:
            self.get_logger().info(f"Execution complete: {result.message}")
            return True
        else:
            self.get_logger().error(f"Execution failed: {result.message}")
            return False

    def _feedback_callback(self, feedback_msg):
        """Afficher progression de l'exécution"""
        progress = feedback_msg.feedback.step_progression * 100
        self.get_logger().info(f"Progress: {progress:.1f}%", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)

    # Utiliser MultiThreadedExecutor pour permettre les appels de services depuis les callbacks
    executor = MultiThreadedExecutor()

    # Créer le node en lui passant l'executor
    node = WallCoverageNode(executor=executor)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
