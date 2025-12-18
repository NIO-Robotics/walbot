from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='walbot_coverage_planning',
            executable='wall_coverage_node',
            name='wall_coverage_node',
            output='screen',
            parameters=[{
                # Wall configuration
                'wall_width': 0.5,
                'wall_height': 0.5,
                'tool_width': 0.3,
                'tool_height': 0.60,
                'overlap_ratio': 0.1,
                'orientation': 'horizontal',
                'turning_radius': 0.00,

                # Robot configuration
                'distance_to_wall': 0.5,
                'target_orientation_qw': 0.707,
                'target_orientation_qx': 0.0,
                'target_orientation_qy': 0.707,
                'target_orientation_qz': 0.0,

                # Curobo configuration
                'curobo_namespace': 'unified_planner',
                'service_timeout': 300.0,  # 5 minutes pour planification multipoint
                'action_timeout': 300.0,
                'connect_waypoints': True,
                'time_dilation_factor': 1.0,

                # Home position (joint space)
                'home_joint_positions': [
                    -0.07784457504749298,
                    -0.18099723756313324,
                    1.6212745904922485,
                    -2.5986220836639404,
                    -0.1518784463405609,
                    -0.537718653678894
                ],
            }]
        ),
    ])
