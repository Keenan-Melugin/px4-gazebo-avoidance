"""Nav2 path planning on top of the base stack.

Separate from sim.launch.py on purpose. The base stack brakes for obstacles
and is proven; this adds planning *around* them, with more moving parts and
more ways to fail. Keeping it opt-in means a reader can get the simple thing
working first and can tell which layer broke.

    ros2 launch avoidance_sim nav2.launch.py

What this adds:

    depth cloud --> pointcloud_to_laserscan --> /scan --> Nav2 costmaps
    goal --> planner --> MPPI controller --> /cmd_vel --> software pilot

The last arrow is the design decision. Nav2's /cmd_vel could be turned into
PX4 trajectory setpoints in Offboard mode, which is the obvious route and the
one PX4's own docs lead you to. It is also the wrong one here: PX4 holds
collision prevention only in its manual Position-mode flight tasks, so
Offboard silently discards it and makes Nav2 solely responsible for not
hitting things.

Routing /cmd_vel through the software pilot instead keeps the aircraft in
Position mode, so PX4's collision prevention stays underneath Nav2 as an
independent backstop. Nav2 plans around obstacles; PX4 still brakes if the
plan drives at one.

Nav2 is two-dimensional. It plans in the horizontal plane and knows nothing
about altitude, which stays with the pilot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Nav2 nodes that have to be driven through their lifecycle before they do
# anything. Order matters: costmaps before the things that read them.
LIFECYCLE_NODES = ['controller_server', 'planner_server',
                   'behavior_server', 'bt_navigator', 'velocity_smoother']


def generate_launch_description():
    share = get_package_share_directory('avoidance_sim')
    params = os.path.join(share, 'config', 'nav2.yaml')
    sim_time = [{'use_sim_time': True}]

    return LaunchDescription([
        DeclareLaunchArgument(
            'base', default_value='true',
            description='Also start the base stack. false if it is already '
                        'running.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Passed through to the base stack.'),
        DeclareLaunchArgument(
            'agent', default_value='true',
            description='Passed through. false if an agent is already running, '
                        'otherwise the second one collides on UDP 8888.'),
        DeclareLaunchArgument(
            'agent_cmd', default_value='MicroXRCEAgent',
            description='Passed through to the base stack.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'sim.launch.py')),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz'),
                'agent': LaunchConfiguration('agent'),
                'agent_cmd': LaunchConfiguration('agent_cmd'),
            }.items()),

        # The depth cloud flattened into a 2D scan, because Nav2 costmaps are
        # 2D. min/max height is a slab around the camera: a taller slab makes
        # the aircraft afraid of the ground, a thinner one misses thin walls.
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='cloud_to_scan',
            remappings=[('cloud_in', '/depth_camera/points'),
                        ('scan', '/scan')],
            parameters=sim_time + [{
                'target_frame': 'base_link',
                'transform_tolerance': 0.05,
                'min_height': -1.0,
                'max_height': 1.0,
                # The camera's real arc is 73 degrees. Claiming more would
                # mark space as clear that was never observed.
                'angle_min': -0.637,
                'angle_max': 0.637,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.25,
                'range_max': 18.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            output='screen'),

        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', parameters=[params],
             remappings=[('cmd_vel', '/cmd_vel_nav')], output='screen'),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', parameters=[params], output='screen'),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', parameters=[params], output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', parameters=[params], output='screen'),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', parameters=[params],
             remappings=[('cmd_vel', '/cmd_vel_nav'),
                         ('cmd_vel_smoothed', '/cmd_vel')], output='screen'),

        Node(package='nav2_lifecycle_manager',
             executable='lifecycle_manager', name='lifecycle_manager_nav',
             parameters=sim_time + [{'autostart': True,
                                     'node_names': LIFECYCLE_NODES}],
             output='screen'),

        # No extra node converts /cmd_vel to sticks. The software pilot in the
        # base stack subscribes to /cmd_vel itself, because two nodes
        # streaming ManualControlSetpoint would interleave on the same PX4
        # input. One publisher, two sources.
    ])
