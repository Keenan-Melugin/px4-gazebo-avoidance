"""Nav2 path planning on top of the base stack.

Separate from sim.launch.py on purpose. The base stack brakes for obstacles
and is proven; this adds planning *around* them, with more moving parts and
more ways to fail. Keeping it opt-in means a reader can get the simple thing
working first and can tell which layer broke.

    ros2 launch avoidance_sim nav2.launch.py

What this adds:

    depth cloud --> pointcloud_to_laserscan --> /scan --> Nav2 costmaps
    goal --> planner --> MPPI controller --> /cmd_vel --> software pilot

The last arrow is the design decision, and the first version of this got it
wrong in a way worth recording.

The intent was defence in depth. Nav2's /cmd_vel could be turned into PX4
trajectory setpoints in Offboard mode, which is the obvious route, but PX4
holds collision prevention only in its manual Position-mode flight tasks, so
Offboard discards it. Routing /cmd_vel through the software pilot keeps the
aircraft in Position mode, which looked like a way to have Nav2 plan around
obstacles while PX4 still braked if the plan drove at one.

It does not work, and the reason is structural rather than a tuning problem.
A planner approaches an obstacle deliberately in order to go around it, and
collision prevention exists to veto motion toward obstacles, so the lower
layer vetoes the upper layer plan. Measured with one A/B, same goal and same
everything else:

    CP_DIST 1.0   Nav2 commanded 1.50 m/s forward, achieved 0.00 m/s,
                  deadlocked in front of the wall
    CP_DIST -1    achieved 1.24 m/s and rounded the end of the wall

So PX4 collision prevention and a planner cannot share an axis. Collision
prevention is a manual-flight assist; it does not compose with an autonomous
planner. It stays for the base stack, where it is the whole mechanism.

The resolution is not a parameter. The pilot has two modes, selected on
/avoidance_sim/mode or from the RViz right-click menu:

    brake   Position mode on synthetic sticks. Collision prevention live.
    plan    Offboard, TrajectorySetpoint velocity from Nav2. PX4 holds
            collision prevention only in its manual Position-mode flight
            tasks, so Offboard structurally has none.

CP_DIST therefore never needs changing: it simply does not apply in plan
mode. Verified by leaving it at 2.0, the value that deadlocked the stick
path, and watching plan mode track 1.00 m/s commanded to 1.00 m/s achieved.

Three more things were measured on the way to a goal actually being reached,
and each is a comment in config/nav2.yaml or config/avoidance_bt.xml:

  * The costmap's max_obstacle_height is compared in the odom frame, so the
    default of 2.0 discarded every observation from a flying aircraft and
    the planner worked off marks made during moments below 2 m.
  * Pure pursuit aborts the whole goal on "detected collision ahead!", which
    fires when the camera marks a wall cell under a path planned a moment
    earlier. The bundled behaviour tree recovers (clear local costmap, wait,
    replan) instead of failing.
  * The camera sees 73 degrees, so planning around a 10 m wall needs about
    10 m of standoff to have observed both ends, and the global costmap must
    not clear, or turning away forgets the wall.

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
    # The tree lives in this package, so only the launch file knows its
    # installed path. This override beats the value in nav2.yaml.
    bt_xml = os.path.join(share, 'config', 'avoidance_bt.xml')
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
        DeclareLaunchArgument(
            'world', default_value='walls',
            description='Passed through: the world PX4 was started with.'),
        DeclareLaunchArgument(
            'lidar', default_value='false',
            description='Passed through: the aircraft carries the extra lidar.'),
        DeclareLaunchArgument(
            'sensors', default_value='',
            description='Passed through: a parameter file describing the '
                        'obstacle node\'s sensors.'),
        DeclareLaunchArgument(
            'bridge_extra', default_value='',
            description='Passed through: extra Gazebo-to-ROS bridge specs.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'sim.launch.py')),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz'),
                'agent': LaunchConfiguration('agent'),
                'agent_cmd': LaunchConfiguration('agent_cmd'),
                'world': LaunchConfiguration('world'),
                'lidar': LaunchConfiguration('lidar'),
                'sensors': LaunchConfiguration('sensors'),
                'bridge_extra': LaunchConfiguration('bridge_extra'),
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
        # The navigator's goal_pose subscription is moved off RViz's
        # /goal_pose. Both used to hear the 2D Goal Pose tool, so in brake
        # mode one click sent PX4 a reposition AND started a Nav2 navigation
        # (measured 2026-10-10). Now rviz_goal_bridge owns /goal_pose and
        # forwards it here only in plan mode. The Nav2 Goal tool is not
        # affected: it sends the navigate_to_pose action directly.
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator',
             parameters=[params, {'default_nav_to_pose_bt_xml': bt_xml}],
             remappings=[('goal_pose', '/avoidance_sim/nav2_goal_pose')],
             output='screen'),
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
