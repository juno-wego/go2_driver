"""
go2_bringup.launch.py
======================
Main launch file for Unitree GO2 robot.

Usage:
  ros2 launch go2_base go2_bringup.launch.py
  ros2 launch go2_base go2_bringup.launch.py network_interface:=eth0
  ros2 launch go2_base go2_bringup.launch.py use_rviz:=true

What it starts:
  1. robot_state_publisher  — publishes /robot_description + /tf (URDF)
  2. go2_driver_node        — DDS ↔ ROS2 bridge + sport control
  3. (optional) rviz2       — visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    # ── Package directories ──────────────────────────────────────────
    go2_base_dir  = get_package_share_directory('go2_base')
    go2_desc_dir  = get_package_share_directory('go2_description')

    # ── Launch arguments ─────────────────────────────────────────────
    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='',
        description='Network interface for DDS communication (e.g. eth0, wlan0). '
                    'Leave empty for default.',
    )

    use_sdk_arg = DeclareLaunchArgument(
        'use_sdk',
        default_value='true',
        description='Use unitree_sdk2_python DDS mode. '
                    'Set false to use ROS2-only mode (requires unitree_ros2 bridge).',
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish odom → base_link TF transform from SportModeState.',
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization.',
    )

    use_xacro_arg = DeclareLaunchArgument(
        'use_xacro',
        default_value='true',
        description='Use xacro to process robot description. Set false to use plain URDF.',
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID.',
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame ID.',
    )

    # ── Substitutions ────────────────────────────────────────────────
    network_interface = LaunchConfiguration('network_interface')
    use_sdk           = LaunchConfiguration('use_sdk')
    publish_tf        = LaunchConfiguration('publish_tf')
    use_rviz          = LaunchConfiguration('use_rviz')
    odom_frame        = LaunchConfiguration('odom_frame')
    base_frame        = LaunchConfiguration('base_frame')

    # ── Robot description (xacro) ────────────────────────────────────
    xacro_file = os.path.join(go2_desc_dir, 'xacro', 'robot.xacro')
    urdf_file  = os.path.join(go2_desc_dir, 'urdf', 'go2_description.urdf')

    # Use xacro if available, else plain URDF
    if os.path.exists(xacro_file):
        robot_description_content = Command(['xacro ', xacro_file])
    else:
        with open(urdf_file, 'r') as f:
            robot_description_content = f.read()

    # ── Nodes ────────────────────────────────────────────────────────

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False,
        }],
    )

    go2_driver_node = Node(
        package='go2_base',
        executable='go2_driver_node.py',
        name='go2_driver_node',
        output='screen',
        parameters=[{
            'network_interface': network_interface,
            'use_sdk_mode':      use_sdk,
            'publish_tf':        publish_tf,
            'odom_frame':        odom_frame,
            'base_frame':        base_frame,
            'cmd_vel_timeout':   0.5,
        }],
        remappings=[
            # Standard ROS2 cmd_vel (no namespace prefix)
            ('cmd_vel', '/cmd_vel'),
        ],
    )

    rviz_config = os.path.join(go2_desc_dir, 'launch', 'check_joint.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        # Arguments
        network_interface_arg,
        use_sdk_arg,
        publish_tf_arg,
        use_rviz_arg,
        use_xacro_arg,
        odom_frame_arg,
        base_frame_arg,
        # Nodes
        LogInfo(msg='Starting GO2 bringup...'),
        robot_state_publisher_node,
        go2_driver_node,
        rviz_node,
    ])
