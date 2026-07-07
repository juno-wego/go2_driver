"""
Launch file for go2_description:
  - Loads GO2 URDF/xacro and starts robot_state_publisher
  - Optionally starts joint_state_publisher_gui for visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('go2_description')

    # Prefer xacro if available, fall back to plain URDF
    xacro_file = os.path.join(pkg_dir, 'xacro', 'robot.xacro')
    urdf_file   = os.path.join(pkg_dir, 'urdf', 'go2_description.urdf')

    use_xacro_arg = DeclareLaunchArgument(
        'use_xacro',
        default_value='true' if os.path.exists(xacro_file) else 'false',
        description='Use xacro to generate URDF (requires xacro package)',
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Start joint_state_publisher_gui for manual joint control',
    )

    use_xacro = LaunchConfiguration('use_xacro')
    use_gui   = LaunchConfiguration('use_gui')

    # Robot description: xacro → URDF string, or plain URDF file
    robot_description_content = Command(
        ['xacro ', xacro_file]
    )

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

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui),
    )

    return LaunchDescription([
        use_xacro_arg,
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
    ])
