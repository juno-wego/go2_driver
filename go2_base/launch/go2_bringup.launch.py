from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *_args, **_kwargs):
    network_interface = (
        LaunchConfiguration("network_interface")
        .perform(context)
        .strip()
    )

    params_file = Path(
        LaunchConfiguration("params_file").perform(context)
    )

    if not params_file.exists():
        raise FileNotFoundError(params_file)

    actions = []

    # Cyclone DDS 네트워크 인터페이스 설정
    if network_interface:
        actions.extend(
            [
                SetEnvironmentVariable(
                    "RMW_IMPLEMENTATION",
                    "rmw_cyclonedds_cpp",
                ),
                SetEnvironmentVariable(
                    "CYCLONEDDS_URI",
                    "<CycloneDDS>"
                    "<Domain>"
                    "<General>"
                    "<Interfaces>"
                    f'<NetworkInterface name="{network_interface}" '
                    'priority="default" multicast="default" />'
                    "</Interfaces>"
                    "</General>"
                    "</Domain>"
                    "</CycloneDDS>",
                ),
            ]
        )

    # Go2 URDF / robot_state_publisher / RViz
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("go2_description"),
                        "launch",
                        "go2_description.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "description_file": LaunchConfiguration(
                    "description_file"
                ),
                "rviz_config": LaunchConfiguration("rviz_config"),
                "rviz": LaunchConfiguration("rviz"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
            condition=IfCondition(
                LaunchConfiguration("enable_description")
            ),
        )
    )

    # cmd_vel → Unitree SDK 제어 명령
    actions.append(
        Node(
            package="go2_base",
            executable="go2_cmd_vel_bridge",
            name="go2_cmd_vel_bridge",
            output="screen",
            parameters=[
                str(params_file),
                {
                    "use_sim_time": LaunchConfiguration(
                        "use_sim_time"
                    )
                },
            ],
            condition=IfCondition(
                LaunchConfiguration("enable_control")
            ),
        )
    )

    # Unitree SDK 상태 → ROS 2 토픽
    actions.append(
        Node(
            package="go2_base",
            executable="go2_state_bridge",
            name="go2_state_bridge",
            output="screen",
            parameters=[
                str(params_file),
                {
                    "use_sim_time": LaunchConfiguration(
                        "use_sim_time"
                    ),
                    "rebase_odom_on_start": LaunchConfiguration(
                        "rebase_odom_on_start"
                    ),
                },
            ],
            condition=IfCondition(
                LaunchConfiguration("enable_bridge")
            ),
        )
    )

    # Receive XT16 UDP packets directly and publish the point cloud.
    actions.append(
        Node(
            package="hesai_lidar",
            executable="hesai_lidar_node",
            name="hesai_lidar_node",
            output="screen",
            parameters=[
                {
                    "config_path": LaunchConfiguration(
                        "hesai_config_file"
                    )
                }
            ],
            condition=IfCondition(
                LaunchConfiguration("enable_hesai")
            ),
        )
    )

    return actions


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [
            FindPackageShare("go2_base"),
            "config",
            "go2_driver_params.yaml",
        ]
    )

    default_description = PathJoinSubstitution(
        [
            FindPackageShare("go2_description"),
            "urdf",
            "go2_description.urdf",
        ]
    )

    default_rviz = PathJoinSubstitution(
        [
            FindPackageShare("go2_description"),
            "rviz",
            "go2.rviz",
        ]
    )

    default_hesai_config = PathJoinSubstitution(
        [
            FindPackageShare("hesai_lidar"),
            "config",
            "config.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
            ),
            DeclareLaunchArgument(
                "description_file",
                default_value=default_description,
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
            ),
            DeclareLaunchArgument(
                "network_interface",
                default_value="eno1",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_control",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_bridge",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_description",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "rebase_odom_on_start",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "enable_hesai",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "hesai_config_file",
                default_value=default_hesai_config,
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
