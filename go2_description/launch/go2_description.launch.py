from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _launch_setup(context, *_args, **_kwargs):
    description_file = Path(LaunchConfiguration("description_file").perform(context))
    rviz_config = Path(LaunchConfiguration("rviz_config").perform(context))
    network_interface = LaunchConfiguration("network_interface").perform(context).strip()

    if not description_file.exists():
        raise FileNotFoundError(description_file)
    if not rviz_config.exists():
        raise FileNotFoundError(rviz_config)

    robot_description = description_file.read_text()

    actions = []

    if network_interface:
        actions.extend(
            [
                SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
                SetEnvironmentVariable(
                    "CYCLONEDDS_URI",
                    "<CycloneDDS><Domain><General><Interfaces>"
                    f'<NetworkInterface name="{network_interface}" priority="default" multicast="default" />'
                    "</Interfaces></General></Domain></CycloneDDS>",
                ),
            ]
        )

    actions.extend([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", str(rviz_config)],
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])

    return actions


def generate_launch_description():
    package_share = get_package_share_directory("go2_description")
    default_description = PathJoinSubstitution([package_share, "urdf", "go2_description.urdf"])
    default_rviz = PathJoinSubstitution([package_share, "launch", "check_joint.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("description_file", default_value=default_description),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("network_interface", default_value="eno1"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
