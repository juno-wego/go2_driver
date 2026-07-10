import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _activate_local_debs():
    for parent in Path(__file__).resolve().parents:
        local_ros = parent / ".local_ros" / "opt" / "ros" / os.environ.get("ROS_DISTRO", "humble")
        if local_ros.exists():
            local_ubuntu = parent / ".local_ubuntu"
            _prepend_env("AMENT_PREFIX_PATH", str(local_ros))
            _prepend_env("CMAKE_PREFIX_PATH", str(local_ros))
            _prepend_env("LD_LIBRARY_PATH", str(local_ros / "lib"))
            _prepend_env("LD_LIBRARY_PATH", str(local_ros / "lib" / "aarch64-linux-gnu"))
            _prepend_env("PYTHONPATH", str(local_ros / "local" / "lib" / "python3.10" / "dist-packages"))
            _prepend_env("PYTHONPATH", str(local_ros / "lib" / "python3.10" / "site-packages"))
            sys.path.insert(0, str(local_ros / "local" / "lib" / "python3.10" / "dist-packages"))
            sys.path.insert(0, str(local_ros / "lib" / "python3.10" / "site-packages"))
            if local_ubuntu.exists():
                _prepend_env("LD_LIBRARY_PATH", str(local_ubuntu / "usr" / "lib"))
                _prepend_env("LD_LIBRARY_PATH", str(local_ubuntu / "usr" / "lib" / "aarch64-linux-gnu"))
            break


def _prepend_env(name, value):
    if not value:
        return
    current = [item for item in os.environ.get(name, "").split(":") if item]
    if value in current:
        current.remove(value)
    os.environ[name] = ":".join([value] + current)


def _launch_setup(context, *_args, **_kwargs):
    network_interface = LaunchConfiguration("network_interface").perform(context).strip()
    params_file = Path(LaunchConfiguration("params_file").perform(context))

    if not params_file.exists():
        raise FileNotFoundError(params_file)

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

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("go2_description"), "launch", "go2_description.launch.py"]
                )
            ),
            launch_arguments={
                "description_file": LaunchConfiguration("description_file"),
                "rviz_config": LaunchConfiguration("rviz_config"),
                "start_rviz": LaunchConfiguration("start_rviz"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("enable_description")),
        )
    )

    actions.append(
        Node(
            package="go2_base",
            executable="go2_cmd_vel_bridge",
            name="go2_cmd_vel_bridge",
            output="screen",
            parameters=[str(params_file), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("enable_control")),
        )
    )

    actions.append(
        Node(
            package="go2_base",
            executable="go2_state_bridge",
            name="go2_state_bridge",
            output="screen",
            parameters=[str(params_file), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("enable_bridge")),
        )
    )

    actions.append(
        Node(
            package="go2_base",
            executable="restamp_pointcloud2.py",
            name="hesai_pointcloud_restamper",
            output="screen",
            remappings=[
                ("cloud_in", "/rslidar_points"),
                ("cloud_out", "/rslidar_points_restamped"),
            ],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("enable_sensor_restamp")),
        )
    )

    actions.append(
        Node(
            package="go2_base",
            executable="restamp_pointcloud2.py",
            name="utlidar_pointcloud_restamper",
            output="screen",
            remappings=[
                ("cloud_in", "/utlidar/cloud"),
                ("cloud_out", "/utlidar/cloud_restamped"),
            ],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("enable_sensor_restamp")),
        )
    )

    return actions


def generate_launch_description():
    _activate_local_debs()

    default_params = PathJoinSubstitution(
        [FindPackageShare("go2_base"), "config", "go2_driver_params.yaml"]
    )
    default_description = PathJoinSubstitution(
        [FindPackageShare("go2_description"), "urdf", "go2_description.urdf"]
    )
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("go2_description"), "launch", "check_joint.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("description_file", default_value=default_description),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("network_interface", default_value="eno1"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("enable_control", default_value="true"),
            DeclareLaunchArgument("enable_bridge", default_value="true"),
            DeclareLaunchArgument("enable_description", default_value="true"),
            DeclareLaunchArgument("enable_sensor_restamp", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
