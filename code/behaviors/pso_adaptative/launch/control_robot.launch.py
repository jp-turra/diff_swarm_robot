import os

from launch import LaunchDescription
from launch.launch_description import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

pso_adaptative_pkg = get_package_share_directory("pso_adaptative")


def generate_launch_description():
    robot_id_arg = DeclareLaunchArgument("robot_id", default_value="/r1")
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pso_adaptative_pkg, "config", "params.yaml"),
    )

    return LaunchDescription(
        [
            robot_id_arg,
            params_file_arg,
            Node(
                package="pso_adaptative",
                executable="hardware_protection_layer_us",
                name="hardware_protection_layer_us",
                output="screen",
                namespace=LaunchConfiguration("robot_id"),
                ros_arguments=[
                    "--params-file",
                    LaunchConfiguration("params_file"),
                    "--log-level",
                    "info",
                ],
            ),
            Node(
                package="pso_adaptative",
                executable="pso_adaptative_movement",
                name="pso_adaptative",
                output="screen",
                namespace=LaunchConfiguration("robot_id"),
                ros_arguments=["--params-file", LaunchConfiguration("params_file")],
            ),
        ]
    )
