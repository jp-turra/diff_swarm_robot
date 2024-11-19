import os
import yaml

from launch import LaunchDescription
from launch.launch_description import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

pso_adaptative_pkg = get_package_share_directory("pso_adaptative")
default_params = os.path.join(pso_adaptative_pkg, "config", "params.yaml")


def generate_launch_description():
    robot_id_arg = DeclareLaunchArgument("robot_id", default_value="/r1")
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
    )

    parameters = {}
    with open(default_params, "r") as f:
        parameters = yaml.safe_load(f)

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
                parameters=[
                    parameters["/*"]["hardware_protection_layer"]["ros__parameters"]
                ],
            ),
            Node(
                package="pso_adaptative",
                executable="pso_adaptative_movement",
                name="pso_adaptative",
                output="screen",
                namespace=LaunchConfiguration("robot_id"),
                parameters=[
                    parameters["/*"]["pso_adaptative_movement"]["ros__parameters"]
                ],
            ),
        ]
    )
