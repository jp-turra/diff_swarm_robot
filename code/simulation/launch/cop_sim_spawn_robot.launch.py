import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

from launch_ros.actions import Node

from ament_index_python import get_package_share_path

pakcage_name="simulation"

def generate_launch_description():
    
    project_name_path = get_package_share_path(pakcage_name)

    robot_description_path = os.path.join(
        project_name_path, 
        'models', 
        'Robot_TCC2.ttm'
    )

    robot_desc_param = DeclareLaunchArgument(
        'robot_description_path',
        default_value=robot_description_path,
        description='Path to the robot description file.'
    )

    spawn_robot_node = Node(
        package=pakcage_name,
        executable='cop_sim_spawn_robot',
        name='cop_sim_spawn_robot',
        parameters=[
            {
                'robot_description_path': robot_description_path
            }
        ]
    )

    return LaunchDescription([
        robot_desc_param,
        spawn_robot_node
    ])