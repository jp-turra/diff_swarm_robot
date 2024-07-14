import os
import xacro

from ament_index_python import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

project_name="robot"
robot_name="swarm_diff_bot"

def generate_launch_description():
    launch_rviz_arg = DeclareLaunchArgument(
        'rviz', 
        default_value='false',
        description='Open RViz.'
    )

    # Get <project_name> shared folder
    project_name_path = get_package_share_path(project_name)
    ros_gz_sim_pkg = get_package_share_path('ros_gz_sim')


    # Get robot URDF path
    project_robot_path = os.path.join(project_name_path, 'urdf', 'robot_model.xacro')
    robot_urdf_path = os.path.join(project_name_path, 'urdf', f'{robot_name}.urdf')


    # Run Xacro on the URDF file
    urdf_file = xacro.process_file(project_robot_path)
    robot_description = urdf_file.toxml()
    with open(robot_urdf_path, 'w') as f:
        f.write(robot_description)

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[robot_urdf_path],
        output=['screen'],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(project_name_path, 'config', 'robot.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Launch Gazebo IGN from CMD
    ign_gz_start_command = "ign gazebo empty.sdf"
    ign_gz_add_urdf_command = "sleep 5 && ign service -s /world/empty/create " + \
        "--reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean " + \
        """--timeout 1000 --req 'sdf_filename: "{}", name:"{}"'""".format(robot_urdf_path, robot_name)
    ign_gz_brigde_config_file = os.path.join(
        project_name_path, 'config', 'gazebo.yaml'
    )
    
    gz_sim = ExecuteProcess(
        cmd=[ign_gz_start_command],
        shell=True
    )

    gz_launch_model = ExecuteProcess(
        cmd=[ign_gz_add_urdf_command],
        shell=True
    )
    
    gz_sim_brigde = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': ign_gz_brigde_config_file,
            'qos_overrides./tf_static.publisher.durability': 'transient_local'
        }],
        output='screen'
    )

    return LaunchDescription([
        launch_rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui,
        rviz,
        gz_sim,
        RegisterEventHandler(
            OnProcessStart(
                target_action=gz_sim,
                on_start=[
                    gz_launch_model,
                ]
            )
        ),
        gz_sim_brigde
    ])

