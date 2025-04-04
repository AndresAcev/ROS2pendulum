import os
import xacro

from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import Shutdown, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    launch_description = LaunchDescription()

    robot_name = 'scara'
    pkg_name = 'scara'
    pkg_share = get_package_share_directory(pkg_name)
    pkg_share_gz = get_package_share_directory(pkg_name+"_gz")

    ## ENV Variables
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_share, 'worlds'), ':' +
            str(Path(pkg_share).parent.resolve())
            ]
        )
    launch_description.add_action(gazebo_resource_path)

    ## ARGUMENTS
    rviz = LaunchConfiguration('rviz')
    rviz_argument = DeclareLaunchArgument(
        name='rviz', 
        default_value='False',
        description='Flag to enable rviz'
        )
    launch_description.add_action(rviz_argument)

    ## NODES
    #  Robot state publisher
    #model_path = os.path.join(pkg_share, f'model/{robot_name}.xacro')
    model_path = os.path.join(pkg_share_gz,'urdf','gz.xacro')
    robot_description = xacro.process_file(model_path).toxml()
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="description",
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True}],
    )
    launch_description.add_action(robot_state_publisher_node)

    # Gazebo
    ros_gz_path=PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
            )
        )
    ros_gz_node=IncludeLaunchDescription(
        ros_gz_path,
        launch_arguments={
            'gz_args': ['-r  -v4 empty.sdf'], # --render-engine ogre
            'on_exit_shutdown': 'true'
            }.items()
        )
    launch_description.add_action(ros_gz_node)

    # Spawn Robot
    spawn_model_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-z','0.0'
        ],
        output='screen',
    )
    launch_description.add_action(spawn_model_node)

    # ROS2 - Gazebo Bridge
    # this is very important so we can control the robot from ROS2
    bridge_params = os.path.join(pkg_share_gz,'config','bridge_parameters.yaml')
    ros_gz_bridge_node = Node (
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )
    launch_description.add_action(ros_gz_bridge_node)

    # Rviz2
    rviz_path = os.path.join(pkg_share, f'rviz/view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
        on_exit=Shutdown(),
        condition=IfCondition(rviz)
    )
    launch_description.add_action(rviz_node)
    
    return launch_description