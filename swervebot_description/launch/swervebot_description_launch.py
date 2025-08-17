import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
        # Package name
        package_name = 'swervebot_description'

        # Process URDF
        xacro_file = os.path.join(
                get_package_share_directory(package_name),
                'urdf', 
                'main.xacro'
        )
        robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
        )
        
        # Robot state publisher
        robot_config = {'use_sim_time': True, 'robot_description': robot_description}
        robot_state_publisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[robot_config],
        ) 
        
        # Rviz
        rviz = Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(get_package_share_directory(package_name), 'rviz', 'rviz.rviz')],
                output='screen'
        )

        # Joint state publisher gui
        joint_state_pub_gui = Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui'
        )

        return LaunchDescription([
                robot_state_publisher,
                rviz,
                joint_state_pub_gui
        ])