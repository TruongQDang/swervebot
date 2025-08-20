from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

def generate_launch_description():
        # Package name
        package_name = 'swervebot_description'

        # Process URDF
        xacro_file = PathJoinSubstitution(
		[FindPackageShare(package_name),
		'urdf',
	    	'main.xacro'])
        robot_description_content = Command(['xacro ', xacro_file])
        
        # Robot state publisher
        robot_config = {'use_sim_time': True, 'robot_description': robot_description_content}
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
                arguments=['-d', PathJoinSubstitution([FindPackageShare(package_name), 'rviz', 'rviz.rviz'])],
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