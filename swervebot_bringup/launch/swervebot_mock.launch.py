from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
	description_package = 'swervebot_description'
	description_file = 'main.xacro'
	bringup_package = 'swervebot_bringup'
	controllers_config = 'swervebot_controllers.yaml'

	# Robot state publisher
	xacro_file = PathJoinSubstitution(
		[FindPackageShare(description_package),
		'urdf',
	    	description_file])

	robot_description_content = Command([
		'xacro ', xacro_file,
		' use_mock_hardware:=true'])

	robot_description = {'robot_description': robot_description_content}

	robot_state_publisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[robot_description],
        ) 

	# Rviz
	rviz_config_file = PathJoinSubstitution(
		[FindPackageShare(description_package), 
   		'rviz', 
		'rviz.rviz'])
	rviz = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz2",
		output="log",
		arguments=["-d", rviz_config_file],
   	)

	# Controller manager
	robot_controllers = PathJoinSubstitution(
		[FindPackageShare(bringup_package), 
   		'config', 
		controllers_config])
	controller_manager = Node(
		package="controller_manager",
		executable="ros2_control_node",
		output="both",
		parameters=[robot_description, robot_controllers],
    	)

	# Joint state broadcaster
	joint_state_broadcaster_spawner = Node(
		package="controller_manager",
		executable="spawner",
		arguments=[
			"joint_state_broadcaster",
			"--controller-manager",
			"/controller_manager",],
	)

	# Caster swerve drive controller
	caster_swerve_drive_controller = Node(
		package="controller_manager",
		executable="spawner",
		arguments=[
			"caster_swerve_drive_controller", 
			"--controller-manager", 
			"/controller_manager"],
	)

	return LaunchDescription([
		robot_state_publisher,
		controller_manager,
		rviz,
		joint_state_broadcaster_spawner,
		caster_swerve_drive_controller,
		])