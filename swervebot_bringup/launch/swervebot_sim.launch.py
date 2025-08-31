from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
		' use_mock_hardware:=false', 
		' sim_gazebo:=true',])

	robot_description = {'robot_description': robot_description_content}

	robot_state_publisher = Node(
				package='robot_state_publisher',
				executable='robot_state_publisher',
				name='robot_state_publisher',
				output='screen',
				parameters=[robot_description],) 

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
	
	# Gazebo
	world = PathJoinSubstitution(
		[FindPackageShare(description_package),
		'world',
		'empty_world.sdf'])    
	gazebo = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
			launch_arguments={
				'gz_args': ['-r ', world],
				'on_exit_shutdown': 'True'}.items())

	# Spawn robot
	spawn = Node(
		package='ros_gz_sim', 
		executable='create',
		parameters=[{
			'name': 'swervebot',
			'topic': 'robot_description',
			'z': 0.5,}],			
		output='screen')
	
	# ROS-Gz bridge
	bridge_params = PathJoinSubstitution(
            [FindPackageShare(bringup_package),
            'config',
            'gz_bridge.yaml'])
	bridge = Node(
	    package='ros_gz_bridge',
        executable='parameter_bridge',
		parameters=[{'config_file': bridge_params}],
		output='screen')

	# Joint state broadcaster
	joint_state_broadcaster_spawner = Node(
		package="controller_manager",
		executable="spawner",
		arguments=[
			"joint_state_broadcaster",
			"--controller-manager",
			"/controller_manager",],
	)

	# Forward position controller
	position_controller = Node(
		package="controller_manager",
		executable="spawner",
		arguments=[
			"forward_position_controller", 
			"--controller-manager", 
			"/controller_manager"],
	)

	# Forward velocity controller
	velocity_controller = Node(
		package="controller_manager",
		executable="spawner",
		arguments=[
			"forward_velocity_controller", 
			"--controller-manager", 
			"/controller_manager"],
	)

	return LaunchDescription([ 
		robot_state_publisher,
		rviz,
		gazebo, 
		spawn,
		bridge,
		joint_state_broadcaster_spawner,
		position_controller,
		velocity_controller,
		])