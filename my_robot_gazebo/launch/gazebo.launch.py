from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro

def generate_launch_description():

	pkg_description = FindPackageShare("my_robot_description").find("my_robot_description")
	model_file = os.path.join(pkg_description, "urdf", "my_robot.urdf.xacro")
	
	pkg_gazebo = FindPackageShare("my_robot_gazebo").find("my_robot_gazebo")
	gazebo_path = os.path.join(FindPackageShare("gazebo_ros").find("gazebo_ros"), "launch")
	
	with open("/tmp/my_robot.urdf", "w") as stream: 
		stream.write(xacro.process_file(model_file).toxml())

	robot_state_publisher_node = Node(
		package='robot_state_publisher',
		node_executable='robot_state_publisher',
		arguments=["/tmp/my_robot.urdf"],
		output='screen',
	)

	gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([gazebo_path, '/gazebo.launch.py']))
	
	# GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
	spawn_entity = Node(
						package = 'gazebo_ros', 
						node_executable = 'spawn_entity.py',
						arguments = ['-entity', 'my_robot', '-topic', "/robot_description"],
						# arguments = ['-entity', 'my_robot', '-file', "/tmp/my_robot.urdf"],
						output = 'screen',
					)

	return LaunchDescription([gazebo, spawn_entity, robot_state_publisher_node])
