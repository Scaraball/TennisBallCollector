from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro

def generate_launch_description():

	pkg_description = FindPackageShare("scaraball_description").find("scaraball_description")
	model_file = os.path.join(pkg_description, "urdf", "scaraball.urdf.xacro")
	
	pkg_gazebo = FindPackageShare("scaraball_gazebo").find("scaraball_gazebo")
	gazebo_path = os.path.join(FindPackageShare("gazebo_ros").find("gazebo_ros"), "launch")
	
	with open("/tmp/scaraball.urdf", "w") as stream: 
		stream.write(xacro.process_file(model_file).toxml())

	robot_state_publisher_node = Node(
		package='robot_state_publisher',
		node_executable='robot_state_publisher',
		arguments=["/tmp/scaraball.urdf"],
		output='screen',
	)

	# GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
	spawn_entity = Node(
						package = 'gazebo_ros', 
						node_executable = 'spawn_entity.py',
						arguments = ['-entity', 'scaraball', '-topic', "robot_description",'-x','0','-y','5'],
						# arguments = ['-entity', 'scaraball', '-file', "/tmp/scaraball.urdf", "-x", "5", "-y", "5"],
						output = 'screen',
					)

	return LaunchDescription([spawn_entity, robot_state_publisher_node])
