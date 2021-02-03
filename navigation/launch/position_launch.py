import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

	control_node = Node(
		package='navigation',
		output='screen',
		node_executable='position',
		emulate_tty = True,
	)

	return LaunchDescription([control_node])

