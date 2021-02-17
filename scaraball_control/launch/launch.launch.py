import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

	control_node = Node(
		package='scaraball_control',
		output='screen',
		node_executable='go_to_point',
		emulate_tty = True,
	)

	control_pinces_node = Node(
		package='scaraball_control',
		output='screen',
		node_executable='control_pinces',
		emulate_tty = True
	)

	return LaunchDescription([control_node, control_pinces_node])

