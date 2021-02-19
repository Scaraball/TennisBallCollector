import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

	optimisation = Node(
		package='scaraball_control',
		output='screen',
		node_executable='optimisation',
		emulate_tty = True
	)

	return LaunchDescription([optimisation])

