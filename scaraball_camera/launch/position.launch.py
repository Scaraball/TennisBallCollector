import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

	position_node = Node(
		package='scaraball_camera',
		output='screen',
		node_executable='position',
		emulate_tty = True,
	)

	return LaunchDescription([position_node])

