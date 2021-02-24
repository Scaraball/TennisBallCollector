import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

	pkg_share = FindPackageShare("scaraball_description").find("scaraball_description")
	model_file = os.path.join(pkg_share, "urdf", "scaraball.urdf.xacro")
	rviz_config_file = os.path.join(pkg_share, "config", "display.rviz")
	
	joint_state_publisher_node = Node(
		package='joint_state_publisher',
		node_executable='joint_state_publisher',
		name="joint_state_publisher",
	)

	joint_state_publisher_gui_node = Node(
		package='joint_state_publisher_gui',
		node_executable='joint_state_publisher_gui',
		name="joint_state_publisher_gui",
	)
	
	with open("/tmp/scaraball.urdf", "w") as stream: stream.write(xacro.process_file(model_file).toxml())
	robot_state_publisher_node = Node(
		package='robot_state_publisher',
		node_executable='robot_state_publisher',
		arguments=["tmp/scaraball.urdf"],
		output='screen',
	)

	rviz_node = Node(
		package='rviz2',
		node_executable='rviz2',
		name="rviz2",
		arguments=["-d", rviz_config_file],
		output="screen",
	)

	return LaunchDescription([joint_state_publisher_node, joint_state_publisher_gui_node, robot_state_publisher_node, rviz_node])

