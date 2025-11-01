from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bur_gz_thruster_bridge',
            executable='thruster_bridge_node',
            name='thruster_bridge',
            output='screen'
        ),
    ])