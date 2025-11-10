from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bur_joystick_control',
            executable='network_joystick_node',
            name='network_joystick'
        ),
        Node(
            package='bur_joystick_control', 
            executable='joystick_converter_node',
            name='joystick_converter'
        ),
    ])