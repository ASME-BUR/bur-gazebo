from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0'}]
        ),
        Node(
            package='bur_joystick_control',
            executable='joystick_converter_node',
            name='joystick_converter'
        ),
    ])