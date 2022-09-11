import os
from struct import pack

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',            
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='ros_remocon',
            executable='remocon_ps5',
            name='remocon_ps5'
        )
    ])