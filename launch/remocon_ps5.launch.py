import os
import math

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
            name='remocon_ps5',
            parameters=[
                {"max_trans": 0.5},
                {"max_rot": math.pi},
            ],
        )
    ])