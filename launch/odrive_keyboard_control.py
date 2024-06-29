#!/usr/bin/env python3
import launch
from launch_ros.actions import Node
from launch import LaunchService

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='my_python', 
            executable='my_talker',
            name='talker',
            output='screen'
        ),
        Node(
            package='my_python',
            executable='my_listener',
            name='listener',
            output='screen'
        )
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
