
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Subscribe to /image_raw/h264, decode, and republish on /repub_raw
        # All remappings are shown for clarity
        Node(package='image_transport', 
            executable='republish', 
            output='screen',
             name='republish_node', 
             arguments=[
                'h264',  # Input
                'raw',  # Output
             ], remappings=[
                ('in', 'image_raw'),
                ('in/compressed', 'image_raw/compressed'),
                ('in/theora', 'image_raw/theora'),
                ('in/h264', 'image_raw/h264'),
                ('out', 'repub_raw'),
                ('out/compressed', 'repub_raw/compressed'),
                ('out/theora', 'repub_raw/theora'),
                ('out/theora', 'repub_raw/h264'),
             ]),
    ])
