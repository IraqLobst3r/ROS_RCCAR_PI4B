from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pi_camera_v2",
            executable="pi_camera_v2_cv",
            name="pi_cam_cv",
            # make shure to print output in console
            output="screen",
            emulate_tty=True,
            parameters=[
                {"width": 1920,
                    "height": 1080}
            ]
        )
    ])
