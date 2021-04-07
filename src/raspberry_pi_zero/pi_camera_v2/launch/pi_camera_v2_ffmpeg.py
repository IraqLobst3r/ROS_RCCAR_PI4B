from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pi_camera_v2",
            executable="pi_camera_v2_ffmpeg",
            name="pi_cam_ffmpeg",
            # make shure to print output in console
            output="screen",
            emulate_tty=True,
            parameters=[
                {"widthxheight": "1920x1080",
                    "fps": "30",
                    }
            ]
        )
    ])
