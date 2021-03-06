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
                {"size": "1440x1080",
                    "fps": "30",
                    "frame_id" : "pi_cam",
                    "rotation": 2
                    }
            ]
        )
    ])
