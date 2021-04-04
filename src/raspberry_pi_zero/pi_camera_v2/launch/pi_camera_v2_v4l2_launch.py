from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pi_camera_v2",
            executable="pi_camera_v2_v4l2",
            name="pi_cam_v4l2",
            # make shure to print output in console
            output="screen",
            emulate_tty=True,
            parameters=[
                {"width": 1920,
                    "height": 1080,
                    "fmt_index": 4,
                    "fmt_grey": False,
                    "mmap_req_buffer_num": 5
                    }
            ]
        )
    ])