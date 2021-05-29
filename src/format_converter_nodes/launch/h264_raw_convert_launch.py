from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="format_converter_nodes",
            executable="h264_to_raw",
            name="pi_cam_h264_raw",
            # make shure to print output in console
            output="screen",
            emulate_tty=True,
            parameters=[
                {"pub_topic": "/pi_cam/raw",
                    "sub_topic": "/pi_cam/h264_image",
                    }
            ]
        ),
        Node(
            package="format_converter_nodes",
            executable="h264_to_raw",
            name="arducam_raw_left",
            # make shure to print output in console
            output="screen",
            emulate_tty=True,
            parameters=[
                {"pub_topic": "/arducam_stereo/raw_left",
                    "sub_topic": "/arducam_stereo/h264_image/left",
                    }
            ]
        ),
        Node(
            package="format_converter_nodes",
            executable="h264_to_raw",
            name="arducam_raw_right",
            # make shure to print output in console
            output="screen",
            emulate_tty=True,
            parameters=[
                {"pub_topic": "/arducam_stereo/raw_right",
                    "sub_topic": "/arducam_stereo/h264_image/right",
                    }
            ]
        )
    ])
