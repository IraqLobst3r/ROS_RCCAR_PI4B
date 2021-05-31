"""
    Author: IraqLobst3r
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="arducam_stereo",
                executable="arducam_singlemode",
                name="arducam_singlemode_node",
                # make shure to print output in console
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "width": 1920,
                        "height": 1080,
                        "cam_interface": 1,
                        "auto_exposure": True,
                        "exposure_value": 0,
                        "auto_white_balance": True,
                        "auto_wb_compensation": True,
                        "red_gain": 100,
                        "blue_gain": 100,
                        "frame_id": "arducam_cam0",
                    }
                ],
            )
        ]
    )
