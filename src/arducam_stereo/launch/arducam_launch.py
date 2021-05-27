"""
    Author: IraqLobst3r

    Description:
        Available formats:
            1600x600
            2560x720
            3840x1080
            5184x1944
            6528x1848
            6528x2464

        The standard alignment is 16 in height and width. To add additional alignment use the
        following parameters

"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="arducam_stereo",
            executable="arducam_stereo",
            name="arducam_stereo_node",
            # make shure to print output in console
            output="screen",
            emulate_tty=True,
            parameters=[
                {"width": 1600,
                    "height": 600,
                    "auto_exposure": True,
                    "exposure_value": 0,
                    "auto_white_balance": True,
                    "auto_wb_compensation": True,
                    "red_gain": 100,
                    "blue_gain": 100,
                    "align_width_up": 0,
                    "align_width_down": 0,
                    "align_height_up": 16,
                    "align_height_down": 0,
                    "frame_id" : "arducam_stereo"
                    }
            ]
        )
    ])
