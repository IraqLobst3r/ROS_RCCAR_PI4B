# ROS_RCCAR_PI4B

# Setup ENV
On Host
roscore
export ROS_MASTER_URI=http://[your-desktop-machine-ip]:11311

On Raspberry pi
export ROS_MASTER_URI=http://[your-desktop-machine-ip]:11311
export ROS_IP=[your-desktop-machine-ip]

start rpliidar
roslaunch rplidar_ros rplidar.launch

start picam
roslaunch raspicam_node camerav2_1280x960.launch

start joystick
rosrun joy joy_node
