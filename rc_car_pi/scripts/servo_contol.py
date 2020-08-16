#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Joy 
import Adafruit_PCA9685

# default steering value (90 degree)
steering_default = 375
speed_neutral = 400

def joy_callback(msg):

    current_steering = (msg.axes[0] * 225) + steering_default
    current_speed = ((-(msg.axes[4] - 1) / 2) * 200 + ((msg.axes[5] -1) / 2) * 200) + speed_neutral

    pwm.set_pwm(0, 0, current_speed)
    pwm.set_pwm(1, 0, current_steering)

if __name__ == '__main__':
    rospy.init_node('servo_control')
    rospy.Subscriber('joy', Joy, joy_callback)
    
    pwm = Adafruit_PCA9685.PCA9685(address=0x40)
    pwm.set_pwm_freq(60)

    pwm.set_pwm(0, 0, speed_neutral)
    pwm.set_pwm(1, 0, steering_default)

    rospy.spin()
