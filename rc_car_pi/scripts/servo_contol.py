#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Joy 
import Adafruit_PCA9685

# default steering value (90 degree)
steering_default = 375
speed_neutral = 400

def joy_callback(msg):

    current_steering = -(msg.axes[0] * 225) + steering_default
    current_speed = ((-(msg.axes[5] - 1) / 2) * 200 + ((msg.axes[4] -1) / 2) * 200) + speed_neutral
    print("set joy")
    pwm.set_pwm(0, 0, int(current_speed))
    pwm.set_pwm(1, 0, int(current_steering))


if __name__ == '__main__':
    rospy.init_node('servo_control')
    rospy.Subscriber('/joy', Joy, joy_callback)
    print("starte servo control")    
    pwm = Adafruit_PCA9685.PCA9685(address=0x40)
    pwm.set_pwm_freq(60)

    pwm.set_pwm(0, 0, int(speed_neutral))
    pwm.set_pwm(1, 0, int(steering_default))

    rospy.spin()
