#! /usr/bin/env python
import rospy
from rc_car_control.msg import Control 

if __name__ == '__main__':
    rospy.init_node('kb_control')
    pub = rospy.Publisher('control', Control, queue_size=10)

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        pub.publish('hello')
        r.sleep()
