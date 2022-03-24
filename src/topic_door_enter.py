#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SubscriberClass():
    def __init__(self):
        self.ranges_message = rospy.Subscriber('/scan', LaserScan, self.messageCB)
        self.laser = LaserScan()

    def messageCB(self, recieve_msg):
        self.laser = recieve_msg

    def messae_value(self):
        if bool(self.laser.ranges):
            value = self.laser.ranges[359]
            print 'value'
            return value

class PublishClass():
    def __init__(self):
        self.pub_message = rospy.Publisher('cmd_vel_mux/input/teleope', Twist,queue_size = 1)
        self.count = 1

    def linerContorol(self, value):
        twist_cmd = Twist()
        twist_cmd.linear.x = value
        rospy.sleep(0.1)
        self.pub_message.publish(twist_cmd)

def main():
    safety_distance = 2.0
    rospy.loginfo('start "open_door"')
    sub = SubscriberClass()
    pub = PublishClass()
    while not ropy.is_shutdown():
        state = sub.message_value()
        if state >= safety_distance:
            rospy.loginfo('start forward')
            for i in range(10):
                pub.linercontorol(0.1)
                break
        else:
            pass

if __name__ == '__main__':
    rospy.init_node('door_open')
    main()
