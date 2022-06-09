#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SubscriberClass():
    def __init__(self):
        self.ranges_message = rospy.Subscriber('/scan', LaserScan, self.laserCB)
        #self.laser_dist = LaserScan()
        self.laser_dist = 999.9

    def laserCB(self, recieve_msg):
        self.laser_dist = recieve_msg.ranges[359]

 #   def message_value(self):
 #       if bool(self.laser.ranges):
 #           value = self.laser.ranges[359]
 #           print('value')
 #           return value

class PublisherClass():
    def __init__(self):
        self.pub_message = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist,queue_size = 1)
        self.count = 1

    def linerControl(self, value):
        twist_cmd = Twist()
        twist_cmd.linear.x = value
        rospy.sleep(0.1)
        self.pub_message.publish(twist_cmd)

def main():
    safety_distance = 2.0
    rospy.loginfo('\nstart "open_door"')
    sub = SubscriberClass()
    pub = PublisherClass()
    while not rospy.is_shutdown():
        if sub.laser_dist > safety_distance:
            rospy.loginfo('start forward')
            for i in range(10):
                pub.linerControl(0.1)
                break
        else:
            pass

if __name__ == '__main__':
    rospy.init_node('door_open')
    main()
