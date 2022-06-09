#!/usr/bin/env python
#-*-coding: utf-8 -*-
#-----------------------------------------------------
# Name: srv_avoid_that  (.py)
# Desc: 入室後、avoid_thatしつつ、目的までナビゲーションするノード
# Date: 2022/04/09
# Author: Hiroto Washio
#-----------------------------------------------------
import rospy
import actionlib
import smach
import smach_ros
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from yaml import load
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from std_srvs.srv import Empty
#from navigation_education.srv import value, valueResponse
import navi_location
import mimi_navigation
import door_open2

class AvoidThatServer():
    def __init__(self):
        service = ros.Service('/avoid_that_server', avoidThat,self.execute)
		rospy.loginfo('Start "avoid_that"')
		self.nv = mimi_navigation.Navigation()
		self.do = door_open2.DoorOpen()
		self.navi_srv = NaviLocation()
		self.rate = rospy.Rate(10)
		
    def execute(self, srv_req):
    	# DoorOpen
    	safe_dist = 2.0
    	while not rospy.is_shutdown():
    		if self.front_laser_dist >= safe_dist:
    			rospy.loginfo('Enter this room')
    			self.rate.sleep()
    			for i in range(1):
    				self.do.base_control.translateDist(2.0, 0.2))
    				rospy.loginfo('Finish "door open"')
    				return valueResponse(result = True)
    		elif self.front_laser_dist <= safe_dist:
    			rospy.loginfo('Please open this door')
    			self.rate.sleep()
    			
    	# Navigation
    	self.navi_srv = 
    	
    	
    	
    		
    	nv = mimi_navigation.Navigation()
        state = 0
        rospy.loginfo('Start "Navigation"')
        while not rospy.is_shutdown() and not state == 3:
            if state == 0:
                state = nv.input_value()
            elif state == 1:
                state = nv.searchLocationName()
            elif state == 2:
                state = nv.navigationAC()
        rospy.loginfo('Finish "Navigation"')

# door_openの状態作成（障害物がある状態）
class door_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes =['navigation_transition','door_transition'])

    def execute(self, userdata):
        door_main = door_open2.DoorOpen()
        if door_open2.DoorOpenResult == True:
            return 'navigation_transition'
        else:
            return 'door_transition'
#        safety_distance = 2.0
#        rospy.loginfo('start "door_open"')
#        sub = topic_door_enter.SubscriberClass()
#        pub = topic_door_enter.PublishClass()
#        while not rospy.is_shutdown():
#            door_state = sub.message_value()
#            if door_state >= safety_distance:
#                return 'navigation_transition'
#            else:
#                rospy.loginfo('There are obstacles')


if __name__ == '__main__':
    rospy.init_node('srv_avoid_that')
    main()
