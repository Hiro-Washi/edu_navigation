#!/usr/bin/env python
#-*-coding: utf-8 -*-
#--------
#nIcE tO mE tU.
#This is source code to bring Happychan to a certain destination.nIcE tO mE tU.
#Author is me. aNd nIcE tO mE tU.
#--------
import sys
import rospy
import actionlib
from yaml import load
from std_msgs.msg import String
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

class Navigation():
    def __init__(self):
        self.coord_list = []
        self.sub_message = rospy.Subscriber('/input_target', String, self.messageCB)
        self.target_name = 'NULL'

    def messageCB(self,receive_msg):
        self.target_name = receive_msg.data

    def input_value(self):
        self.target_name = 'Null'
        while not rospy.is_shutdown() and self.target_name == 'NULL':
            print "Waiting for the message..."
            rospy.sleep(2.0)
        return 1

    def searchLocationName(self):
        rospy.loginfo("search LocationName")
        f = open('set_edu_map.yaml')
        location_dict = load(f)
        f.close()
        print self.target_name
        rospy.sleep(2.0)
        if self.target_name in location_dict:
            print location_dict[self.target_name]
            rospy.loginfo("Return location_dict")
            self.coord_list = location_dict[self.target_name]
            return 2
        else:
            rospy.loginfo("NOT found<" + str(self.target_name) + "> in LocationDict")
            return 0

    def navigationAC(self):
        try:
            rospy.loginfo("Start Navigation")
            ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            ac.wait_for_server()
            print("Got action")
            clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = self.coord_list[0]
            goal.target_pose.pose.position.y = self.coord_list[1]
            goal.target_pose.pose.orientation.z = self.coord_list[2]
            goal.target_pose.pose.orientation.w = self.coord_list[3]
            rospy.wait_for_service('move_base/clear_costmaps')
            clear_costmaps()
            rospy.sleep(0.5)
            ac.send_goal(goal)
            print("send goal")
            count = 0
            while not rospy.is_shutdown():
                state = ac.get_state()
                if state == 1:
                    rospy.loginfo('Got out of the obstacle')
                    rospy.sleep(1.0)
                elif state == 3:
                    rospy.loginfo('Navigation success!')
                    return 0
                elif state == 4:
                    if count == 10:
                        count = 0
                        rospy.loginfo('Navigation success!')
                        return 2
                    else:
                        rospy.loginfo('Buried in obstacle')
                        clear_costmaps()
                        rospy.loginfo('Clear Costmaps')
                        rospy.sleep(1.0)
                        count += 1
            rospy.sleep(2.0)
        except rospy.ROSInterruptException:
            pass

def main():
    nv = Navigation()
    state = 0
    rospy.loginfo('start "navigation"')
    while not rospy.is_shutdown() and not state == 3:
        rospy.sleep(0.2)
        if state == 0:
            state = nv.input_value()
        if state == 1:
            state = nv.searchLocationName()
        if state == 2:
            state = nv.navigationAC()
    rospy.loginfo('Finish "Navigation"')

if __name__ == '__main__':
    rospy.init_node('mimi_navigation.py')
    main()
