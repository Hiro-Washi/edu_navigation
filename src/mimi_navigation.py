#!/usr/bin/env python3
#-*-coding: utf-8 -*-
#--------
#nIcE tO mE tU.
#This is source code to bring Happychan to a certain destination.nIcE tO mE tU.
#Author is me. aNd nIcE tO mE tU.
# Memo:     
#    //uint8 PENDING         = 0 The goal has yet to be processed by the action server
#    //uint8 ACTIVE          = 1 The goal is currently being processed by the action server
#    //uint8 PREEMPTED       = 2 The goal received a cancel request after it started executing
#                                and has since completed its execution (Terminal State)
#    //uint8 SUCCEEDED       = 3 The goal was achieved successfully by the action server (Terminal State)
#    //uint8 ABORTED         = 4 The goal was aborted during execution by the action server due
#                                to some failure (Terminal State)
#    //uint8 REJECTED        = 5
#    //uint8 PREEMPTING      = 6
#    //uint8 RECALLING       = 7
#    //uint8 RECALLED        = 8
#    //uint8 LOST            = 9
#--------
import sys, rospy, actionlib
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
        self.target_name = 'Shelf'
        while not rospy.is_shutdown() and self.target_name == 'NULL':
            print("Waiting for the message...")
            rospy.sleep(2.0)
        return 1

    def searchLocation(self):
        f = open('/home/hiroto/@home_ws/src/education_pkg/education_navigation/config/set_edu_map.yaml')
        #f = open('/home/hiroto/@home_ws/src/education_pkg/education_navigation/config/wh_yumeko_1.yaml')
        #f = open('/home/hiroto/@home_ws/src/mc_education/ros_noetic/navigation/location/yumekobo.yaml')
        location_dict = load(f)
        f.close()
        print("search the location name >> " + self.target_name)
        print(list(location_dict.keys()))
#        print(r[8:12] for r in location_dict)
        rospy.sleep(2.0)
        if self.target_name in location_dict:
            print("found " + str(self.target_name) + str(location_dict[self.target_name]) + " in location_dict")
            self.coord_list = location_dict[self.target_name]
            return 2
        elif  self.target_name == "Null":
            print("< Null >")
            return 0
        elif not self.target_name in location_dict:
            print("NOT found < " + str(self.target_name) + " > in location_dict")
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
            print("send goal")
            ac.send_goal(goal)
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
                        rospy.loginfo('Clear the costmaps')
                        rospy.sleep(1.0)
                        count += 1
#                elif state == 5:
#                    rospy.loginfo("It'srejected")
            rospy.sleep(2.0)
        except rospy.ROSInterruptException:
            pass
''' WAITING_FOR_GOAL_ACK    = 0,
    000PENDING              = 1,
    ACTIVE                  = 2,
    WAITING_FOR_RESULT      = 3,
    WAITING_FOR_CANCEL_ACK  = 4,
    RECALLING               = 5,
    PREEMPTING              = 6,
    DONE                    = 7'''
def main():
    nv = Navigation()
    state = 0
    rospy.loginfo('start "navigation"')
    while not rospy.is_shutdown() and not state == 3:
        rospy.sleep(0.2)
        if state == 0:
            state = nv.input_value()
        if state == 1:
            state = nv.searchLocation()
        if state == 2:
            state = nv.navigationAC()
    rospy.loginfo('Done with "Navigation"')

if __name__ == '__main__':
    rospy.init_node('mimi_navigation')
    main()
