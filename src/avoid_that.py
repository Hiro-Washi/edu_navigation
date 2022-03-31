#!/usr/bin/env python
#-*-coding: utf-8 -*-
#-----------------------------------------------------
# Desc: みみがよけていっちまうよ〜〜
# Date: 2022/03/17
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
import mimi_navigation
import door_open2

# ナビゲーションの状態作成
class navigation_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['door_transition', 'navigation_fin'])

    def execute(self, userdata):
        nv = mimi_navigation.Navigation()
        state = 0
        rospy.loginfo('Start navigation')
        while not rospy.is_shutdown() and not state == 3:
            if state == 0:
                state = nv.input_value()
            elif state == 1:
                state = nv.searchLocationName()
            elif state == 2:
                sate = nv.navigationAC()
        rospy.loginfo('Finish "Navigation"')
        return 'navigation_fin'

# door_openの状態作成（障害物がある状態）
class door_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['navigation_transition'])

    def execute(self, userdata):
        door_main = door_open2.DoorOpen()
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

def main():
    sm = smach.StateMachine(outcomes=['success', 'false'])
    # 状態機械に対して状態を登録
    with sm:
        # 出力結果と遷移先を登録
        smach.StateMachine.add('navigation_state', navigation_state(), transitions = {'door_transition':'door_state','navigation_fin':'success'})
        smach.StateMachine.add('door_state', door_state(), transitions = {'navigation_transition':'navigation_state'})

    # 状態機械の状態を外部に出力する手続き
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # 状態機会を実行
    outcome = sm.execute()

if __name__ == '__main__':
    rospy.init_node('avoid_that')
    main()
