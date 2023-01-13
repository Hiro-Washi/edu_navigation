#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------
# Desc: 
# Author: Hiroto Washio
# Date: 17/02/2022

# https://github.com/HKUST-Aerial-Robotics/VINS-Mono



#-------------------------------------------------------------------
import time, datetime
import sys
import rospy
from std_msgs.msg import String, Float64
import smach
import smach_ros
import roslib
from happymimi_msgs.srv import StrTrg
# !!
#from mimi_common_pkg.srv import ManipulateSrv, RecognizeCount
sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_voice_control/src')
from happymimi_voice_msgs.srv import *
from geometry_msgs.msg import Twist
from mimi_navi2.srv import NaviLocation #!!
from happymimi_recognition_msgs.srv import RecognitionFind, RecognitionFindRequest, RecognitionLocalizeRequest
base_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, base_path)
from base_control import BaseControl
reco_path = roslib.packages.get_pkg_dir('recognition_processing') + '/src/'
sys.path.insert(0, reco_path)
from recognition_tools import RecognitionTools
import roslib.packages
happymimi_voice_path = roslib.packages.get_pkg_dir("happymimi_voice")+"/../config/wave_data/aram.wav"
sec_happymimi_voice_path = roslib.packages.get_pkg_dir("happymimi_voice")+"/../config/wave_data/ga9du-ecghy2.wav"
from real_time_navi.srv import RealTimeNavi
from playsound import playsound
from send_gmail.srv import SendGmail
tts_srv = rospy.ServiceProxy('/tts', StrTrg)
rt = RecognitionTools()


class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['start_finish'])

    def execute(self, userdata):

class SearchPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['found_lying','found_standing',
                                            'not_found_one','not_found_two'])
 
    def execute(self, userdata):

class TalkAndAlert(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes = ['to_call','to_exit'])
       
    def execute(self, userdata):
        
class Call(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['call_finish'])

    def execute(self, userdata):

class Exit(smach.State):

if __name__ == '__main__':
    rospy.init_node('fall_down_resc')
    rospy.loginfo('Start "fall_down_resc"')
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    with sm_top:
        smach.StateMachine.add(
                'Start',
                Start(),
                transitions = {'start_finish':'SearchPerson'})
        smach.StateMachine.add(
                'SearchPerson',
                SearchPerson(),
                transitions = {'found_lying':'TalkAndAlert',
                               'found_standing':'Exit',
                               'not_found_one':'SearchPerson',
                               'not_found_two':'Exit'})
        smach.StateMachine.add(
                'TalkAndAlert',
                TalkAndAlert(),
                transitions = {'to_call':'Call',
                               'to_exit':'Exit'})
        smach.StateMachine.add(
                'Call',
                Call(),
                transitions = {'call_finish':'Exit'})
        smach.StateMachine.add(
                'Exit',
                Exit(),
                transitions = {'all_finish':'finish_sm_top'})
        
    outcome = sm_top.execute()
