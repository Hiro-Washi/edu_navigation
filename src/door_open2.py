#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-----------------------------------------------------------
#Title: ドアが開いたときに進むシンプルなノード(トピック通信)
#Author: Shunsuke Wada
#Data: 2022/3/22
#Memo: base_control.pyを用いて進む距離と速度を指定し走らせる
#Memo: 自作モジュールの git cloneが必要
#-----------------------------------------------------------
import rospy 
import roslib
import sys 
from sensor_msgs.msg import LaserScan
file_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, file_path)
from base_control import BaseControl

class DoorOpen():
    def __init__(self):
        #サブスクライバーの宣言
        rospy.Subscriber('/scan', LaserScan, self.laserCB)
        #値の初期化
        self.front_laser_dist = 999.9
        self.base_control = BaseControl()
        rospy.loginfo('start door open')

    def laserCB(self, receive_msg):
        self.front_laser_dist = receive_msg.ranges[359]

    def execute(self):
        #安全距離
        safe_dist = 2.0
        #進む速度と距離
        distance = 2.0
        velocity = 0.2
        rospy.sleep(0.3)
        while not rospy.is_shutdown():
            #前に障害物がなかったとき
            if self.front_laser_dist >= safe_dist:
                rospy.loginfo('start forward')
                rospy.sleep(0.1)
                #一回繰り返す
                for i in range(1):
                    #メソッドを呼び出し
                    self.base_control.translateDist(distance, velocity)
                    rospy.loginfo('finish door open')
                    break
                break
            #前に障害物があるとき
            elif self.front_laser_dist <= safe_dist:
                rospy.loginfo('Please open the door')
                rospy.sleep(3.0)

if __name__ == '__main__':
    rospy.init_node('door_open2')
    do = DoorOpen()
    do.execute()
    
