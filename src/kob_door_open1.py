#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------------------------------
#Title: ドアが開いてるときに進むノード(トピック通信)
#Author: Shunsuke Wada
#Data: 2022/5/10
#Memo: 進む速度と距離から目標タイムを計算し、目標タイムに到達したときプログラムを終了させる。
#Memo: 扉が空いた状態で走らせても問題ない
#--------------------------------------------------------------------------------------------

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class EnterRoom():
    def __init__(self):
        #パブリッシャの宣言
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)
        #サブスクライバーの宣言
        rospy.Subscriber('/scan', LaserScan, self.laserCB)
        #値の初期化
        self.front_laser_dist = 999.9

    def laserCB(self, receive_msg):
        #LIDERの視野角
        self.front_laser_dist = receive_msg.ranges[359]
    
    def execute(self):
        vel = Twist()
        #進む距離と速度
        vel.linear.x = 0.2
        target_dist = 0.5
        #安全距離
        safe_dist = 0.4
        #目標タイムの計算
        target_time = target_dist / vel.linear.x
        #実行開始時間を格納
        start_time = time.time()
        #値の初期化
        stop_time = 0
        stop_start = 0
        stop_end = 0
        rospy.loginfo('Start door open')
        rospy.sleep(0.5)
        
        door_count = 0
        while not rospy.is_shutdown():
            #安全距離のとき実行
            if self.front_laser_dist >= safe_dist:
                self.twist_pub.publish(vel)
                rospy.sleep(0.5)
                    
            #障害物があるとき実行
            elif self.front_laser_dist <= safe_dist:
                door_count =+ 1
                #現在の時間を格納
                stop_start = time.time()
                rospy.loginfo('Please open door')
                rospy.sleep(3.0)

                #止まっていた時間の格納
                stop_end = time.time() - stop_start
                stop_time += stop_end
            #終了時刻を格納
            finish_time = time.time() - start_time - stop_time
            #時間計測
            print("now_time = ", time.time() - start_time - stop_time) 
            #目標タイムに到達したとき実行
            if finish_time >= target_time:
                rospy.loginfo('End door open')
                break
            
if __name__ == '__main__':
    rospy.init_node('door_open1')
    er = EnterRoom()
    er.execute()
    rospy.spin()


