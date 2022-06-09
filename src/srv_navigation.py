#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, rosparam, actionlib, sys
from yaml import load
from std_msgs.msg import String, Float64
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
sys.path.insert(0, '/home/hiroto/@home_ws/src/education_pkg/')
from education_navigation.srv import locationName, locationNameResponse

class NavigationServer():
    def __init__(self):
        print('Get ready for "navigation_server"')
        service = rospy.Service('navigation_server', locationName, self.searchLocation)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.crloc_pub = rospy.Publisher('/current_location', String, queue_size = 1)
        self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        #self.location_dict = rosparam.get_param('/location')
        self.location_name = "Null"
        self.rate = rospy.Rate(2)

    def searchLocation(self, request):
        print("Searching a location name")
        f = open("/home/hiroto/@home_ws/src/education_pkg/education_navigation/config/set_edu_map.yaml")
        location_dict = load(f)
        f.close()
        print(list(location_dict.keys()))
        self.location_name = "Null"
        if request.location_name in location_dict:
            self.location_name = request.location_name
            print(location_dict[self.location_name])
            return self.Goal(location_dict[self.location_name])
        else:
            rospy.logerr("<" + request.location_name + "> doesn't exist.")
            return locationNameResponse(result = False)

    def Goal(self, location_value):
        print('Start "Navigation"')
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = location_value[0]
        goal.target_pose.pose.position.y = location_value[1]
        goal.target_pose.pose.orientation.z = location_value[2]
        goal.target_pose.pose.orientation.w = location_value[3]
        rospy.loginfo("Clearing costmap...")
        rospy.wait_for_service('move_base/clear_costmaps')
        self.clear_costmap()
        self.rate.sleep()
        
        self.client.send_goal(goal)
        # navigation state
        while not rospy.is_shutdown():
            state = self.client.get_state()
            if state == 3:
                rospy.loginfo('Navigation success!!')
                self.crloc_pub.publish(self.location_name)
                return locationNameResponse(result = True)
            elif state == 4:
                rospy.loginfo('Navigation Failed')
                return locationNameResponse(result = False)
            else:
                pass

def main():
    NS = NavigationServer()
    state = 0
    while not rospy.is_shutdown() and not state == 3:
        rospy.sleep(0.2)
        if state == 0 and state == 1:
            state = NS.searchLocation()
        if state == 2:
            state = NS.Goal()
    rospy.loginfo('Done "navigation"')

if __name__ == '__main__':
    rospy.init_node('navigation_server', anonymous = True)
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
