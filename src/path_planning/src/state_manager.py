#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from shapely.geometry import Point, Polygon

class StatetManager:
    def __init__(self):
        rospy.init_node('state_manager', anonymous=True)
        self.mode_pub = rospy.Publisher('mode', String, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        
        self.parking_area = Polygon([
            (-20.4306, 1030.4216),
            (2.4075, 1073.8303),
            (42.0613, 1053.5362),
            (18.6102, 1011.4155)
        ])
        
        self.parking = Polygon([
            (16.88, 1039.94),
            (14.92, 1036.30),
            (8.2419, 1039.8883),
            (10.35, 1043.66)
        ])
        
        # 초기 모드 설정
        self.mode = "HIGHWAY MODE"
        self.current_position = Point(0, 0)  # 초기 위치 설정
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.parking.contains(self.current_position):
                self.mode = "FINISH MODE"
            elif self.parking_area.contains(self.current_position):
                self.mode = "PARKING MODE"
            else:
                self.mode = "HIGHWAY MODE"
                
            mode_msg = String()
            mode_msg.data = self.mode
            self.mode_pub.publish(mode_msg)    
            rate.sleep()

    def odom_callback(self, msg):
        self.current_position = Point(msg.pose.pose.position.x, msg.pose.pose.position.y)

if __name__ == '__main__':
    try:
        StatetManager()
    except rospy.ROSInterruptException:
        pass