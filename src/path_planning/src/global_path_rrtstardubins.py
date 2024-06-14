#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rosbag
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from morai_msgs.msg  import ObjectStatusList
from tf.transformations import euler_from_quaternion
from rrtstardubins import *

class rrtstardubins_path_pub :
    def __init__(self):
        # rospy.init_node('rrt_star_dubins_path_planner', anonymous=True)
        # self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_callback)
        
        self.is_object = False
        self.path_calculated = False
        self.start = (3.39, 1069.296, -26.251*np.pi/180)
        # self.start = (5.92, 1067.31, -27*np.pi/180)
        # self.goal = (13.648, 1041.565, -117.374 * np.pi / 180)
        self.goal = (10.588, 1041.426, -117.125*np.pi/180)
        # self.goal = (10.482, 1041.228, -117.145 * np.pi / 180)
        # self.goal = (9.618, 1040.0186, -116.1514*np.pi/180)
        # self.goal = (8.19, 1021.49, -120.1 * np.pi / 180)
        # self.goal = (12.548, 1040.054, -116.71 * np.pi / 180)
        self.x_bounds = (-50, 50)
        self.y_bounds = (1000, 1100)
        
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'
        self.obstacle_list = []
        
        # if self.is_object and not self.path_calculated:
        #     self.global_path_msg = self.calc_rrtstar_path_node()

        # rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     if self.is_object and not self.path_calculated:
        #         self.global_path_msg = self.calc_rrtstar_path_node()
        #         self.path_calculated = True
        #     self.global_path_pub.publish(self.global_path_msg)
        #     rate.sleep()

    def calc_rrtstar_path_node(self):

        out_path = Path()
        out_path.header.frame_id = '/map'

        # rospy.loginfo(self.obstacle_list)
        self.rrt_star = RRTStarDubins(self.start, self.goal, self.obstacle_list, self.x_bounds, self.y_bounds)
    
        self.path = self.rrt_star.plan_path()
        
        for x, y, _ in self.path:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            out_path.poses.append(pose)

        return out_path
    
    def object_callback(self, msg):
        self.is_object=True
        self.object_data = msg
        
        self.obstacle_list = []
        for obstacle in self.object_data.obstacle_list:
            x = obstacle.position.x
            y = obstacle.position.y
            size_x = obstacle.size.x
            size_y = obstacle.size.y
            self.obstacle_list.append((x, y, size_x, size_y))

if __name__ == '__main__':
    dubins_path_pub = rrtstardubins_path_pub()
