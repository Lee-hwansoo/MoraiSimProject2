#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from global_path_dijkstra import dijkstra_path_pub
from global_path_dubins import dubins_path_pub
from global_path_ccrs import ccrs_path_pub
from global_path_rrtstardubins import rrtstardubins_path_pub
from global_path_a_star import a_star_path_pub
from morai_msgs.msg import ObjectStatusList
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String
import numpy as np
import math
import rosnode

class CombinedPathPub:
    def __init__(self):
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        rospy.Subscriber("/mode", String, self.mode_callback)
        # rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_callback)

        # self.dijkstra_planner = dijkstra_path_pub()
        self.a_star_planner = a_star_path_pub()
        self.dubins_planner = dubins_path_pub()
        self.parking_planner = dubins_path_pub()
        self.rrtstar_planner = rrtstardubins_path_pub()
        self.mode = None
        # self.is_object = False
        
    def publish_global_path(self):
        
        a_star_path = self.a_star_planner.calc_a_star_path_node(self.a_star_planner.node_list)
        # dijkstra_path = self.dijkstra_planner.calc_dijkstra_path_node(self.dijkstra_planner.node_list)
        last_point = a_star_path.poses[-1].pose.position

        self.dubins_planner.waypoints[0] = [last_point.x, last_point.y, 91 * np.pi / 180]
        dubins_path = self.dubins_planner.calc_dubins_path_node()
        
        self.parking_planner.waypoints=[
                                        [5.92, 1067.31, -27*np.pi/180],
                                        [12.60, 1058.37, -117.16*np.pi/180],
                                        [14.71, 1044.34, -116.21*np.pi/180],
                                        [11.81, 1038.76, -116.21*np.pi/180]
                                        ]
        
        combined_path = Path()
        combined_path.header.frame_id = '/map' 
        combined_path.poses = a_star_path.poses + dubins_path.poses
        # combined_path.poses = dijkstra_path.poses + dubins_path.poses
        
        
        
        # rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     if self.mode == "HIGHWAY MODE":
        #         self.global_path_pub.publish(combined_path)
        #     elif self.mode == "PARKING MODE":
        #         if not self.rrtstar_planner.path_calculated:
        #             rrtstar_path = self.rrtstar_planner.calc_rrtstar_path_node()
        #             self.rrtstar_planner.path_calculated = True
        #             combined_path.poses = rrtstar_path.poses
        #             # print(combined_path.poses[-1])
        #             self.global_path_pub.publish(combined_path)
        #     rate.sleep()
        
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.mode == "HIGHWAY MODE":
                self.global_path_pub.publish(combined_path)
            elif self.mode == "PARKING MODE":
                parking_path = self.parking_planner.calc_dubins_path_node()
                combined_path.poses = parking_path.poses
                self.global_path_pub.publish(combined_path)
            rate.sleep()
    
    # def object_callback(self, msg):
    #     self.is_object=True
    #     self.object_data = msg
        
    #     self.obstacle_list = []
    #     for obstacle in self.object_data.obstacle_list:
    #         x = obstacle.position.x
    #         y = obstacle.position.y
    #         size_x = obstacle.size.x
    #         size_y = obstacle.size.y
    #         self.obstacle_list.append((x, y, size_x, size_y))
    
    def mode_callback(self, msg):
        self.mode = msg.data

if __name__ == '__main__':
    rospy.init_node('combined_path_pub', anonymous=True)
    path_publisher = CombinedPathPub()
    path_publisher.publish_global_path()
