#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
# import rosbag
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from dubins_ccrs import *

class ccrs_path_pub :
    def __init__(self):
        rospy.init_node('ccrs_path_pub', anonymous=True)

        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size = 1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.kappa_ = 1./6
        self.ccrs = CCRS()
        self.current_position = PoseStamped().pose.position
        self.vehicle_yaw = 0
        
        self.waypoints = [[-24.27404157264391, 1022.6089978003874, 91.746*np.pi/180],
                          [-18.801, 1040.58, 67*np.pi/180],
                          [-5.7, 1065.7, 67*np.pi/180],
                          [5.92, 1067.31, -27*np.pi/180],
                          [19.796, 1054.974, -87.727*np.pi/180]
                          ]
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'
        
        self.global_path_msg = self.calc_ccrs_path_node()
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # self.global_path_msg = self.calc_ccrs_path_node()
            self.global_path_pub.publish(self.global_path_msg)
            rate.sleep()

    def calc_ccrs_path_node(self):

        out_path = Path()
        out_path.header.frame_id = '/map'

        for i in range(len(self.waypoints) - 1):
            state1 = self.waypoints[i]
            state2 = self.waypoints[i+1]
            cartesian_path, controls, ccrs_path = self.ccrs.plan(state1, state2, self.kappa_)
            for x,y,_ in zip(*cartesian_path):
                path_x = x
                path_y = y
                read_pose = PoseStamped()
                read_pose.pose.position.x = path_x
                read_pose.pose.position.y = path_y
                out_path.poses.append(read_pose)

        return out_path
    
    def odom_callback(self,msg):
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.waypoints[0] = [self.current_position.x, self.current_position.y, self.vehicle_yaw]
        # rospy.loginfo("waypoints[0] : %s", self.waypoints[0])
            
if __name__ == '__main__':
    ccrs_path_pub = ccrs_path_pub()
