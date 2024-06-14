#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import sys
import os
import copy
import numpy as np
import json
from math import cos, sin, sqrt, pow, atan2, pi
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Odometry, Path
import time

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *
from a_star import AStar

class a_star_path_pub:
    def __init__(self):
        # rospy.init_node('a_star_path_pub', anonymous=True)

        # self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)

        # TODO: (1) Mgeo data 읽어온 후 데이터 확인
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/kcity'))
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.nodes = node_set.nodes
        self.links = link_set.lines

        self.global_planner = AStar(self.nodes, self.links)

        # TODO: (2) 시작 Node 와 종료 Node 정의 > edited to node list format
        ### ADD NODE list
        self.node_list = ['A119BS010209', 'A119BS010313', 'A119BS010316', 'A119BS010324', 'A119BS010184']

        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'
        self.global_path_msg = self.calc_a_star_path_node(self.node_list)

        # rate = rospy.Rate(10)  # 10hz
        # while not rospy.is_shutdown():
        #     # TODO: Global Path 정보 Publish
        #     self.global_path_pub.publish(self.global_path_msg)
        #     rate.sleep()

    def calc_a_star_path_node(self, node_list):

        # TODO: (10) 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
        out_path = Path()
        out_path.header.frame_id = '/map'

        for i in range(len(node_list) - 1):
            # start_time = time.time()  # Start timing
            success, node_path = self.global_planner.find_shortest_path(node_list[i], node_list[i + 1])
            # end_time = time.time()  # End timing
            # elapsed_time = end_time - start_time  # Calculate elapsed time
            # print(f"A_STAR Time taken to find shortest path from {node_list[i]} to {node_list[i + 1]}: {elapsed_time:.4f} seconds")

            if success:
                link_path = self.global_planner.create_link_path(node_path)
                point_path = self.global_planner.generate_point_path(link_path)

                for waypoint in point_path:
                    path_x = waypoint[0]
                    path_y = waypoint[1]
                    read_pose = PoseStamped()
                    read_pose.pose.position.x = path_x
                    read_pose.pose.position.y = path_y
                    read_pose.pose.orientation.w = 1
                    out_path.poses.append(read_pose)

        return out_path

if __name__ == '__main__':
    a_star_path_pub = a_star_path_pub()
