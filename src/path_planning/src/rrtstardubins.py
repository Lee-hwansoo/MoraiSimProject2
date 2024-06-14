#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import random
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from dubins import Dubins, DubinsPath, DubinsControl, twopify, pify

def euclidean_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

class Node:
    def __init__(self, x, y, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.cost = 0.0
        self.parent = None

    def point(self):
        return (self.x, self.y)

class RRTStarDubins:
    def __init__(self, start, goal, obstacle_list, x_bounds, y_bounds, step_size=0.07, max_iter=3000, turning_radius=2.5 * np.pi):
        self.start = Node(start[0], start[1], start[2])
        self.goal = Node(goal[0], goal[1], goal[2])
        self.obstacle_list = obstacle_list
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.step_size = step_size
        self.max_iter = max_iter
        self.node_list = [self.start]
        self.dubins = Dubins()
        self.turning_radius = turning_radius

    def plan_path(self):
        random.seed(20)
        for i in range(self.max_iter):
            random_node = self.get_random_node()
            print("get")
            nearest_node = self.get_nearest_node(self.node_list, random_node)

            new_node = self.steer(nearest_node, random_node, self.step_size)

            if self.check_collision(nearest_node, new_node, self.obstacle_list):
                near_nodes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_nodes)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_nodes)
        # print("course")
        return self.generate_final_course(len(self.node_list) - 1)

    def steer(self, from_node, to_node, extend_length=float('inf')):
        new_node = Node(from_node.x, from_node.y, from_node.yaw)
        path, controls, dubins_path = self.dubins.plan((from_node.x, from_node.y, from_node.yaw),
                                                       (to_node.x, to_node.y, to_node.yaw),
                                                       1.0 / self.turning_radius)

        if path:
            n_expand = min(len(path[0]), int(extend_length / self.step_size))

            for i in range(n_expand):
                new_node.x = path[0][i]
                new_node.y = path[1][i]
                new_node.yaw = path[2][i]

            new_node.path_x = path[0][:n_expand]
            new_node.path_y = path[1][:n_expand]
            new_node.path_yaw = path[2][:n_expand]

            new_node.parent = from_node
            new_node.cost = from_node.cost + self.calculate_cost(new_node.path_x, new_node.path_y)

        return new_node

    def get_random_node(self):
        if random.randint(0, 100) > 50:
            return Node(self.goal.x, self.goal.y, self.goal.yaw)
        else:
            return Node(random.uniform(self.x_bounds[0], self.x_bounds[1]),
                        random.uniform(self.y_bounds[0], self.y_bounds[1]),
                        random.uniform(-np.pi, np.pi))

    def get_nearest_node(self, node_list, random_node):
        distances = [euclidean_distance(node.point(), random_node.point()) for node in node_list]
        min_index = distances.index(min(distances))
        return node_list[min_index]

    def check_collision(self, from_node, to_node, obstacle_list):
        for ox, oy, size_x, size_y in obstacle_list:
            dx = ox - to_node.x
            dy = oy - to_node.y
            d = dx ** 2 + dy ** 2
            if d <= ((2 * 2) ** 2):
                return False
        return True

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = 50.0 * np.sqrt((np.log(nnode) / nnode))
        dlist = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2 for node in self.node_list]
        near_nodes = [self.node_list[idx] for idx in range(len(dlist)) if dlist[idx] <= r ** 2]
        return near_nodes

    def choose_parent(self, new_node, near_nodes):
        if not near_nodes:
            return None

        dlist = []
        for node in near_nodes:
            if self.check_collision(node, new_node, self.obstacle_list):
                dx = new_node.x - node.x
                dy = new_node.y - node.y
                d = np.sqrt(dx ** 2 + dy ** 2)
                theta = np.arctan2(dy, dx)
                if node.cost + d < new_node.cost:
                    new_node.cost = node.cost + d
                    new_node.parent = node
        return new_node

    def rewire(self, new_node, near_nodes):
        for node in near_nodes:
            dx = new_node.x - node.x
            dy = new_node.y - node.y
            d = np.sqrt(dx ** 2 + dy ** 2)
            scost = new_node.cost + d

            if node.cost > scost:
                if self.check_collision(new_node, node, self.obstacle_list):
                    node.parent = new_node
                    node.cost = scost

    def generate_final_course(self, goal_index):
        path = [[self.goal.x, self.goal.y, self.goal.yaw]]
        node = self.node_list[goal_index]
        while node.parent:
            path.append([node.x, node.y, node.yaw])
            node = node.parent
        path.append([node.x, node.y, node.yaw])

        return path[::-1]
    
    def calculate_cost(self, path_x, path_y):
        cost = 0.0
        for i in range(1, len(path_x)):
            cost += np.sqrt((path_x[i] - path_x[i-1]) ** 2 + (path_y[i] - path_y[i-1]) ** 2)
        return cost