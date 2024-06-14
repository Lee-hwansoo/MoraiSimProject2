#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import random
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def euclidean_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None

    def point(self):
        return (self.x, self.y)

class RRTStar:
    def __init__(self, start, goal, obstacle_list, x_bounds, y_bounds, step_size=1.0, max_iter=1000):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacle_list = obstacle_list
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.step_size = step_size
        self.max_iter = max_iter
        self.node_list = [self.start]

    def plan_path(self):
        for i in range(self.max_iter):
            random_node = self.get_random_node()
            nearest_node = self.get_nearest_node(self.node_list, random_node)

            new_node = self.steer(nearest_node, random_node, self.step_size)

            if self.check_collision(nearest_node, new_node, self.obstacle_list):
                near_nodes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_nodes)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_nodes)

        return self.generate_final_course(len(self.node_list) - 1)

    def steer(self, from_node, to_node, extend_length=float('inf')):
        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calculate_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = int(extend_length / self.step_size)

        for _ in range(n_expand):
            new_node.x += self.step_size * np.cos(theta)
            new_node.y += self.step_size * np.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calculate_distance_and_angle(new_node, to_node)
        if d <= self.step_size:
            new_node.x = to_node.x
            new_node.y = to_node.y
            new_node.path_x[-1] = to_node.x
            new_node.path_y[-1] = to_node.y

        new_node.parent = from_node

        return new_node

    def get_random_node(self):
        if random.randint(0, 100) > 50:
            return Node(self.goal.x, self.goal.y)
        else:
            return Node(random.uniform(self.x_bounds[0], self.x_bounds[1]),
                        random.uniform(self.y_bounds[0], self.y_bounds[1]))

    def get_nearest_node(self, node_list, random_node):
        distances = [euclidean_distance(node.point(), random_node.point()) for node in node_list]
        min_index = distances.index(min(distances))
        return node_list[min_index]

    def check_collision(self, from_node, to_node, obstacle_list):
        for ox, oy, size in obstacle_list:
            dx = ox - to_node.x
            dy = oy - to_node.y
            d = dx * dx + dy * dy
            if d <= size ** 2:
                return False  # Collision
        return True  # Safe

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
        path = [[self.goal.x, self.goal.y]]
        node = self.node_list[goal_index]
        while node.parent:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path
    
    def calculate_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        distance = np.sqrt(dx**2 + dy**2)
        angle = np.arctan2(dy, dx)
        return distance, angle

if __name__ == '__main__':
    rospy.init_node('rrt_star_path_planner', anonymous=True)
    start = (3.3923792839050293, 1069.2960205078125)
    goal = (19.7967529296875, 1054.97412109375)
    obstacle_list = [(5, 5, 2), (3, 6, 2), (3, 8, 2), (3, 10, 2)]  # example: (x, y, radius)
    x_bounds = (-50, 50)
    y_bounds = (1000, 1100)

    rrt_star = RRTStar(start, goal, obstacle_list, x_bounds, y_bounds)
    path = rrt_star.plan_path()

    path_publisher = rospy.Publisher('/global_path', Path, queue_size=1)
    global_path_msg = Path()
    global_path_msg.header.frame_id = '/map'

    for x, y in path:
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        global_path_msg.poses.append(pose)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        path_publisher.publish(global_path_msg)
        rate.sleep()
