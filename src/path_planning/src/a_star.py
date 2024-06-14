#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import heapq
import math
import numpy as np
from math import cos, sin, sqrt, atan2, pi

class AStar:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links
        self.weight = self.get_weight_matrix()
        self.lane_change_link_idx = []

    def get_weight_matrix(self):
        weight = dict()
        for from_node_id, from_node in self.nodes.items():
            weight_from_this_node = dict()
            for to_node_id, to_node in self.nodes.items():
                weight_from_this_node[to_node_id] = float('inf')
            weight[from_node_id] = weight_from_this_node

        for from_node_id, from_node in self.nodes.items():
            weight[from_node_id][from_node_id] = 0
            for to_node in from_node.get_to_nodes():
                shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node, to_node)
                weight[from_node_id][to_node.idx] = min_cost

        return weight

    def find_shortest_link_leading_to_node(self, from_node, to_node):
        to_links = []
        for link in from_node.get_to_links():
            if link.to_node is to_node:
                to_links.append(link)

        if len(to_links) == 0:
            raise BaseException('[ERROR] Error @ AStar.find_shortest_path : Internal data error. There is no link from node (id={}) to node (id={})'.format(self.idx, to_node.idx))

        shortest_link = None
        min_cost = float('inf')
        for link in to_links:
            if link.cost < min_cost:
                min_cost = link.cost
                shortest_link = link

        return shortest_link, min_cost

    def heuristic(self, node1, node2):
        dx = node1.point[0] - node2.point[0]
        dy = node1.point[1] - node2.point[1]
        return math.sqrt(dx * dx + dy * dy)

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path

    def find_shortest_path(self, start_node_idx, end_node_idx):
        open_set = []
        heapq.heappush(open_set, (0, start_node_idx))
        came_from = {}
        g_score = {node: float('inf') for node in self.nodes}
        g_score[start_node_idx] = 0
        f_score = {node: float('inf') for node in self.nodes}
        f_score[start_node_idx] = self.heuristic(self.nodes[start_node_idx], self.nodes[end_node_idx])

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == end_node_idx:
                return True, self.reconstruct_path(came_from, current)

            for neighbor in self.nodes[current].get_to_nodes():
                tentative_g_score = g_score[current] + self.weight[current][neighbor.idx]
                if tentative_g_score < g_score[neighbor.idx]:
                    came_from[neighbor.idx] = current
                    g_score[neighbor.idx] = tentative_g_score
                    f_score[neighbor.idx] = g_score[neighbor.idx] + self.heuristic(neighbor, self.nodes[end_node_idx])
                    if neighbor.idx not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor.idx], neighbor.idx))

        raise Exception("No path found")

    def generate_lane_change_trajectory(self, start_link, end_link, lane_change_distance, step_size):
        output_path = []

        translation = [start_link.points[0][0], start_link.points[0][1]]
        theta = atan2(start_link.points[1][1] - start_link.points[0][1], start_link.points[1][0] - start_link.points[0][0])

        t = np.array([
            [cos(theta), -sin(theta), translation[0]],
            [sin(theta), cos(theta), translation[1]],
            [0, 0, 1]
        ])

        det_t = np.linalg.inv(t)
        world_end_link_list = []
        for point in end_link.points:
            world_end_link_list.append([point[0], point[1], 1])

        world_end_link_metrix = np.array(world_end_link_list).T
        local_end_link_metrix = det_t.dot(world_end_link_metrix).T

        min_dis = float('inf')
        local_end_point_list = []

        for point in local_end_link_metrix:
            if point[0] > 0:
                dis = abs(sqrt(point[0] * point[0] + point[1] * point[1]) - lane_change_distance)
                if dis < min_dis:
                    min_dis = dis
                    local_end_point_list = [[point[0]], [point[1]], [1]]

        local_end_point_matrix = np.array(local_end_point_list)

        x = []
        y = []
        x_interval = step_size
        xs = 0
        xf = local_end_point_matrix[0][0]

        ps = 0.0
        pf = local_end_point_matrix[1][0]

        x_num = int(xf / x_interval)
        for i in range(xs, x_num):
            x.append(i * x_interval)

        a = [0.0, 0.0, 0.0, 0.0]
        a[0] = ps
        a[2] = 3.0 * (pf - ps) / (xf ** 2)
        a[3] = -2.0 * (pf - ps) / (xf ** 3)
        for i in x:
            result = a[3] * i ** 3 + a[2] * i ** 2 + a[0]
            y.append(result)

        for i in range(0, len(y)):
            local_change_path = np.array([[x[i]], [y[i]], [1]])
            global_change_path = t.dot(local_change_path)
            output_path.append([global_change_path[0][0], global_change_path[1][0], 0])

        end_point_index = 0
        for i, end_point in enumerate(local_end_link_metrix.tolist()):
            if end_point[0] == local_end_point_matrix[0][0] and end_point[1] == local_end_point_matrix[1][0]:
                end_point_index = i
                break

        for end_point in end_link.points[end_point_index:]:
            output_path.append([end_point[0], end_point[1], 0])

        return output_path

    def create_link_path(self, node_path):
        link_path = []
        for i in range(len(node_path) - 1):
            from_node_idx = node_path[i]
            to_node_idx = node_path[i + 1]
            from_node = self.nodes[from_node_idx]
            to_node = self.nodes[to_node_idx]
            shortest_link, _ = self.find_shortest_link_leading_to_node(from_node, to_node)
            link_path.append(shortest_link.idx)
        return link_path

    def generate_point_path(self, link_path):
        point_path = []
        user_option_draw_lane_change = True

        for link_id in link_path:
            link = self.links[link_id]
            if link.is_it_for_lane_change() and user_option_draw_lane_change:
                lane_ch_pair_list = link.get_lane_change_pair_list()
                lane_ch_first = lane_ch_pair_list[0]
                lane_ch_last = lane_ch_pair_list[-1]
                lane_ch_distance = 20 * link.get_number_of_lane_change()

                output_path = self.generate_lane_change_trajectory(
                    lane_ch_first['from'], lane_ch_last['to'], lane_ch_distance, 1)

                output_path = np.array(output_path)
                x, y = output_path[:, 0], output_path[:, 1]
                for xi, yi in zip(x, y):
                    point_path.append([xi, yi, 0])

            else:
                for point in link.points:
                    point_path.append([point[0], point[1], 0])

        return point_path