#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import numpy as np
import heapq
from math import cos, sin, sqrt, pow, atan2, pi

class Dijkstra:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links
        self.weight = self.get_weight_matrix()
        self.lane_change_link_idx = []

    def get_weight_matrix(self):
        # 초기 설정
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
        to_links = [link for link in from_node.get_to_links() if link.to_node is to_node]
        if not to_links:
            raise BaseException('[ERROR] Error @ Dijkstra.find_shortest_path : Internal data error. There is no link from node (id={}) to node (id={})'.format(from_node.idx, to_node.idx))

        shortest_link = min(to_links, key=lambda link: link.cost)
        return shortest_link, shortest_link.cost

    def find_shortest_path(self, start_node_idx, end_node_idx):
        s = {node_id: False for node_id in self.nodes}
        from_node = {node_id: start_node_idx for node_id in self.nodes}
        distance = {node_id: float('inf') for node_id in self.nodes}
        distance[start_node_idx] = 0

        priority_queue = [(0, start_node_idx)]
        while priority_queue:
            current_distance, selected_node_idx = heapq.heappop(priority_queue)
            if s[selected_node_idx]:
                continue

            s[selected_node_idx] = True
            for to_node in self.nodes[selected_node_idx].get_to_nodes():
                to_node_idx = to_node.idx
                if not s[to_node_idx]:
                    distance_candidate = current_distance + self.weight[selected_node_idx][to_node_idx]
                    if distance_candidate < distance[to_node_idx]:
                        distance[to_node_idx] = distance_candidate
                        from_node[to_node_idx] = selected_node_idx
                        heapq.heappush(priority_queue, (distance_candidate, to_node_idx))

        node_path = []
        tracking_idx = end_node_idx
        while tracking_idx != start_node_idx:
            node_path.append(tracking_idx)
            tracking_idx = from_node[tracking_idx]
        node_path.append(start_node_idx)
        node_path.reverse()

        link_path = self.create_link_path(node_path)

        if not link_path:
            return False, {'node_path': node_path, 'link_path': link_path, 'point_path': []}

        point_path = self.generate_point_path(link_path)
        
        return True, {'node_path': node_path, 'link_path': link_path, 'point_path': point_path}

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

                output_path = self.draw_lange_change(
                    lane_ch_first['from'], lane_ch_last['to'], lane_ch_distance, 1)

                output_path = np.array(output_path)
                x, y = output_path[:, 0], output_path[:, 1]
                for xi, yi in zip(x, y):
                    point_path.append([xi, yi, 0])

            else:
                for point in link.points:
                    point_path.append([point[0], point[1], 0])

        return point_path

    def draw_lange_change(self, start_link, end_link, lane_change_distance, step_size):
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
