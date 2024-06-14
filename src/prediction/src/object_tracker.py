#!/usr/bin/env python3
# coding: utf-8
import numpy as np
from state_filters import Extended_KalmanFilter, IMM_filter
from state_models import CA, CTRA

import rospy
from math import sqrt, pow, radians
from morai_msgs.msg  import ObjectStatusList
from prediction.msg import TrackedPoint, PredictedObjectPath, PredictedObjectPathList, TrackedObjectPose, TrackedObjectPoseList
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry


class DynamicObstacleTracker:
    def __init__(self, dt=0.1, T=1):
        mat_trans = np.array([[0.85, 0.15],
                              [0.15, 0.85]])

        mu = [0.8, 0.2]

        self.dt = dt
        self.T = T

        self.filters = [Extended_KalmanFilter(5, 4),
                        Extended_KalmanFilter(6, 4)]

        self.models = [CA(self.dt), CTRA(self.dt)]

        self.Q_list = [[0.1, 0.1, 0.1, 0.1, 0.1],
                       [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]] # process noise

        for i in range(len(self.filters)):
            self.filters[i].F = self.models[i].step
            self.filters[i].H = self.models[i].H
            self.filters[i].JA = self.models[i].JA
            self.filters[i].JH = self.models[i].JH
            self.filters[i].Q = np.diag(self.Q_list[i])
            self.filters[i].R = np.diag([0.05, 0.05, 0.05, 0.05]) # measurement noise, [x, y, v, theta]

        self.IMM = IMM_filter(self.filters, mu, mat_trans)
        self.MM = [mu]
        self.X = []

    def initialize(self, data):
        # data = [x, y, h, v]
        # x = [x, y, v, a, theta], [x, y, v, a, theta, theta_rate]

        # temp data
        x = [np.array([data[0], data[1], data[3], 0, data[2]]),
                     np.array([data[0], data[1], data[3], 0, data[2], 0])]

        for i in range(len(self.filters)):
            self.filters[i].x = x[i]

        self.X.append(x[1])

    def update(self, data):
        # data = [x, y, h, v]
        # z = [x, y, v, theta]

        # temp data
        z = [data[0], data[1],
                data[3], data[2]]

        self.IMM.prediction()
        self.IMM.merging(z)

        while len(self.MM) > 10:
            self.MM.pop(0)
        while len(self.X) > 10:
            self.X.pop(0)

        self.MM.append(self.IMM.mu.copy())
        self.X.append(self.IMM.x.copy())

    def predict(self):
        traj = self.IMM.predict(self.T)
        return traj


class MultiDynamicObstacleTracker:
    def __init__(self, dt=0.1, T=1, timeout=1.0):
        self.trackers = {}
        self.dt = dt
        self.T = T
        self.timeout = timeout  # 객체의 타임아웃 시간 (초)

        self.deleted_ids = set()  # 삭제된 객체 ID를 추적하기 위한 집합 추가

    def add_tracker(self, obj_id):
        if obj_id not in self.trackers.keys():
            self.trackers[obj_id] = {
                'tracker': DynamicObstacleTracker(dt=self.dt, T=self.T),
                'last_update_time': rospy.Time.now()  # 객체가 마지막으로 업데이트된 시간 초기화
            }

    def initialize(self, obj_id, data):
        if obj_id in self.trackers.keys():
            self.trackers[obj_id]['tracker'].initialize(data)
            self.trackers[obj_id]['last_update_time'] = rospy.Time.now()  # 업데이트된 시간 갱신
        else:
            self.add_tracker(obj_id)
            self.trackers[obj_id]['tracker'].initialize(data)
            self.trackers[obj_id]['last_update_time'] = rospy.Time.now()  # 업데이트된 시간 갱신

    def clean(self):
        current_time = rospy.Time.now()
        to_delete = []
        for obj_id in self.trackers.keys():
            if current_time - self.trackers[obj_id]['last_update_time'] > rospy.Duration(self.timeout):
                to_delete.append(obj_id)  # 타임아웃된 객체

        for obj_id in to_delete:
            self.delete(obj_id) # 타임아웃된 객체 삭제

    def delete(self, obj_id):
        self.deleted_ids.add(obj_id)  # 삭제된 객체 ID를 집합에 추가
        del self.trackers[obj_id]

    def get_deleted_ids(self):
        deleted_ids = list(self.deleted_ids)
        self.deleted_ids.clear()
        return deleted_ids

    def update(self, obj_id, data):
        if obj_id in self.trackers.keys():
            current_time = rospy.Time.now()  # 현재 시간
            if current_time - self.trackers[obj_id]['last_update_time'] > rospy.Duration(self.timeout):
                self.delete(obj_id)  # 타임아웃된 객체 삭제
                return

            self.trackers[obj_id]['tracker'].update(data)
            self.trackers[obj_id]['last_update_time'] = rospy.Time.now()  # 업데이트된 시간 갱신
        else:
            self.initialize(obj_id, data)

    def predict(self):
        trajs = {}
        for obj_id in self.trackers.keys():
            trajs[obj_id] = self.trackers[obj_id]['tracker'].predict()

        if len(trajs) == 0:
            return None
        else:
            return trajs


class DynamicObstacleTrackerNode:
    def __init__(self):
        rospy.init_node("dynamic_obstacle_tracker_node", anonymous=True)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_info_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.object_pose_pub = rospy.Publisher('/Object_topic/tracked_object_pose_topic', TrackedObjectPoseList, queue_size=10)
        self.object_path_pub = rospy.Publisher('/Object_topic/tracked_object_path_topic', PredictedObjectPathList, queue_size=10)
        self.deleted_id_pub = rospy.Publisher('/Object_topic/deleted_object_id', Int32, queue_size=10)
        self.tracker = MultiDynamicObstacleTracker(dt=0.05, T=0.8, timeout=0.1)

        self.rate = rospy.Rate(30)
        self.is_odom = False
        self.odom_data = None
        self.is_object = False
        self.object_data = None
        self.previous_time = rospy.Time.now()

    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_data = msg

    def object_info_callback(self, msg):
        self.is_object=True
        self.object_data = msg

    def data_preprocessing(self, obstacle, delta_time):
        # 평면 속도 계산
        v = sqrt(pow(obstacle.velocity.x, 2) + pow(obstacle.velocity.y, 2))

        data = [obstacle.position.x, obstacle.position.y, radians(obstacle.heading), v]

        return data

    def publish_object_pose(self):
        pose_list = TrackedObjectPoseList()
        pose_list.header.stamp = rospy.Time.now()

        for obj_id in self.tracker.trackers.keys():
            # [x, y, v, a, theta, theta_rate]
            x = self.tracker.trackers[obj_id]['tracker'].X[-1]
            pose = TrackedObjectPose(unique_id=obj_id, pose=TrackedPoint(x=x[0], y=x[1], v=x[2], a=x[3], theta=x[4], theta_rate=x[5]))
            pose_list.pose_list.append(pose)

        self.object_pose_pub.publish(pose_list)

    def publish_object_path(self):
        trajs = self.tracker.predict()
        if trajs is None:
            return

        path_list = PredictedObjectPathList()
        path_list.header.stamp = rospy.Time.now()

        for obj_id in trajs.keys():
            path = PredictedObjectPath()
            path.unique_id = obj_id
            for point in trajs[obj_id]:
                # [x, y, v, a, theta, theta_rate]
                pose = TrackedPoint(x=point[0], y=point[1], v=point[2], a=point[3], theta=point[4], theta_rate=point[5])
                path.path.append(pose)

            path_list.path_list.append(path)

        self.object_path_pub.publish(path_list)

    def publish_deleted_ids(self):
        deleted_ids = self.tracker.get_deleted_ids()
        for deleted_id in deleted_ids:
            self.deleted_id_pub.publish(deleted_id)

    def run(self):
        while not rospy.is_shutdown():
            if self.is_object and self.is_odom:
                current_time = rospy.Time.now()
                delta_time = (current_time - self.previous_time).to_sec()

                x = self.odom_data.pose.pose.position.x
                y = self.odom_data.pose.pose.position.y

                for obstacle in self.object_data.npc_list:
                    obj_id = obstacle.unique_id

                    obstacle_x = obstacle.position.x
                    obstacle_y = obstacle.position.y

                    distance = sqrt(pow(x - obstacle_x, 2) + pow(y - obstacle_y, 2))
                    if distance > 30:
                        continue

                    data = self.data_preprocessing(obstacle, delta_time)

                    self.tracker.update(obj_id, data)

                self.previous_time = current_time

                self.tracker.clean()
                self.publish_deleted_ids()

                # self.publish_object_pose()
                self.publish_object_path()

            self.rate.sleep()

if __name__ == '__main__':
    try:
        tracker = DynamicObstacleTrackerNode()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
