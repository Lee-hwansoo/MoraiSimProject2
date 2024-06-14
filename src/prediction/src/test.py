#!/usr/bin/env python3
# coding: utf-8
import rospy
import numpy as np
from state_models_opt import CA, CTRA
from state_filters_opt import Extended_KalmanFilter, IMM_filter
from object_tracker_opt import DynamicObstacleTracker, MultiDynamicObstacleTracker

def test_CA():
    ca_model = CA(0.1)
    x = np.array([0, 0, 10, 1, 0.1])  # Initial state: [x, y, v, a, theta]

    print("Initial state (CA):", x)

    for i in range(3):
        x = ca_model.step(x)
        print(f"State after step {i + 1} (CA):", x)

        z = ca_model.H(x)
        print(f"Measurement after step {i + 1} (CA):", z)

        JA = ca_model.JA(x)
        print(f"Jacobian of state transition after step {i + 1} (CA):\n", JA)

        JH = ca_model.JH(x)
        print(f"Jacobian of measurement function after step {i + 1} (CA):\n", JH)

def test_CTRA():
    ctra_model = CTRA(0.1)
    x = np.array([0, 0, 10, 1, 0.1, 0.01])  # Initial state: [x, y, v, a, theta, theta_rate]

    print("Initial state (CTRA):", x)

    for i in range(3):
        x = ctra_model.step(x)
        print(f"State after step {i + 1} (CTRA):", x)

        z = ctra_model.H(x)
        print(f"Measurement after step {i + 1} (CTRA):", z)

        JA = ctra_model.JA(x)
        print(f"Jacobian of state transition after step {i + 1} (CTRA):\n", JA)

        JH = ctra_model.JH(x)
        print(f"Jacobian of measurement function after step {i + 1} (CTRA):\n", JH)

def test_ExtendedKalmanFilter():
    ca_model = CA(0.1)
    ekf = Extended_KalmanFilter(5, 4)

    Q = np.diag([0.1, 0.1, 0.1, 0.1, 0.1])
    R = np.diag([0.05, 0.05, 0.05, 0.05])

    ekf.F = ca_model.step
    ekf.H = ca_model.H
    ekf.JA = ca_model.JA
    ekf.JH = ca_model.JH
    ekf.Q = Q
    ekf.R = R

    ekf.x = np.array([0, 0, 10, 1, 0.1])
    print("Initial state (EKF): ", ekf.x)

    measurements = [
        np.array([0.999979, 0.100333, 10.1, 0.1]),
        np.array([2.000123, 0.201234, 10.2, 0.1]),
        np.array([3.001234, 0.302345, 10.3, 0.1])
    ]

    for z in measurements:
        ekf.prediction()
        print("State after prediction (EKF): ", ekf.x)
        ekf.correction(z)
        print("State after correction (EKF): ", ekf.x, "\n")

    T = 1.0
    predicted_states = ekf.pred(T)
    print("\nPredicted states:\n", predicted_states)

def test_IMMFilter():
    ca_model = CA(0.1)
    ctra_model = CTRA(0.1)

    ekf_ca = Extended_KalmanFilter(5, 4)
    ekf_ctra = Extended_KalmanFilter(6, 4)

    Q_ca = np.diag([0.1, 0.1, 0.1, 0.1, 0.1])
    R_ca = np.diag([0.05, 0.05, 0.05, 0.05])
    Q_ctra = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    R_ctra = np.diag([0.05, 0.05, 0.05, 0.05])


    ekf_ca.F = ca_model.step
    ekf_ca.H = ca_model.H
    ekf_ca.JA = ca_model.JA
    ekf_ca.JH = ca_model.JH
    ekf_ca.Q = Q_ca
    ekf_ca.R = R_ca
    ekf_ctra.F = ctra_model.step
    ekf_ctra.H = ctra_model.H
    ekf_ctra.JA = ctra_model.JA
    ekf_ctra.JH = ctra_model.JH
    ekf_ctra.Q = Q_ctra
    ekf_ctra.R = R_ctra

    ekf_ca.x = np.array([0, 0, 10, 1, 0.1])
    ekf_ctra.x = np.array([0, 0, 10, 1, 0.1, 0.01])

    filters = [ekf_ca, ekf_ctra]
    mu = np.array([0.5, 0.5])
    M = np.array([[0.97, 0.03], [0.03, 0.97]])
    imm = IMM_filter(filters, mu, M)
    print("Initial state (IMM): ", imm.x)

    measurements = [
        np.array([0.999979, 0.100333, 10.1, 0.1]),
        np.array([2.000123, 0.201234, 10.2, 0.1]),
        np.array([3.001234, 0.302345, 10.3, 0.1])
    ]

    for z in measurements:
        imm.prediction()
        print("State after prediction (IMM): ", imm.x)
        imm.merging(z)
        print("State after merging (IMM): ", imm.x, "\n")

    T = 1.0
    predicted_states = imm.pred(T)
    print("\nPredicted states (IMM):\n", predicted_states)

def test_multi_dynamic_obstacle_tracker():
    # ROS 노드 초기화
    rospy.init_node('test_dynamic_obstacle_tracker', anonymous=True)

    # MultiDynamicObstacleTracker 초기화
    multi_tracker = MultiDynamicObstacleTracker(dt=0.1, T=1.0, timeout=2.0)

    # 객체 ID 및 초기 데이터 설정
    obj_ids = [1, 2, 3]
    init_data = [
        [1.0, 2.0, 0.5, 3.0],  # [x, y, h, v]
        [2.0, 3.0, 0.6, 2.5],  # [x, y, h, v]
        [3.0, 4.0, 0.7, 4.0]   # [x, y, h, v]
    ]

    # 객체 초기화
    for obj_id, data in zip(obj_ids, init_data):
        multi_tracker.initialize(obj_id, data)
        rospy.loginfo(f"Initialized object ID: {obj_id}")

    # 객체 업데이트 데이터 설정
    update_data = [
        [1.1, 2.1, 0.55, 3.1],  # [x, y, h, v]
        [2.1, 3.1, 0.65, 2.6],  # [x, y, h, v]
        [3.1, 4.1, 0.75, 4.1]   # [x, y, h, v]
    ]

    # 객체 업데이트
    for obj_id, data in zip(obj_ids, update_data):
        multi_tracker.update(obj_id, data)
        rospy.loginfo(f"Updated object ID: {obj_id}")

    # 객체 예측
    trajs = multi_tracker.predict()
    for obj_id in obj_ids:
        if trajs and obj_id in trajs:
            rospy.loginfo(f"Predicted trajectory for object ID {obj_id}: \n{trajs[obj_id]}")

    # 타임아웃 테스트를 위해 3초 대기
    rospy.loginfo("Waiting for 3 seconds to test clean...")
    rospy.sleep(3)

    # 객체 정리
    multi_tracker.clean()
    deleted_ids = multi_tracker.get_deleted_ids()
    for obj_id in deleted_ids:
        rospy.loginfo(f"Deleted object ID: {obj_id}")

if __name__ == "__main__":
    # print("Testing CA Model...")
    # test_CA()

    # print("\nTesting CTRA Model...")
    # test_CTRA()

    # print("\nTesting Extended_KalmanFilter...")
    # test_ExtendedKalmanFilter()

    # print("\nTesting IMM_filter...")
    # test_IMMFilter()

    # print("\nTesting MultiDynamicObstacleTracker...")
    # test_multi_dynamic_obstacle_tracker()
    pass
