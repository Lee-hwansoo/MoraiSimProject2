#!/usr/bin/env python3
# coding: utf-8
import numpy as np

class CA():
    def __init__(self, dt=0.1):
        """
        CA 모델 클래스의 초기화 메서드

        Args:
            dt (float): 시간 간격 (기본값: 0.1)
        """
        self.dt = dt

    def step(self, x):
        """
        CA 모델의 한 단계 예측 수행

        Args:
            x (np.ndarray): 현재 상태 벡터 [x, y, v, a, theta]

        Returns:
            np.ndarray: 다음 시간 단계의 상태 벡터
        """
        dt = self.dt
        cos_theta = np.cos(x[4])
        sin_theta = np.sin(x[4])
        x_new = np.array([
            x[0] + (x[2] + 0.5 * x[3] * dt) * cos_theta * dt,
            x[1] + (x[2] + 0.5 * x[3] * dt) * sin_theta * dt,
            x[2] + x[3] * dt,
            x[3],
            x[4]
        ])
        return x_new

    def H(self, x):
        """
        관측 모델

        Args:
            x (np.ndarray): 상태 벡터 [x, y, v, a, theta]

        Returns:
            np.ndarray: 관측 벡터
        """
        return x[[0, 1, 2, 4]]

    def JA(self, x, dt=0.1):
        """
        예측 모델의 야코비안

        Args:
            x (np.ndarray): 상태 벡터 [x, y, v, a, theta]
            dt (float): 시간 간격

        Returns:
            np.ndarray: 야코비안 행렬
        """
        if dt is None:
            dt = self.dt

        px, py, v, a, yaw = x
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        JA_ = np.array([
            [1, 0, cos_yaw * dt, 0.5 * cos_yaw * dt**2, -(v + 0.5 * a * dt) * sin_yaw * dt],
            [0, 1, sin_yaw * dt, 0.5 * sin_yaw * dt**2, (v + 0.5 * a * dt) * cos_yaw * dt],
            [0, 0, 1, dt, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])
        return JA_

    def JH(self, x, dt=0.1):
        """
        관측 모델의 야코비안

        Args:
            x (np.ndarray): 상태 벡터 [x, y, v, a, theta]
            dt (float): 시간 간격

        Returns:
            np.ndarray: 야코비안 행렬
        """

        JH_ = np.array([[1, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0],
                        [0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 1]])

        return JH_


class CTRA():
    def __init__(self, dt=0.1):
        """
        CTRA 모델 클래스의 초기화 메서드

        Args:
            dt (float): 시간 간격 (기본값: 0.1)
        """
        self.dt = dt

    def step(self, x):
        """
        CTRA 모델의 한 단계 예측 수행

        Args:
            x (np.ndarray): 현재 상태 벡터 [x, y, v, a, theta, theta_rate]

        Returns:
            np.ndarray: 다음 시간 단계의 상태 벡터
        """
        dt = self.dt
        cos_theta = np.cos(x[4])
        sin_theta = np.sin(x[4])
        if np.abs(x[5]) > 0.1:
            theta_dt = x[4] + x[5] * dt
            cos_theta_dt = np.cos(theta_dt)
            sin_theta_dt = np.sin(theta_dt)

            x_new = np.array([
                x[0] + x[2] / x[5] * (sin_theta_dt - sin_theta) + x[2] / (x[5]**2) * (cos_theta_dt + dt * x[5] * sin_theta_dt - cos_theta),
                x[1] + x[2] / x[5] * (-cos_theta_dt + cos_theta) + x[2] / (x[5]**2) * (sin_theta_dt - dt * x[5] * cos_theta_dt - sin_theta),
                x[2] + x[3] * dt,
                x[3],
                x[4] + x[5] * dt,
                x[5]
            ])

        else:
            x_new = np.array([
                x[0] + x[2] * cos_theta * dt,
                x[1] + x[2] * sin_theta * dt,
                x[2] + x[3] * dt,
                x[3],
                x[4],
                x[5]
            ])

        return x_new

    def H(self, x):
        """
        관측 모델

        Args:
            x (np.ndarray): 상태 벡터 [x, y, v, a, theta, theta_rate]

        Returns:
            np.ndarray: 관측 벡터
        """
        return x[[0, 1, 2, 4]]

    def JA(self, x, dt=0.1):
        """
        예측 모델의 야코비안

        Args:
            x (np.ndarray): 상태 벡터 [x, y, v, a, theta, theta_rate]
            dt (float): 시간 간격

        Returns:
            np.ndarray: 야코비안 행렬
        """
        if dt is None:
            dt = self.dt

        px, py, v, a, yaw, r = x

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        # upper
        if np.abs(r) > 0.1:
            cos_yaw_dt = np.cos(yaw + r * dt)
            sin_yaw_dt = np.sin(yaw + r * dt)

            JA_ = np.array([
                [1, 0, (sin_yaw_dt - sin_yaw) / r, (-cos_yaw + cos_yaw_dt + r * dt * sin_yaw_dt) / r**2, ((r * v + a * r * dt) * cos_yaw_dt - a * sin_yaw_dt - v * r * cos_yaw + a * sin_yaw) / r**2, -2 / r**3 * ((r * v + a * r * dt) * sin_yaw_dt + a * cos_yaw_dt - v * r * sin_yaw - a * cos_yaw) + ((v + a * dt) * sin_yaw_dt + dt * (r * v + a * r * dt) * cos_yaw_dt - dt * a * sin_yaw_dt - v * sin_yaw) / r**2],
                [0, 1, (-cos_yaw_dt + cos_yaw) / r, (-sin_yaw + sin_yaw_dt - r * dt * cos_yaw_dt) / r**2, ((r * v + a * r * dt) * sin_yaw_dt + a * cos_yaw_dt - v * r * sin_yaw - a * cos_yaw) / r**2, -2 / r**3 * ((-r * v - a * r * dt) * cos_yaw_dt + a * sin_yaw_dt + v * r * cos_yaw - a * sin_yaw) + ((-v - a * dt) * cos_yaw_dt + dt * (r * v + a * r * dt) * sin_yaw_dt + a * dt * cos_yaw_dt + v * cos_yaw) / r**2],
                [0, 0, 1, dt, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, dt],
                [0, 0, 0, 0, 0, 1]
            ])
        else:
            JA_ = np.array([
                [1, 0, cos_yaw * dt, 0.5 * cos_yaw * dt**2, -(v + 0.5 * a * dt) * sin_yaw * dt, 0],
                [0, 1, sin_yaw * dt, 0.5 * sin_yaw * dt**2, (v + 0.5 * a * dt) * cos_yaw * dt, 0],
                [0, 0, 1, dt, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, dt],
                [0, 0, 0, 0, 0, 1]
            ])

        return JA_

    def JH(self, x, dt=0.1):
        """
        관측 모델의 야코비안

        Args:
            x (np.ndarray): 상태 벡터 [x, y, v, a, theta, theta_rate]
            dt (float): 시간 간격

        Returns:
            np.ndarray: 야코비안 행렬
        """
        if dt is None:
            dt = self.dt

        px, py, v, a, yaw, r = x

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        # upper
        if np.abs(r) > 0.1:
            cos_yaw_dt = np.cos(yaw + r * dt)
            sin_yaw_dt = np.sin(yaw + r * dt)

            JH_ = np.array([
                [1, 0, (sin_yaw_dt - sin_yaw) / r, (-cos_yaw + cos_yaw_dt + r * dt * sin_yaw_dt) / r**2, ((r * v + a * r * dt) * cos_yaw_dt - a * sin_yaw_dt - v * r * cos_yaw + a * sin_yaw) / r**2, -2 / r**3 * ((r * v + a * r * dt) * sin_yaw_dt + a * cos_yaw_dt - v * r * sin_yaw - a * cos_yaw) + ((v + a * dt) * sin_yaw_dt + dt * (r * v + a * r * dt) * cos_yaw_dt - dt * a * sin_yaw_dt - v * sin_yaw) / r**2],
                [0, 1, (-cos_yaw_dt + cos_yaw) / r, (-sin_yaw + sin_yaw_dt - r * dt * cos_yaw_dt) / r**2, ((r * v + a * r * dt) * sin_yaw_dt + a * cos_yaw_dt - v * r * sin_yaw - a * cos_yaw) / r**2, -2 / r**3 * ((-r * v - a * r * dt) * cos_yaw_dt + a * sin_yaw_dt + v * r * cos_yaw - a * sin_yaw) + ((-v - a * dt) * cos_yaw_dt + dt * (r * v + a * r * dt) * sin_yaw_dt + a * dt * cos_yaw_dt + v * cos_yaw) / r**2],
                [0, 0, 1, dt, 0, 0],
                [0, 0, 0, 0, 1, dt]
            ])

        else:
            JH_ = np.array([
                [1, 0, cos_yaw * dt, 0.5 * cos_yaw * dt ** 2, -(v + 0.5 * a * dt) * sin_yaw * dt, 0],
                [0, 1, sin_yaw * dt, 0.5 * sin_yaw * dt ** 2, (v + 0.5 * a * dt) * cos_yaw * dt, 0],
                [0, 0, 1, dt, 0, 0],
                [0, 0, 0, 0, 1, dt]
            ])

        return JH_

    def pred(self, x, t_pred):
        """
        주어진 시간만큼 예측하여 상태 벡터 목록 반환

        Args:
            x (np.ndarray): 초기 상태 벡터 [x, y, v, a, theta, theta_rate]
            t_pred (float): 예측할 시간

        Returns:
            np.ndarray: 예측된 상태 벡터들로 이루어진 목록
        """
        self.x = x

        x_list = [self.x]
        for t in range(int(t_pred/self.dt)):
            x_list.append(self.step(self.x))

        return np.array(x_list)
