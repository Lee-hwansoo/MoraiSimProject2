#!/usr/bin/env python3
# coding: utf-8
import numpy as np
import copy
from scipy.stats import norm, multivariate_normal

import matplotlib.pyplot as plt

class KalmanFilter():
    def __init__(self, x_dim, z_dim):
        """
        Kalman 필터 클래스

        Args:
            x_dim (int): 상태 벡터의 차원
            z_dim (int): 측정 벡터의 차원
        """
        self.Q = np.eye(x_dim)  # 시스템 노이즈 공분산 행렬
        self.R = np.eye(z_dim)  # 측정 노이즈 공분산 행렬
        self.B = None           # 제어 변수 행렬
        self.P = np.eye(x_dim)  # 상태 공분산 행렬
        self.A = np.eye(x_dim)  # 상태 전이 행렬
        self.H = np.zeros((z_dim, x_dim))   # 측정 함수 행렬

        # 초기 상태 및 측정 벡터
        self.x = np.zeros((x_dim, 1))
        self.y = np.zeros((z_dim, 1))

        # 칼만 이득 및 측정 노이즈 공분산 행렬
        self.K = np.zeros((x_dim, z_dim))
        self.S = np.zeros((z_dim, z_dim))

        # 단위 행렬
        self._I = np.eye(x_dim)

        # 사전 예측 상태 및 공분산
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()

        # 사후 예측 상태 및 공분산
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

        # 측정 오차 공분산 행렬의 역행렬 계산 함수
        self.SI = np.zeros((z_dim, z_dim))
        self.inv = np.linalg.inv

    def predict(self, u=None, B=None, A=None, Q=None):
        """
        예측 단계 수행

        Args:
            u (np.ndarray, optional): 제어 입력 벡터
            B (np.ndarray, optional): 제어 변수 행렬
            A (np.ndarray, optional): 상태 전이 행렬
            Q (np.ndarray, optional): 시스템 노이즈 공분산 행렬
        """
        # 제어 변수와 상태 전이 행렬의 기본값 처리
        if B is None:
            B = self.B
        if A is None:
            A = self.A
        if Q is None:
            Q = self.Q

        # 예측된 상태 벡터 계산
        if B is not None and u is not None:
            self.x = np.dot(A, self.x) + np.dot(B, u)
        else:
            self.x = np.dot(A, self.x)

        # 예측된 상태 공분산 계산
        self.P = np.dot(np.dot(A, self.P), A.T) + Q

        # 사전 예측 상태 및 공분산 업데이트
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()

    def correction(self, z, R=None, H=None):
        """
        수정 단계를 수행

        Args:
            z (np.ndarray): 측정 벡터
            R (np.ndarray, optional): 측정 노이즈 공분산 행렬
            H (np.ndarray, optional): 측정 함수 행렬
        """
        # 측정 노이즈 공분산 행렬과 측정 함수 행렬의 기본값 처리
        if R is None:
            R = self.R
        if H is None:
            H = self.H

        # 측정 오차 계산
        self.y = z - np.dot(H, self.x)

        # 공분산 행렬과 측정 함수 행렬의 곱 계산
        PHT = np.dot(self.P, H.T)

        # 측정 노이즈 공분산 계산
        self.S = np.dot(H, PHT) + R
        self.SI = self.inv(self.S)

        # 칼만 이득 계산
        self.K = np.dot(PHT, self.SI)

        # 상태 벡터 수정
        self.x = self.x + np.dot(self.K, self.y)

        # 칼만 이득과 측정 함수 행렬의 차 계산
        I_KH = self._I - np.dot(self.K, H)

        self.P = np.dot(np.dot(I_KH, self.P), I_KH.T) +\
            np.dot(np.dot(self.K, R), self.K.T)

        # 사후 예측 상태 및 공분산 계산
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()


class Extended_KalmanFilter():
    def __init__(self, x_dim, z_dim):
        """
        Extended Kalman Filter 클래스

        Args:
            x_dim (int): 상태 벡터의 차원
            z_dim (int): 측정 벡터의 차원
        """
        self.Q = np.eye(x_dim)  # 프로세스 노이즈 공분산 행렬
        self.R = np.eye(z_dim)  # 측정 노이즈 공분산 행렬
        self.B = None           # 제어 변수 행렬
        self.P = np.eye(x_dim)  # 초기 오차 공분산 행렬
        self.JA = None          # 상태 전이 함수의 야코비안
        self.JH = None          # 측정 함수의 야코비안
        self.F = (lambda x: x)  # 상태 전이 함수
        self.H = (lambda x: np.zeros(z_dim, 1)) # 측정 함수

        # 초기 상태 및 측정 벡터
        self.x = np.zeros((x_dim, ))
        self.y = np.zeros((z_dim, ))

        # 칼만 이득 및 측정 노이즈 공분산 행렬
        self.K = np.zeros((x_dim, z_dim))
        self.S = np.zeros((z_dim, z_dim))

        # 상태 및 측정 벡터 차원 저장
        self.x_dim = x_dim
        self.z_dim = z_dim

        # 단위 행렬
        self._I = np.eye(x_dim)

        # 사전 예측 상태 및 공분산
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()

        # 사후 예측 상태 및 공분산
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

        # 측정 오차 공분산 행렬의 역행렬 계산 함수
        self.SI = np.zeros((z_dim, z_dim))
        self.inv = np.linalg.inv

        # 추정된 측정 노이즈의 확률밀도함수 값
        self.likelihood = 1.0

    def predict(self, u=None, JA=None, F=None, Q=None):
        """
        예측 단계를 수행

        Args:
            u (np.ndarray, optional): 조작 입력 벡터
            JA (function, optional): 상태 전이 함수의 야코비안
            F (function, optional): 상태 전이 함수
            Q (np.ndarray, optional): 프로세스 노이즈 공분산 행렬
        """
        # 프로세스 노이즈 공분산 행렬 및 상태 전이 함수의 기본값 처리
        if Q is None:
            Q = self.Q
        # x = Fx + Bu
        if JA is None:
            if self.JA is None:
                JA_ = np.eye(self.x_dim)
            else:
                JA_ = self.JA(self.x)
        else:
            JA_ = JA(self.x)
        if F is None:
            F = self.F

        # 예측 상태 벡터 업데이트
        self.x = F(self.x)

        # 예측 공분산 행렬 업데이트
        self.P = np.dot(np.dot(JA_, self.P), JA_.T) + Q

        # 사전 예측 상태 및 공분산 행렬 저장
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()

    def pred(self, T, dt=0.1):
        """
        시간에 따른 예측 수행

        Args:
            T (float): 예측할 총 시간
            dt (float): 각 예측 단계의 시간 간격
        Returns:
            np.ndarray: 예측된 상태 벡터의 배열
        """
        x = self.x.copy()
        X = [x]

        for i in range(int(T/dt)):
            x = self.F(x)

            X.append(x)

        return np.array(X)

    def correction(self, z, JH=None, H=None, R=None):
        """
        수정 단계를 수행

        Args:
            z (np.ndarray): 측정 벡터
            JH (function, optional): 측정 함수의 야코비안
            H (function, optional): 측정 함수
            R (np.ndarray, optional): 측정 노이즈 공분산 행렬
        """
        # 초기화
        if JH is None:
            if self.JH is None:
                JH_ = np.zeros((self.x_dim, self.z_dim))
            else:
                JH_ = self.JH(self.x)
        else:
            JH_ = JH(self.x)
        if H is None:
            H = self.H
        if R is None:
            R = self.R

        # 예측된 측정 벡터
        z_pred = H(self.x)
        # 예측 오차
        self.y = z - z_pred

        # 공분산 계산
        PHT = np.dot(self.P, JH_.T)

        # 칼만 이득 및 측정 노이즈 공분산 행렬 측정 오차 공분산 행렬
        self.S = np.dot(JH_, PHT) + R
        self.SI = self.inv(self.S)
        self.K = np.dot(PHT, self.SI)

        # 상태 벡터 및 공분산 업데이트
        self.x = self.x + np.dot(self.K, self.y)
        I_KH = self._I - np.dot(self.K, JH_)
        self.P = np.dot(np.dot(I_KH, self.P), I_KH.T) + \
            np.dot(np.dot(self.K, R), self.K.T)

        # 사후 예측 상태 및 공분산 행렬 저장
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

        # 추정된 측정 노이즈의 확률밀도함수 값 계산
        self.likelihood = multivariate_normal.pdf(
            self.y, np.zeros_like(self.y), self.S)


class IMM_filter():
    def __init__(self, filters, mu, M):
        """
        IMM (Interacting Multiple Model) 필터 클래스

        Args:
            filters (list): 사용할 필터 리스트
            mu (np.ndarray): 모드 확률 벡터
            M (np.ndarray): 전이 확률 행렬
        """
        self.filters = filters
        self.mu = mu
        self.M = M
        self.N = len(filters)

        # 각 필터의 상태 벡터의 차원 중 가장 큰 값을 기준으로 전체 상태 벡터를 초기화합니다.
        n_cand = [len(f.x) for f in filters]
        target_filter = np.argmax(n_cand)
        self.x = np.zeros(filters[target_filter].x.shape)
        self.P = np.zeros(filters[target_filter].P.shape)

        # 모드 전환 확률을 초기화합니다.
        self.omega = 1/self.N*np.ones((self.N, self.N))
        self.mM = np.dot(self.mu, self.M)
        self.likelihood = np.zeros(self.N)

        # 상태 벡터의 사전 예측 및 사후 예측을 저장
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

    def mixing(self):
        """
        다양한 모드의 예측 결과를 혼합
        """
        # 혼합된 상태 벡터와 공분산 행렬을 저장할 리스트를 초기화합니다.
        self.xs, self.Ps = [], []

        # 각 필터와 해당 모드의 가중치를 이용하여 예측 결과를 혼합합니다.
        for i, (f, w) in enumerate(zip(self.filters, self.omega.T)):
            # 혼합된 상태 벡터를 계산합니다.
            x = np.zeros(f.x.shape)
            for kf, wj in zip(self.filters, w):
                x[0:5] += kf.x[0:5] * wj
                if len(kf.x) > 5 and len(x) > 5:
                    x[5] = kf.x[5]
            self.xs.append(x)

            # 혼합된 공분산 행렬을 계산합니다.
            P = np.zeros(f.P.shape)
            for kf, wj in zip(self.filters, w):
                y = kf.x[0:5] - x[0:5]
                P[0:5, 0:5] += wj * (np.outer(y, y) + kf.P[0:5, 0:5])
                if len(kf.x) > 5 and len(P) > 5:
                    P[5, 5] = kf.P[5, 5]
            self.Ps.append(P)

    def prediction(self, mixing=True):
        """
        모드 전환을 고려하여 예측을 수행

        Args:
            mixing (bool): 상태 벡터와 공분산 행렬을 혼합할지 여부를 결정합니다.
        """
        # 상태 벡터 및 공분산 행렬의 혼합을 선택적으로 수행합니다.
        if mixing:
            self.mixing()

        for i, f in enumerate(self.filters):
            if mixing:
                f.x = self.xs[i].copy()
                f.P = self.Ps[i].copy()

            f.predict()


    def merging(self, z):
        """
        관측값을 사용하여 필터를 병합하고 모드 전환을 업데이트

        Args:
            z (np.ndarray): 관측 벡터, [x, y, v, theta]
        """
        # 각 필터를 관측값을 사용하여 수정하고 각 필터의 likelihood를 업데이트합니다.
        for i, f in enumerate(self.filters):
            f.correction(z)
            self.likelihood[i] = f.likelihood

        # 새로운 모드 확률을 업데이트합니다.
        self.mu = self.mM * self.likelihood
        self.mu /= np.sum(self.mu)
        self.mM = np.dot(self.mu, self.M)

        for i in range(self.N):
            for j in range(self.N):
                self.omega[i, j] = (self.M[i, j]*self.mu[i]) / self.mM[j]

        # 병합된 상태 벡터를 계산합니다.
        self.x.fill(0)
        for f, mu in zip(self.filters, self.mu):
            self.x[0:5] += f.x[0:5] * mu
            if len(f.x) > 5:
                self.x[5] = f.x[5]

        # 병합된 공분산 행렬을 계산합니다.
        self.P.fill(0)
        for f, mu in zip(self.filters, self.mu):
            y = f.x[0:5] - self.x[0:5]
            self.P[0:5, 0:5] += mu * (np.outer(y, y) + f.P[0:5, 0:5])
            if len(f.x) > 5:
                self.P[5, 5] = (f.x[5]-self.x[5])**2+f.P[5, 5]

    def predict(self, T, dt=0.1):
        """
        시간에 따른 예측 수행

        Args:
            T (float): 예측할 총 시간
            dt (float): 각 예측 단계의 시간 간격
        Returns:
            np.ndarray: 예측된 상태 벡터의 배열
        """
        # 초기 상태 벡터를 결과 리스트에 추가
        X = [self.x]

        # 모드 전환 확률을 복사하고 필터를 복사합니다.
        omega = self.omega.copy()
        mu = self.mu.copy()
        mM = self.mM.copy()
        likelihood = np.array([1.0, 1.0])
        filters = [copy.deepcopy(self.filters[i])
                   for i in range(len(self.filters))]

        # 주어진 시간 동안 예측을 반복합니다.
        for i in range(int(T/dt)):
            xs, Ps = [], []

            # 각 모델과 해당 모드의 가중치를 사용하여 예측 결과를 혼합합니다.
            for j, (f, w) in enumerate(zip(filters, omega.T)):
                x = np.zeros(f.x.shape)
                for kf, wj in zip(filters, w):
                    x[0:5] += kf.x[0:5] * wj
                    if len(kf.x) > 5 and len(x) > 5:
                        x[5] = kf.x[5]
                xs.append(x)

                P = np.zeros(f.P.shape)
                for kf, wj in zip(filters, w):
                    y = kf.x[0:5] - x[0:5]
                    P[0:5, 0:5] += wj * (np.outer(y, y) + kf.P[0:5, 0:5])
                    if len(kf.x) > 5 and len(P) > 5:
                        P[5, 5] = kf.P[5, 5]
                Ps.append(P)

            # 각 필터에 대해 예측을 수행합니다.
            for j, f in enumerate(filters):
                f.x = xs[j].copy()
                f.P = Ps[j].copy()
                f.predict()

                likelihood[j] = (f.P[0, 0] + f.P[1, 1])

            y = np.zeros(self.x.shape)
            mu = mM * likelihood
            mu /= np.sum(mu)
            mM = np.dot(mu, self.M)

            # 모드 확률 업데이트
            for i in range(self.N):
                for j in range(self.N):
                    omega[i, j] = (self.M[i, j]*mu[i]) / mM[j]

            # 예측된 상태 벡터를 결과 리스트에 추가합니다.
            for f, m_ in zip(filters, mu):
                y[0:5] += f.x[0:5] * m_
                if len(f.x) > 5 and len(y) > 5:
                    y[5] = f.x[5]
            X.append(y)

        return np.array(X)
