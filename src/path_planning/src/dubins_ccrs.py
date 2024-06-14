#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

def twopify(alpha):
    return alpha - np.pi * 2 * np.floor(alpha / (np.pi * 2))

def pify(alpha):
    v = np.fmod(alpha, 2*np.pi)
    if (v < - np.pi):
        v += 2 * np.pi
    else:
        v -= 2 * np.pi
    return v

class CCRSPath(object):
    def __init__(self, t=0, p=1e10, q=0, type=None):
        self.t = t
        self.p = p
        self.q = q
        self.length_ = [t, p, q]
        self.type = type
        self.controls = None

    def length(self):
        return self.t + self.p + self.q

class CCRSControl(object):
    def __init__(self):
        self.delta_s = 0.0
        self.kappa = 0.0

class CCRS(object):
    def __init__(self):
        self.constant = {
            "ccrs_zero": -1e-9,
            "ccrs_eps": 1e-6
        }

    def ccrsLSL(self, d, alpha, beta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = 2 + d * d - 2 * (ca * cb + sa * sb - d * (sa - sb))
        if (tmp >= self.constant["ccrs_zero"]):
            theta = np.arctan2(cb - ca, d + sa - sb)
            t = twopify(-alpha + theta)
            p = np.sqrt(np.amax([tmp, 0]))
            q = twopify(beta - theta)

            return CCRSPath(t, p, q, ["L", "S", "L"])

        else:
            return None

    def ccrsRSR(self, d, alpha, beta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sb - sa))
        if (tmp >= self.constant["ccrs_zero"]):
            theta = np.arctan2(ca - cb, d - sa + sb)
            t = twopify(alpha - theta)
            p = np.sqrt(np.amax([tmp, 0]))
            q = twopify(-beta + theta)
            return CCRSPath(t, p, q, ["R", "S", "R"])
        else:
            return None

    def ccrsRSL(self, d, alpha, beta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb))
        if (tmp >= self.constant["ccrs_zero"]):
            p = np.sqrt(np.amax([tmp, 0]))
            theta = np.arctan2(ca + cb, d - sa - sb) - np.arctan2(2., p)
            t = twopify(alpha - theta)
            q = twopify(beta - theta)
            return CCRSPath(t, p, q, ["R", "S", "L"])
        else:
            return None

    def ccrsLSR(self, d, alpha, beta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = -2. + d * d + 2. * (ca * cb + sa * sb + d * (sa + sb))
        if (tmp >= self.constant["ccrs_zero"]):
            p = np.sqrt(np.amax([tmp, 0]))
            theta = np.arctan2(-ca - cb, d + sa + sb) - np.arctan2(-2., p)
            t = twopify(-alpha + theta)
            q = twopify(-beta + theta)
            return CCRSPath(t, p, q, ["L", "S", "R"])
        else:
            return None

    def ccrsRLR(self, d, alpha, beta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb)))
        if (np.abs(tmp) < 1.):
            p = 2 * np.pi - np.arccos(tmp)
            theta = np.arctan2(ca - cb, d - sa + sb)
            t = twopify(alpha - theta + .5 * p)
            q = twopify(alpha - beta - t + p)
            return CCRSPath(t, p, q, ["R", "L", "R"])
        else:
            return None

    def ccrsLRL(self, d, alpha, beta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb - d * (sa - sb)))
        if (np.abs(tmp) < 1.):
            p = 2 * np.pi - np.arccos(tmp)
            theta = np.arctan2(-ca + cb, d + sa - sb)
            t = twopify(-alpha + theta + .5 * p)
            q = twopify(beta - alpha - t + p)
            return CCRSPath(t, p, q, ["L", "R", "L"])
        else:
            return None

    def get_best_ccrs_path(self, d, alpha, beta):
        ccrs_functions = [
            self.ccrsLSL, self.ccrsRSR, self.ccrsRSL,
            self.ccrsLSR, self.ccrsRLR, self.ccrsLRL
        ]

        min_length = 1e10
        path = None
        for ccrs_function in ccrs_functions:
            tmp_path = ccrs_function(d, alpha, beta)
            if tmp_path is not None:
                if (tmp_path.length() < min_length):
                    min_length = tmp_path.length()
                    path = tmp_path
        return path

    def plan(self, state1, state2, kappa):
        dx = state2[0] - state1[0]
        dy = state2[1] - state1[1]
        th = np.arctan2(dy, dx)

        d = np.hypot(dx, dy) * kappa
        alpha = twopify(state1[2] - th)
        beta = twopify(state2[2] - th)

        ccrs_path = self.get_best_ccrs_path(d, alpha, beta)
        controls = self.ccrs_path_to_controls(ccrs_path, kappa)
        cartesian_path = self.controls_to_cartesian_path(controls, state1)

        return cartesian_path, controls, ccrs_path

    def ccrs_path_to_controls(self, ccrs_path, kappa):
        controls = []
        kappa_inv = 1.0/kappa

        if ccrs_path is not None:
            for i in range(3):
                control = CCRSControl()
                type = ccrs_path.type[i]
                length = ccrs_path.length_[i]
                delta_s = kappa_inv * length

                control.delta_s = delta_s
                if (type == "L"):
                    control.kappa = kappa

                if (type == "S"):
                    control.kappa = 0

                if (type == "R"):
                    control.kappa = -kappa

                controls.append(control)
            return controls

        else:
            return None

    def controls_to_cartesian_path(self, controls, state1, discretization=0.7):
        if controls is None:
            return None

        x, y, yaw = state1
        xs, ys, yaws = [], [], []

        for control in controls:
            delta_s = control.delta_s
            abs_delta_s = np.abs(delta_s)
            kappa = control.kappa

            s_seg = 0
            integration_step = 0.0
            for j in range(int(np.ceil(abs_delta_s / discretization))):
                s_seg += discretization
                if (s_seg > abs_delta_s):
                    integration_step = discretization - (s_seg - abs_delta_s)
                    s_seg = abs_delta_s
                else:
                    integration_step = discretization

                if np.abs(kappa) > 0.0001:
                    x += 1/kappa * (-np.sin(yaw) + np.sin(yaw + integration_step * kappa))
                    y += 1/kappa * (np.cos(yaw) - np.cos(yaw + integration_step * kappa))
                    yaw = pify(yaw + integration_step * kappa)
                else:
                    x += integration_step * np.cos(yaw)
                    y += integration_step * np.sin(yaw)

                xs.append(x)
                ys.append(y)
                yaws.append(yaw)

        return xs, ys, yaws
