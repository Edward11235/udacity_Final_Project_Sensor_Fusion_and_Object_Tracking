# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        # by hint, I find load them
        self.dim_state = params.dim_state
        self.dt = params.dt
        self.q = params.q

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        n = self.dim_state
        F = np.identity(self.dim_state)
        F[0, 3] = self.dt
        F[1, 4] = self.dt
        F[2, 5] = self.dt
        F = np.matrix(F)
        return F

        ############
        # END student code
        ############

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        Q = np.zeros((self.dim_state, self.dim_state))
        order_1 = self.dt * self.q
        order_2 = 1/2 * self.dt**2 * self.q
        order_3 = 1/3 * self.dt**3 * self.q
        Q[0, 0] = order_3
        Q[1, 1] = order_3
        Q[2, 2] = order_3
        Q[0, 3] = order_2
        Q[1, 4] = order_2
        Q[2, 5] = order_2

        Q[3, 0] = order_2
        Q[4, 1] = order_2
        Q[5, 2] = order_2
        Q[3, 3] = order_1
        Q[4, 4] = order_1
        Q[5, 5] = order_1

        return Q
        ############
        # END student code
        ############

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        F = self.F()
        Q = self.Q()
        x = F * track.x
        track.set_x(x)
        P = F*track.P*F.transpose() + Q
        track.set_P(P)
        ############
        # END student code
        ############

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        gamma = self.gamma(track, meas)
        H = meas.sensor.get_H(track.x)
        S = self.S(track, meas, H)
        K = track.P * H.transpose() * np.linalg.inv(S)
        new_x = track.x + K*gamma
        new_P = (np.identity(self.dim_state) - K*H)*track.P
        track.set_x(new_x)
        track.update_attributes(meas)
        track.set_P(new_P)
        track.set_t(meas.t)
        ############
        # END student code
        ############
        track.update_attributes(meas)

    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        gamma = meas.z - meas.sensor.get_hx(track.x)
        return gamma

        ############
        # END student code
        ############

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############
        S = H*track.P*H.transpose() + meas.R
        return S

        ############
        # END student code
        ############ 