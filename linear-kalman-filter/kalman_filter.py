"""
Based on the work of Marko Cotra.

References: 
* https://gist.github.com/cotramarko
* https://towardsdatascience.com/wtf-is-sensor-fusion-part-2-the-good-old-kalman-filter-3642f321440

"""
import numpy as np


class LinearKalmanFilter():
    def __init__(self, A, H, Q, R, x_0, P_0):
        # Model parameters
        self.A = A
        self.H = H
        self.Q = Q
        self.R = R

        # Estimated variables
        self.estimated_state = x_0
        self.estimation_covariance = P_0

    def predict(self):
        self.estimated_state = self.A @ self.estimated_state
        self.estimation_covariance = self.A @ self.estimation_covariance @ self.A.transpose() + self.Q

    def update(self, measurement):
        S = self.H @ self.estimation_covariance @ self.H.transpose() + self.R
        V = measurement - self.H @ self.estimated_state
        K = self.estimation_covariance @ self.H.transpose() @ np.linalg.inv(S)

        self.estimated_state = self.estimated_state + K @ V
        self.estimation_covariance = self.estimation_covariance - K @ S @ K.transpose()
