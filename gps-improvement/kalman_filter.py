"""
Based on the work of Marko Cotra.

References: 
* https://gist.github.com/cotramarko
* https://towardsdatascience.com/wtf-is-sensor-fusion-part-2-the-good-old-kalman-filter-3642f321440

"""
import numpy as np
import math


class ExtendedKalmanFilter():
    def __init__(self, dt, wheelbase, Q, H, R, x_0, P_0):
        # Model parameters
        self.dt = dt
        self.wheelbase = wheelbase
        self.Q = Q
        self.H = H
        self.R = R

        # Estimated variables
        self.estimated_state = x_0
        self.estimation_covariance = P_0

    def predict(self, v_transl, steering_angle):
        u = np.array(
            [v_transl * self.dt * np.cos(self.estimated_state[2]),
             v_transl * self.dt * np.sin(self.estimated_state[2]),
             v_transl * self.dt * np.tan(steering_angle) / self.wheelbase])
        G = self._compute_jacobian(v_transl)

        self.estimated_state = self.estimated_state + u
        self.estimation_covariance = G @ self.estimation_covariance @ G.transpose() + self.Q

    def _compute_jacobian(self, v_transl):
        return np.block([
            [1, 0, -np.sin(self.estimated_state[2]) * v_transl * self.dt],
            [0, 1, np.cos(self.estimated_state[2]) * v_transl * self.dt],
            [0, 0, 1]])

    def update(self, measurement):
        S = self.H @ self.estimation_covariance @ self.H.transpose() + self.R
        V = measurement - self.H @ self.estimated_state
        K = self.estimation_covariance @ self.H.transpose() @ np.linalg.inv(S)

        self.estimated_state = self.estimated_state + K @ V
        self.estimation_covariance = self.estimation_covariance - K @ S @ K.transpose()


def create_model_parameters(
        sigma_x=0.3, sigma_y=0.3, sigma_theta=0.3, lamda=50):
    # Motion model parameters
    Q = np.block([[sigma_x ** 2, 0, 0],
                  [0, sigma_y ** 2, 0],
                  [0, 0, sigma_theta ** 2]])

    # Measurement model parameters
    H = np.array([[1, 0, 0],
                  [0, 1, 0]])
    R = (lamda ** 2) * np.eye(2)

    return H, Q, R
