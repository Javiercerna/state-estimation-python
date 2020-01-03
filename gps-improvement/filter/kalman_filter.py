"""
Based on the work of Marko Cotra.

References:
* https://gist.github.com/cotramarko
* https://towardsdatascience.com/wtf-is-sensor-fusion-part-2-the-good-old-kalman-filter-3642f321440

"""
import numpy as np
import math

from utils import normalize_angle


class ExtendedKalmanFilter():
    def __init__(
            self, dt, wheelbase, Q, H, R, x_0, P_0, gps_threshold_m=None,
            drift_time_seconds=None):
        """
        Constructor for the Extended Kalman Filter.

        :param dt: timestamp difference, or sampling time [s]
        :param wheelbase: wheelbase of the vehicle [m]
        :param Q: process covariance matrix (3x3)
        :param H: output (measurement) matrix (3x2)
        :param R: measurement covariance matrix (2x2)
        :param x_0: initial estimate state (3x1)
        :param P_0: initial estimate covariance (3x3)
        :param gps_threshold_m: custom filter parameter (optional)
        :param drift_time_seconds: custom filter parameter (optional) 
        """
        # Model parameters
        self.dt = dt
        self.wheelbase = wheelbase
        self.Q_initial = Q
        self.Q = self.Q_initial
        self.H = H
        self.R = R

        # Custom filter parameters
        self.gps_threshold_m = gps_threshold_m
        self.drift_time_seconds = drift_time_seconds

        # Estimation variables
        self.estimated_state = x_0
        self.estimation_covariance = P_0

    def predict(self, v_transl, steering_angle, iteration_step=None):
        """
        Executes the predict step of the Extended Kalman Filter.
        x_k = f(x_k-1, u_k)
        P_k = G_k*P_k-1*G_k-1' + Q

        :param v_transl: translational velocity of the vehicle [m/s]
        :param steering_angle: steering angle of the vehicle [rad/s]
        :param iteration_step: discrete time step of the simulation (optional)
        """
        u = np.array(
            [v_transl * self.dt * np.cos(self.estimated_state[2]),
             v_transl * self.dt * np.sin(self.estimated_state[2]),
             v_transl * self.dt * np.tan(steering_angle) / self.wheelbase])
        G = self._compute_jacobian(v_transl)

        self.estimated_state = self.estimated_state + u
        self.estimated_state[2] = normalize_angle(self.estimated_state[2])
        self.estimation_covariance = G @ self.estimation_covariance @ G.transpose() + self.Q

        # Custom filter modification: take drift into account
        if self.drift_time_seconds is not None and iteration_step is not None:
            if self._is_time_to_model_drift(iteration_step):
                self.Q = self._get_Q_drift()
            else:
                self.Q = self.Q_initial

    def update(self, measurement):
        """
        Executes the update step of the Extended Kalman Filter.
        S_k = H*P_k*H' + R
        V_k = z_k - h(x_k)

        K_k = P_k*H'*inv(S_k)
        x_k = x_k + K_k*V_k
        P_k = (I - K_k*H)*P_k

        :param measurement: GPS measurement (gps_x, gps_y) [m]. Also known as
                            the variable "z" of the EKF.
        """
        S = self.H @ self.estimation_covariance @ self.H.transpose() + self.R
        V = measurement - self.H @ self.estimated_state

        # Custom filter modification: Use GPS measurements only when there is enough difference
        if self.gps_threshold_m is not None:
            distance_innovation = math.sqrt(V[0] ** 2 + V[1] ** 2)
            if distance_innovation < self.gps_threshold_m:
                return

        K = self.estimation_covariance @ self.H.transpose() @ np.linalg.inv(S)

        self.estimated_state = self.estimated_state + K @ V
        self.estimation_covariance = self.estimation_covariance - K @ self.H @ self.estimation_covariance

    def _compute_jacobian(self, v_transl):
        """
        Computes the jacobian (numerically) of the input signal "u", with
        respect to the state [x, y, theta]'.
        u = [v_transl * self.dt * cos(theta),
             v_transl * self.dt * sin(theta),
             v_transl * self.dt * tan(steering_angle) / self.wheelbase]

        :param v_transl: translational velocity of the vehicle [m/s]
        """
        return np.block([
            [1, 0, -np.sin(self.estimated_state[2]) * v_transl * self.dt],
            [0, 1, np.cos(self.estimated_state[2]) * v_transl * self.dt],
            [0, 0, 1]])

    def _get_Q_drift(self, sigma_xy=0.1, sigma_theta=0.7):
        """
        Helper function to create a process covariance matrix that simulates
        drift, called Q_drift. The sigma values are higher than usual to 
        represent noisy measurements

        :param sigma_xy: standard deviations of x and y axis [m]
        :param sigma_theta: standard deviation of theta [rad]
        :return: Q_drift (process covariance matrix for drift)
        """
        sigma_x = sigma_xy
        sigma_y = sigma_xy
        sigma_theta = sigma_theta

        return np.array([[sigma_x ** 2, 0, 0],
                         [0, sigma_y ** 2, 0],
                         [0, 0, sigma_theta ** 2]])

    def _is_time_to_model_drift(self, iteration_step):
        """
        Helper function to decide if the motion model should take drift into
        account. It becomes True every self.drift_time_seconds and remains
        True for the following 5 iteration steps.

        :param iteration_step: discrete time step of the simulation
        :return: True/False
        """
        iteration_time_seconds = iteration_step * self.dt
        return iteration_time_seconds > self.drift_time_seconds and \
            (iteration_step % (self.drift_time_seconds / self.dt)) in range(5)


def create_model_parameters(
        sigma_x=0.02, sigma_y=0.02, sigma_theta=0, lambda_gps=20):
    """
    Helper function to create the model parameters (H, Q, R) of a model
    that has three states [x, y, theta]' and measures two states [x, y]'.

    :param sigma_x: standard deviation of x [m]
    :param sigma_y: standard deviation of y [m]
    :param sigma_theta: standard deviation of theta [rad]
    :param lamda: standard deviation of GPS measurements [m]
    :return: H (measurement matrix), Q (process covariance matrix),
             R (measurement covariance matrix)
    """
    # Motion model parameters
    Q = np.array([[sigma_x ** 2, 0, 0],
                  [0, sigma_y ** 2, 0],
                  [0, 0, sigma_theta ** 2]])

    # Measurement model parameters
    H = np.array([[1, 0, 0],
                  [0, 1, 0]])
    R = np.array([[lambda_gps ** 2, 0],
                  [0, lambda_gps ** 2]])

    return H, Q, R
