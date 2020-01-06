"""
Based on the work of Marko Cotra.

References: 
* https://gist.github.com/cotramarko
* https://towardsdatascience.com/wtf-is-sensor-fusion-part-2-the-good-old-kalman-filter-3642f321440

"""

import numpy as np
import matplotlib.pyplot as plt


class MotionModel():
    """
    Motion model of the vehicle. It is defined using the state [x, vx, y, vy]
    and the matrices A and Q.

    :param x: Vehicle x position [m]
    :param vx: Vehicle speed in the x-axis [m/s]
    :param y: Vehicle y position [m]
    :param vy: Vehicle speed in the y-axis [m/s]
    :param A: State transition matrix (must be 4x4)
    :param Q: Motion covariance matrix (must be 4x4)
    """

    def __init__(self, x, vx, y, vy, A, Q):
        self.state = np.array([x, vx, y, vy])
        self.A = A
        self.Q = Q

    def update_state(self):
        zero_mean = np.zeros(self.Q.shape[0])

        self.state = self.A @ self.state + \
            np.random.multivariate_normal(mean=zero_mean, cov=self.Q)


class MeasurementModel():
    """
    Measurement model of the vehicle. The measurement is of the vehicle's 
    position [x, y]. It is defined using the matrices H and R.

    :param H: State measurement matrix (must be 2x4)
    :param R: Measurement covariance matrix (must be 2x2)
    """

    def __init__(self, H, R):
        self.H = H
        self.R = R

    def get_measurement(self, state):
        zero_mean = np.zeros(self.R.shape[0])

        measurement = self.H @ state + \
            np.random.multivariate_normal(mean=zero_mean, cov=self.R)
        return measurement


def create_model_parameters(dt=1, sigma_x=0.1, sigma_y=0.1, lamda=0.3):
    # Motion model parameters
    F = np.array([[1, dt],
                  [0, 1]])

    base_sigma = np.array([[dt ** 3 / 3, dt ** 2 / 2],
                           [dt ** 2 / 2, dt]])
    zeros_2x2 = np.zeros((2, 2))

    sigma_x = (sigma_x ** 2) * base_sigma
    sigma_y = (sigma_y ** 2) * base_sigma

    A = np.block([[F, zeros_2x2],
                  [zeros_2x2, F]])
    Q = np.block([[sigma_x, zeros_2x2],
                  [zeros_2x2, sigma_y]])

    # Measurement model parameters
    H = np.array([[1, 0, 0, 0],
                  [0, 0, 1, 0]])
    R = (lamda ** 2) * np.eye(2)

    return A, H, Q, R


def simulate_system(simulation_time, x0):
    (A, H, Q, R) = create_model_parameters()

    # Create models
    motion_model = MotionModel(x=x0[0], vx=x0[1], y=x0[2], vy=x0[3], A=A, Q=Q)
    measurement_model = MeasurementModel(H, R)

    state = np.zeros((simulation_time, Q.shape[0]))
    measurements = np.zeros((simulation_time, R.shape[0]))

    for k in range(simulation_time):
        motion_model.update_state()
        measurement = measurement_model.get_measurement(motion_model.state)

        state[k, :] = motion_model.state
        measurements[k, :] = measurement

    return state, measurements


if __name__ == '__main__':
    np.random.seed(0)
    state, measurements = simulate_system(
        simulation_time=20, x0=np.array([0, 0.1, 0, 0.1]))

    plt.plot(state[:, 0], state[:, 2], '-bo')  # [x, vx, y, vy]
    plt.plot(measurements[:, 0], measurements[:, 1], 'rx')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend(['true state', 'observed measurement'])
    plt.show()
