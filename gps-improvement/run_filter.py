import numpy as np
import matplotlib.pyplot as plt
import math

from check_dataset import get_groundtruth_data, get_gps_data, \
    get_dead_reckoning_data, get_translational_speed, get_steering_angles
from kalman_filter import ExtendedKalmanFilter, create_model_parameters
from utils import apply_coordinate_rotation

np.random.seed(0)
H, Q, R = create_model_parameters()

ground_truth = get_groundtruth_data()
state = get_dead_reckoning_data()
measurements = get_gps_data()
measurements = apply_coordinate_rotation(
    measurements[0],
    measurements[1],
    rotation_angle=-math.pi / 3)
v_transl = get_translational_speed()
steering_angles = get_steering_angles()

simulation_time = len(state[0])
print(simulation_time)
dt = 0.02
wheelbase = 1.21

# Initial state
x0 = np.array([state[0][0], state[1][0], state[2][0]])
P0 = 0 * np.eye(3)

kalman_filter = ExtendedKalmanFilter(
    dt=dt, wheelbase=wheelbase, Q=Q, H=H, R=R, x_0=x0, P_0=P0)
estimated_state = np.zeros((simulation_time, 3))
estimation_covariance = np.zeros((simulation_time, 3, 3))

for k in range(simulation_time):
    kalman_filter.predict(v_transl[k], steering_angles[k])
    kalman_filter.update(
        [measurements[0][int(k / 50)],
         measurements[1][int(k / 50)]])

    estimated_state[k, :] = kalman_filter.estimated_state
    estimation_covariance[k, ...] = kalman_filter.estimation_covariance


plt.plot(ground_truth[0][0:1000], ground_truth[1][0:1000], 'b')
plt.plot(estimated_state[0:5000, 0], estimated_state[0:5000, 1], 'g')
print(estimated_state[0][-1], estimated_state[1][-1])
plt.plot(measurements[0][0:100], measurements[1][0:100], 'r')

plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend(['ground_truth', 'estimated_state', 'measurements'])
plt.show()
