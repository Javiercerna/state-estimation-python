import numpy as np
import matplotlib.pyplot as plt
import math

from dead_reckoning import DeadReckoning
from gps import GPS
from ground_truth import GroundTruth
from motor_encoder import MotorEncoder
from steering_angle_encoder import SteeringAngleEncoder

from kalman_filter import ExtendedKalmanFilter, create_model_parameters
from utils import apply_coordinate_rotation

np.random.seed(0)
H, Q, R = create_model_parameters()

ground_truth = GroundTruth()
state = DeadReckoning()
measurements = GPS()
motor_encoder = MotorEncoder()
steering_angle_encoder = SteeringAngleEncoder()

simulation_time = 5000
dt = 0.02
wheelbase = 1.21

# Synchronize all sensors
state.synchronize_data(ground_truth)
state.synchronize_data(measurements)
ground_truth.synchronize_data(measurements)

motor_encoder.synchronize_data(state)
steering_angle_encoder.synchronize_data(state)

# Initial state
x0 = np.array([state.positions_x[state.timestamp_index],
               state.positions_y[state.timestamp_index],
               -1.15*math.pi/3])
P0 = 0 * np.eye(3)
start_timestamp = motor_encoder.get_timestamp()

print('Initial state:', x0)

kalman_filter = ExtendedKalmanFilter(
    dt=dt, wheelbase=wheelbase, Q=Q, H=H, R=R, x_0=x0, P_0=P0)
estimated_state = np.zeros((simulation_time, 3))
estimation_covariance = np.zeros((simulation_time, 3, 3))

for k in range(simulation_time):
    kalman_filter.predict(
        motor_encoder.get_next_data(),
        steering_angle_encoder.get_next_data())

    if measurements.get_timestamp() <= motor_encoder.get_timestamp():
        kalman_filter.update(measurements.get_next_data())

    estimated_state[k, :] = kalman_filter.estimated_state
    estimation_covariance[k, ...] = kalman_filter.estimation_covariance

end_timestamp = motor_encoder.get_timestamp()
ground_truth_data = ground_truth.get_data_until_timestamp(end_timestamp)
measurements = GPS()
measurements_data = measurements.get_data_until_timestamp(end_timestamp)
state_data = state.get_data_until_timestamp(end_timestamp)

ground_truth_x = [position[0] for position in ground_truth_data]
ground_truth_y = [position[1] for position in ground_truth_data]

measurements_x = [position[0] for position in measurements_data]
measurements_y = [position[1] for position in measurements_data]

state_x = [position[0] for position in state_data]
state_y = [position[1] for position in state_data]

plt.plot(ground_truth_x, ground_truth_y, 'b')
plt.plot(estimated_state[:, 0], estimated_state[:, 1], 'g')
plt.plot(measurements_x, measurements_y, 'r')
#plt.plot(state_x, state_y, 'k')

plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend(['ground_truth', 'estimated_state', 'measurements'])
plt.show()
