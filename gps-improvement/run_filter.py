import argparse
import numpy as np
import matplotlib.pyplot as plt
import math
import sys

from sensors.dead_reckoning import DeadReckoning
from sensors.gps import GPS
from sensors.ground_truth import GroundTruth
from sensors.motor_encoder import MotorEncoder
from sensors.steering_angle_encoder import SteeringAngleEncoder

from filter.kalman_filter import ExtendedKalmanFilter, create_model_parameters
from utils import apply_coordinate_rotation, normalize_angle
from filter.statistics import calculate_error_between_ground_truth_and_estimate, \
    calculate_error_between_ground_truth_and_odometry

# Custom filter parameters
GPS_THRESHOLD_M = 13
DRIFT_TIME_SECONDS = 100


def run_filter(simulation_time, gps_threshold_m, drift_time_seconds):
    # Initialize sensors
    ground_truth = GroundTruth()
    state = DeadReckoning()
    measurements = GPS()
    motor_encoder = MotorEncoder()
    steering_angle_encoder = SteeringAngleEncoder()

    # Synchronize all sensors
    state.synchronize_data(ground_truth)
    state.synchronize_data(measurements)
    ground_truth.synchronize_data(measurements)

    motor_encoder.synchronize_data(state)
    steering_angle_encoder.synchronize_data(state)

    # Define initial estimates
    x0 = np.array([state.positions_x[state.timestamp_index],
                   state.positions_y[state.timestamp_index],
                   -1.15*math.pi/3])
    P0 = 0 * np.eye(3)
    start_timestamp = motor_encoder.get_timestamp()

    estimated_state = np.zeros((simulation_time, 3))
    estimation_covariance = np.zeros((simulation_time, 3, 3))

    # Initialize filter
    np.random.seed(0)
    H, Q, R = create_model_parameters()
    dt = 0.02
    wheelbase = 1.21

    kalman_filter = ExtendedKalmanFilter(
        dt=dt, wheelbase=wheelbase, Q=Q, H=H, R=R, x_0=x0, P_0=P0,
        gps_threshold_m=gps_threshold_m, drift_time_seconds=drift_time_seconds)

    # Main loop: predict and update steps
    for k in range(simulation_time):
        kalman_filter.predict(
            motor_encoder.get_next_measurement(),
            steering_angle_encoder.get_next_measurement(), iteration_step=k)

        if measurements.get_timestamp() <= motor_encoder.get_timestamp():
            kalman_filter.update(measurements.get_next_measurement())

        estimated_state[k, :] = kalman_filter.estimated_state
        estimation_covariance[k, ...] = kalman_filter.estimation_covariance

    # Format data for X-Y plots
    end_timestamp = motor_encoder.get_timestamp()
    ground_truth_data = ground_truth.get_measurements_until_timestamp(
        end_timestamp)
    measurements = GPS()
    measurements_data = measurements.get_measurements_until_timestamp(
        end_timestamp)
    state_data = state.get_measurements_until_timestamp(end_timestamp)

    ground_truth_x = [position[0] for position in ground_truth_data]
    ground_truth_y = [position[1] for position in ground_truth_data]

    measurements_x = [position[0] for position in measurements_data]
    measurements_y = [position[1] for position in measurements_data]

    state_x = [position[0] for position in state_data]
    state_y = [position[1] for position in state_data]

    # Compute error statistics
    error_x, error_y = calculate_error_between_ground_truth_and_estimate(
        estimated_state, start_timestamp, end_timestamp)

    error_odometry_x, error_odometry_y = calculate_error_between_ground_truth_and_odometry(
        start_timestamp, end_timestamp)

    # X-Y plots
    plt.plot(ground_truth_x, ground_truth_y, 'b')
    plt.plot(estimated_state[:, 0], estimated_state[:, 1], 'g')
    plt.plot(measurements_x, measurements_y, 'r')
    plt.plot(state_x, state_y, 'k')
    plt.title('X-Y plot')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend(['ground_truth', 'estimated_state',
                'GPS measurements', 'Dead Reckoning'])
    plt.grid()

    # Error plots
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(error_x, 'g')
    plt.plot(error_odometry_x, 'k')
    plt.title('Errors from ground truth - x axis')
    plt.legend(['estimate errors', 'Dead Reckoning errors'])
    plt.xlabel('samples')
    plt.ylabel('error [m]')
    plt.grid()
    plt.subplot(2, 1, 2)
    plt.plot(error_y, 'g')
    plt.plot(error_odometry_y, 'k')
    plt.title('Errors from ground truth - y axis')
    plt.legend(['estimate errors', 'Dead Reckoning errors'])
    plt.xlabel('samples')
    plt.ylabel('error [m]')
    plt.grid()
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--dataset', help='dataset type: \'basic\' or \'full\'')

    args = parser.parse_args()

    if args.dataset is None or args.dataset == 'full':
        simulation_time = 15000
    elif args.dataset == 'basic':
        simulation_time = 5000
    else:
        print('Please enter a valid dataset (\'basic\' or \'full\')')
        sys.exit()

    run_filter(simulation_time, GPS_THRESHOLD_M, DRIFT_TIME_SECONDS)
