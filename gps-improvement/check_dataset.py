import matplotlib.pyplot as plt
import math

from dead_reckoning import DeadReckoning
from gps import GPS
from ground_truth import GroundTruth


def get_translational_speed():
    MOTOR_ENCODER_FILENAME = './dataset/motor_encoder.txt'
    TRANSLATIONAL_SPEED_COL = 2

    with open(file=MOTOR_ENCODER_FILENAME, mode='rb') as f:
        translational_speed = []

        for line in f.readlines():
            data = str(line).split(' ')
            translational_speed.append(float(data[TRANSLATIONAL_SPEED_COL]))

    return translational_speed


def get_steering_angles():
    STEERING_ANGLE_FILENAME = './dataset/steering_angle_encoder.txt'
    STEERING_ANGLE_COL = 1

    with open(file=STEERING_ANGLE_FILENAME, mode='rb') as f:
        steering_angles = []

        for line in f.readlines():
            data = str(line).split(' ')
            steering_angles.append(float(data[STEERING_ANGLE_COL]))

    return steering_angles


if __name__ == '__main__':
    ground_truth = GroundTruth()
    gps = GPS()
    dead_reckoning = DeadReckoning()

    # Synchronize from highest resolution to lowest
    # i.e. dead_reckoning -> ground_truth -> gps
    dead_reckoning.synchronize_data(ground_truth)
    dead_reckoning.synchronize_data(gps)
    ground_truth.synchronize_data(gps)

    plt.plot(ground_truth.positions_x[0:1000],
             ground_truth.positions_y[0:1000], 'b')
    plt.plot(gps.positions_x[0:100], gps.positions_y[0:100], 'r')
    plt.plot(
        dead_reckoning.positions_x[0: 5000],
        dead_reckoning.positions_y[0: 5000],
        'g')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend(['ground truth', 'measurements', 'odometry'])
    plt.show()
