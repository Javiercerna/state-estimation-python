import matplotlib.pyplot as plt
import math

from utils import convert_gps_to_xy, apply_coordinate_rotation


def get_groundtruth_data():
    GROUND_TRUTH_FILENAME = './dataset/rtk_gps.txt'
    GROUND_TRUTH_LATS_COL = 4
    GROUND_TRUTH_LONS_COL = 5

    with open(file=GROUND_TRUTH_FILENAME, mode='rb') as f:
        ground_truth_lats = []
        ground_truth_lons = []

        for line in f.readlines():
            data = str(line).split(' ')
            ground_truth_lats.append(float(data[GROUND_TRUTH_LATS_COL]))
            ground_truth_lons.append(float(data[GROUND_TRUTH_LONS_COL]))

    return convert_gps_to_xy(ground_truth_lats, ground_truth_lons)


def get_gps_data():
    GPS_FILENAME = './dataset/ublox_gps.txt'
    GPS_LATS_COL = 4
    GPS_LONS_COL = 5

    with open(file=GPS_FILENAME, mode='rb') as f:
        gps_lats = []
        gps_lons = []

        for line in f.readlines():
            data = str(line).split(' ')
            gps_lats.append(float(data[GPS_LATS_COL]))
            gps_lons.append(float(data[GPS_LONS_COL]))

        return convert_gps_to_xy(gps_lats, gps_lons)


def get_dead_reckoning_data():
    DEAD_RECKONING_FILENAME = './dataset/dead_reckoning.txt'
    DEAD_RECKONING_X_COL = 1
    DEAD_RECKONING_Y_COL = 2
    DEAD_RECKONING_THETA_COL = 3

    with open(file=DEAD_RECKONING_FILENAME, mode='rb') as f:
        dead_reckoning_x = []
        dead_reckoning_y = []
        dead_reckoning_theta = []

        for line in f.readlines():
            data = [val.strip('\\n\'') for val in str(line).split(' ')]
            dead_reckoning_x.append(float(data[DEAD_RECKONING_X_COL]))
            dead_reckoning_y.append(float(data[DEAD_RECKONING_Y_COL]))
            dead_reckoning_theta.append(float(data[DEAD_RECKONING_THETA_COL]))

    dead_reckoning_x, dead_reckoning_y = apply_coordinate_rotation(
        dead_reckoning_x, dead_reckoning_y, rotation_angle=math.pi / 3)

    return dead_reckoning_x, dead_reckoning_y, dead_reckoning_theta


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
    ground_truth_x, ground_truth_y = get_groundtruth_data()
    gps_x, gps_y = get_gps_data()
    dead_reckoning_x, dead_reckoning_y, dead_reckoning_theta = get_dead_reckoning_data()

    plt.plot(ground_truth_x[0:1000], ground_truth_y[0:1000], 'b')
    plt.plot(gps_x[0:100], gps_y[0:100], 'r')
    plt.plot(dead_reckoning_x[0:5000], dead_reckoning_y[0:5000], 'g')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend(['ground truth', 'measurements', 'odometry'])
    plt.show()
