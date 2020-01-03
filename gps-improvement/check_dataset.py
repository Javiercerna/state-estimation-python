import matplotlib.pyplot as plt
import math

from sensors.dead_reckoning import DeadReckoning
from sensors.gps import GPS
from sensors.ground_truth import GroundTruth
from sensors.motor_encoder import MotorEncoder
from sensors.steering_angle_encoder import SteeringAngleEncoder

from utils import normalize_angle

DATASET_BASIC_SECONDS = 100
DATASET_FULL_SECONDS = 300

ground_truth = GroundTruth()
gps = GPS()
dead_reckoning = DeadReckoning()
motor_encoder = MotorEncoder()
steering_angle_encoder = SteeringAngleEncoder()

# Synchronize from highest resolution to lowest
# i.e. dead_reckoning -> ground_truth -> gps
dead_reckoning.synchronize_data(ground_truth)
dead_reckoning.synchronize_data(gps)
ground_truth.synchronize_data(gps)

# Synchronize sensors at the same resolution as dead_reckoning
motor_encoder.synchronize_data(dead_reckoning)
steering_angle_encoder.synchronize_data(dead_reckoning)

# Define timestamps to analyze (in microseconds)
start_timestamp = dead_reckoning.get_timestamp()
end_timestamp = start_timestamp + DATASET_BASIC_SECONDS * 1e6

# Format data for X-Y plots
dead_reckoning_measurements = dead_reckoning.get_measurements_until_timestamp(
    end_timestamp)
ground_truth_measurements = ground_truth.get_measurements_until_timestamp(
    end_timestamp)
gps_measurements = gps.get_measurements_until_timestamp(end_timestamp)

dead_reckoning_x = [position[0] for position in dead_reckoning_measurements]
dead_reckoning_y = [position[1] for position in dead_reckoning_measurements]
dead_reckoning_theta = [position[2]
                        for position in dead_reckoning_measurements]

ground_truth_x = [position[0] for position in ground_truth_measurements]
ground_truth_y = [position[1] for position in ground_truth_measurements]

gps_x = [position[0] for position in gps_measurements]
gps_y = [position[1] for position in gps_measurements]

# X-Y plots
plt.plot(ground_truth_x, ground_truth_y, 'b')
plt.plot(gps_x, gps_y, 'r')
plt.plot(dead_reckoning_x, dead_reckoning_y, 'k')
plt.title('X-Y plot')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend(['Ground truth'])
plt.grid()
plt.legend(['Ground truth', 'GPS measurements', 'Dead Reckoning'])

plt.show()
