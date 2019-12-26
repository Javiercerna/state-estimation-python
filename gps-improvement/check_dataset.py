import matplotlib.pyplot as plt
import math

from dead_reckoning import DeadReckoning
from gps import GPS
from ground_truth import GroundTruth
from motor_encoder import MotorEncoder
from steering_angle_encoder import SteeringAngleEncoder

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

ground_truth_x = ground_truth.positions_x[ground_truth.timestamp_index:ground_truth.timestamp_index+1000]
ground_truth_y = ground_truth.positions_y[ground_truth.timestamp_index:ground_truth.timestamp_index+1000]

gps_x = gps.positions_x[gps.timestamp_index:gps.timestamp_index+100]
gps_y = gps.positions_y[gps.timestamp_index:gps.timestamp_index+100]

dead_reckoning_x = dead_reckoning.positions_x[
    dead_reckoning.timestamp_index:dead_reckoning.timestamp_index+5000]
dead_reckoning_y = dead_reckoning.positions_y[
    dead_reckoning.timestamp_index:dead_reckoning.timestamp_index+5000]

plt.plot(ground_truth_x, ground_truth_y, 'b')
plt.plot(gps_x, gps_y, 'r')
plt.plot(dead_reckoning_x, dead_reckoning_y, 'g')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend(['ground truth', 'measurements', 'odometry'])
plt.show()
