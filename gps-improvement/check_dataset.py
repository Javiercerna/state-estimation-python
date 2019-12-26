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

start_timestamp = dead_reckoning.get_timestamp()
end_timestamp = start_timestamp + 100e6

dead_reckoning_data = dead_reckoning.get_data_until_timestamp(end_timestamp)
ground_truth_data = ground_truth.get_data_until_timestamp(end_timestamp)
gps_data = gps.get_data_until_timestamp(end_timestamp)

dead_reckoning_x = [position[0] for position in dead_reckoning_data]
dead_reckoning_y = [position[1] for position in dead_reckoning_data]

ground_truth_x = [position[0] for position in ground_truth_data]
ground_truth_y = [position[1] for position in ground_truth_data]

gps_x = [position[0] for position in gps_data]
gps_y = [position[1] for position in gps_data]

plt.plot(ground_truth_x, ground_truth_y, 'b')
plt.plot(gps_x, gps_y, 'r')
plt.plot(dead_reckoning_x, dead_reckoning_y, 'g')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend(['ground truth', 'measurements', 'odometry'])
plt.show()
