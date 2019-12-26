from sensor import Sensor

STEERING_ANGLE_FILENAME = './dataset/steering_angle_encoder.txt'
STEERING_ANGLE_TIMESTAMP_COL = 0
STEERING_ANGLE_COL = 1


class SteeringAngleEncoder(Sensor):
    def __init__(self, filename=STEERING_ANGLE_FILENAME):
        self.steering_angles = []

        super().__init__(filename=filename)

    def _append_data(self, data):
        self._timestamps.append(float(data[STEERING_ANGLE_TIMESTAMP_COL]))
        self.steering_angles.append(float(data[STEERING_ANGLE_COL]))
