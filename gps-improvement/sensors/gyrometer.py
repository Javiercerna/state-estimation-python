from sensors.sensor import Sensor

GYROMETER_FILENAME = 'dataset/gyrometer.txt'
GYROMETER_TIMESTAMP_COL = 0
GYROMETER_ANGULAR_SPEED_COL = 1


class Gyrometer(Sensor):
    def __init__(self, filename=GYROMETER_FILENAME):
        self.angular_speeds = []

        super().__init__(filename=filename)

    def _append_measurement(self, measurement):
        self._timestamps.append(float(measurement[GYROMETER_TIMESTAMP_COL]))
        self.angular_speeds.append(
            float(measurement[GYROMETER_ANGULAR_SPEED_COL]))

    def get_next_measurement(self):
        angular_speed = self.angular_speeds[self.timestamp_index]

        self.timestamp_index += 1

        return angular_speed
