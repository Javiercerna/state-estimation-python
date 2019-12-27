from sensor import Sensor

GYROMETER_FILENAME = './dataset/gyrometer.txt'
GYROMETER_TIMESTAMP_COL = 0
GYROMETER_ANGULAR_SPEED_COL = 1


class Gyrometer(Sensor):
    def __init__(self, filename=GYROMETER_FILENAME):
        self.angular_speeds = []

        super().__init__(filename=filename)

    def _append_data(self, data):
        self._timestamps.append(float(data[GYROMETER_TIMESTAMP_COL]))
        self.angular_speeds.append(float(data[GYROMETER_ANGULAR_SPEED_COL]))

    def get_next_data(self):
        angular_speed = self.angular_speeds[self.timestamp_index]

        self.timestamp_index += 1

        return angular_speed
