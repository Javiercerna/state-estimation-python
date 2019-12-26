from sensor import Sensor

MOTOR_ENCODER_FILENAME = './dataset/motor_encoder.txt'
MOTOR_ENCODER_TIMESTAMP_COL = 0
TRANSLATIONAL_SPEED_COL = 2


class MotorEncoder(Sensor):
    def __init__(self, filename=MOTOR_ENCODER_FILENAME):
        self.translational_speeds = []

        super().__init__(filename=filename)

    def _append_data(self, data):
        self._timestamps.append(float(data[MOTOR_ENCODER_TIMESTAMP_COL]))
        self.translational_speeds.append(float(data[TRANSLATIONAL_SPEED_COL]))

    def get_next_data(self):
        translational_speed = self.translational_speeds[self.timestamp_index]

        self.timestamp_index += 1

        return translational_speed
