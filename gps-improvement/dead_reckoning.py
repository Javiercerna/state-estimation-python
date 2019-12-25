import matplotlib.pyplot as plt
import math

from sensor import Sensor
from utils import apply_coordinate_rotation

DEAD_RECKONING_FILENAME = './dataset/dead_reckoning.txt'
DEAD_RECKONING_TIMESTAMP_COL = 0
DEAD_RECKONING_X_COL = 1
DEAD_RECKONING_Y_COL = 2
DEAD_RECKONING_THETA_COL = 3


class DeadReckoning(Sensor):
    def __init__(self, filename=DEAD_RECKONING_FILENAME):
        self.positions_x = []
        self.positions_y = []
        self.positions_theta = []

        super().__init__(filename=filename)

        self.positions_x, self.positions_y = apply_coordinate_rotation(
            self.positions_x, self.positions_y, rotation_angle=math.pi / 3)

    def _append_data(self, data):
        self._timestamps.append(float(data[DEAD_RECKONING_TIMESTAMP_COL]))
        self.positions_x.append(float(data[DEAD_RECKONING_X_COL]))
        self.positions_y.append(float(data[DEAD_RECKONING_Y_COL]))
        self.positions_theta.append(float(data[DEAD_RECKONING_THETA_COL]))
