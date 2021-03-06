import matplotlib.pyplot as plt

from sensors.sensor import Sensor
from utils import convert_gps_to_xy

GROUND_TRUTH_FILENAME = 'dataset/rtk_gps.txt'
GROUND_TRUTH_TIMESTAMP_COL = 0
GROUND_TRUTH_LATS_COL = 4
GROUND_TRUTH_LONS_COL = 5


class GroundTruth(Sensor):
    def __init__(self, filename=GROUND_TRUTH_FILENAME):
        self.lats = []
        self.lons = []

        super().__init__(filename=filename)

        self.positions_x, self.positions_y = convert_gps_to_xy(
            self.lats, self.lons)

    def _append_measurement(self, measurement):
        self._timestamps.append(float(measurement[GROUND_TRUTH_TIMESTAMP_COL]))
        self.lats.append(float(measurement[GROUND_TRUTH_LATS_COL]))
        self.lons.append(float(measurement[GROUND_TRUTH_LONS_COL]))

    def get_next_measurement(self):
        x = self.positions_x[self.timestamp_index]
        y = self.positions_y[self.timestamp_index]

        self.timestamp_index += 1

        return x, y
