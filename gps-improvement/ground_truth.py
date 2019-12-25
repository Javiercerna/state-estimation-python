import matplotlib.pyplot as plt

from sensor import Sensor
from utils import convert_gps_to_xy

GROUND_TRUTH_FILENAME = './dataset/rtk_gps.txt'
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

    def _append_data(self, data):
        self._timestamps.append(float(data[GROUND_TRUTH_TIMESTAMP_COL]))
        self.lats.append(float(data[GROUND_TRUTH_LATS_COL]))
        self.lons.append(float(data[GROUND_TRUTH_LONS_COL]))
