import matplotlib.pyplot as plt

from sensor import Sensor
from utils import convert_gps_to_xy

GPS_FILENAME = './dataset/ublox_gps.txt'
GPS_TIMESTAMP_COL = 0
GPS_LATS_COL = 4
GPS_LONS_COL = 5


class GPS(Sensor):
    def __init__(self, filename=GPS_FILENAME):
        self.lats = []
        self.lons = []

        super().__init__(filename=filename)

        self.positions_x, self.positions_y = convert_gps_to_xy(
            self.lats, self.lons)

    def _append_data(self, data):
        self._timestamps.append(float(data[GPS_TIMESTAMP_COL]))
        self.lats.append(float(data[GPS_LATS_COL]))
        self.lons.append(float(data[GPS_LONS_COL]))
