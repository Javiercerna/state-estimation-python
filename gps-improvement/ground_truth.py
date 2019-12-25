import matplotlib.pyplot as plt

from sensor import Sensor
from utils import convert_gps_to_xy

GROUND_TRUTH_FILENAME = './dataset/rtk_gps.txt'
GROUND_TRUTH_TIMESTAMPS = 0
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
        self._timestamps.append(float(data[GROUND_TRUTH_TIMESTAMPS]))
        self.lats.append(float(data[GROUND_TRUTH_LATS_COL]))
        self.lons.append(float(data[GROUND_TRUTH_LONS_COL]))


if __name__ == '__main__':
    ground_truth = GroundTruth()
    GPS_FILENAME = './dataset/ublox_gps.txt'
    gps = GroundTruth(filename=GPS_FILENAME)

    ground_truth.synchronize_data(gps)

    plt.plot(ground_truth.positions_x[0:1000],
             ground_truth.positions_y[0:1000], 'b')
    plt.plot(gps.positions_x[0:100], gps.positions_y[0:100], 'r')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend(['ground truth', 'measurements'])
    plt.show()
