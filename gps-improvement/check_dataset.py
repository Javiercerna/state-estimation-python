import matplotlib.pyplot as plt

from utils import convert_gps_to_xy

GROUND_TRUTH_FILENAME = './dataset/rtk_gps.txt'
GROUND_TRUTH_LATS_COL = 4
GROUND_TRUTH_LONS_COL = 5

GPS_FILENAME = './dataset/ublox_gps.txt'
GPS_LATS_COL = 4
GPS_LONS_COL = 5

with open(file=GROUND_TRUTH_FILENAME, mode='rb') as f:
    ground_truth_lats = []
    ground_truth_lons = []

    for line in f.readlines():
        data = str(line).split(' ')
        ground_truth_lats.append(float(data[GROUND_TRUTH_LATS_COL]))
        ground_truth_lons.append(float(data[GROUND_TRUTH_LONS_COL]))

ground_truth_x, ground_truth_y = convert_gps_to_xy(
    ground_truth_lats, ground_truth_lons)

with open(file=GPS_FILENAME, mode='rb') as f:
    gps_lats = []
    gps_lons = []

    for line in f.readlines():
        data = str(line).split(' ')
        gps_lats.append(float(data[GPS_LATS_COL]))
        gps_lons.append(float(data[GPS_LONS_COL]))

gps_x, gps_y = convert_gps_to_xy(gps_lats, gps_lons)

plt.plot(ground_truth_x[0:1000], ground_truth_y[0:1000], 'b')
plt.plot(gps_x[0:100], gps_y[0:100], 'r')
plt.show()
