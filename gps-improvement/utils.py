import math


def convert_gps_to_xy(lats, lons):
    """
    Converts GPS measurements to x, y coordinates in m. Since the conversion
    involves displacements, the first pair (x, y) is equal to (0, 0). The next
    coordinates follow the formula
        x = x + dx
        y = y + dy

    where the displacements are given by two consecutive gps (lat, lon)

    :param lats: List of lat coordinates (WGS84)
    :param lons: List of lon coordinates (WGS84)
    :return: x, y coordinates, where the first pair (x,y) is equal to (0,0)
    """
    num_measurements = len(lats)
    x = [0] * num_measurements
    y = [0] * num_measurements

    for ind in range(1, num_measurements):
        dx, dy = _calculate_dx_dy_from_gps(
            lats[ind-1], lons[ind-1], lats[ind], lons[ind])

        x[ind] = x[ind-1] + dx
        y[ind] = y[ind-1] + dy

    return x, y


def _calculate_dx_dy_from_gps(lat1, lon1, lat2, lon2):
    """
    Calculates the displacements of two gps measurements given by (lat1, lon1)
    and (lat2, lon2). The displacements are (dx, dy).

    :param lats: List of lat coordinates (WGS84)
    :param lons: List of lon coordinates (WGS84)
    :return: displacements (dx, dy) [m]
    """
    EARTH_RADIUS_M = 6371000
    # Center point: 45.76020235, 3.11006703333
    center_lat = 0.79866619738

    x1, y1 = (EARTH_RADIUS_M * math.cos(center_lat)
              * lon1, EARTH_RADIUS_M * lat1)
    x2, y2 = (EARTH_RADIUS_M * math.cos(center_lat)
              * lon2, EARTH_RADIUS_M * lat2)

    return (x2 - x1, y2 - y1)


def apply_coordinate_rotation(list_x, list_y, rotation_angle):
    """
    Applies a coordinate rotation of two lists of (x,y) values around the z-axis,
    depending on the value of the rotation angle.

    :param list_x: List of x values [m]
    :param list_y: List of y values [m]
    :param rotation_angle: Angle to rotate around z-axis [rad/s]
    :return: Two lists of rotated (rotated_x, rotated_y).
    """
    list_rotated_x = []
    list_rotated_y = []

    for ind in range(len(list_x)):
        x = list_x[ind]
        y = list_y[ind]
        r = math.sqrt(x ** 2 + y ** 2)
        original_angle = math.atan2(y, x)

        list_rotated_x.append(r*math.cos(original_angle - rotation_angle))
        list_rotated_y.append(r*math.sin(original_angle - rotation_angle))

    return list_rotated_x, list_rotated_y


def normalize_angle(angle):
    """
    Keeps the angle between -pi and pi.

    :param angle: input angle [rad]
    :return: Normalized angle between -pi and pi [rad]
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi
