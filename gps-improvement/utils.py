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
    EARTH_RADIUS_M = 6371*1000
    x1, y1 = (EARTH_RADIUS_M * lon1, EARTH_RADIUS_M * lat1)
    x2, y2 = (EARTH_RADIUS_M * lon2, EARTH_RADIUS_M * lat2)

    return (x2 - x1, y2 - y1)
