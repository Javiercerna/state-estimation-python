from sensors.dead_reckoning import DeadReckoning
from sensors.ground_truth import GroundTruth


def calculate_error_between_ground_truth_and_estimate(
        estimate, start_timestamp, end_timestamp):
    ground_truth = GroundTruth()
    dead_reckoning = DeadReckoning()

    while dead_reckoning.get_timestamp() < start_timestamp:
        dead_reckoning.get_next_measurement()

    while ground_truth.get_timestamp() < start_timestamp:
        ground_truth.get_next_measurement()

    initial_index = dead_reckoning.timestamp_index

    error_x = []
    error_y = []

    while dead_reckoning.get_timestamp() <= end_timestamp:
        ground_truth_measurement = ground_truth.get_measurement_closest_to_timestamp(
            dead_reckoning.get_timestamp())
        if ground_truth_measurement is None:
            dead_reckoning.get_next_measurement()
            continue
        real_x, real_y = ground_truth_measurement[0], ground_truth_measurement[1]

        # Update the dead_reckoning index, which is shared by the estimate index
        dead_reckoning.get_next_measurement()

        x, y, _ = estimate[dead_reckoning.timestamp_index - initial_index]
        error_x.append(x - real_x)
        error_y.append(y - real_y)

    return error_x, error_y


def calculate_error_between_ground_truth_and_odometry(
        start_timestamp, end_timestamp):
    ground_truth = GroundTruth()
    dead_reckoning = DeadReckoning()

    while dead_reckoning.get_timestamp() < start_timestamp:
        dead_reckoning.get_next_measurement()

    while ground_truth.get_timestamp() < start_timestamp:
        ground_truth.get_next_measurement()

    error_x = []
    error_y = []

    while dead_reckoning.get_timestamp() <= end_timestamp:
        ground_truth_measurement = ground_truth.get_measurement_closest_to_timestamp(
            dead_reckoning.get_timestamp())
        if ground_truth_measurement is None:
            dead_reckoning.get_next_measurement()
            continue
        real_x, real_y = ground_truth_measurement[0], ground_truth_measurement[1]
        x, y, _ = dead_reckoning.get_next_measurement()
        error_x.append(x - real_x)
        error_y.append(y - real_y)

    return error_x, error_y
