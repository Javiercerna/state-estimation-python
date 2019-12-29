from sensors.dead_reckoning import DeadReckoning
from sensors.ground_truth import GroundTruth


def calculate_error_between_ground_truth_and_estimate(
        estimate, start_timestamp, end_timestamp):
    ground_truth = GroundTruth()
    dead_reckoning = DeadReckoning()

    while dead_reckoning.get_timestamp() < start_timestamp:
        dead_reckoning.get_next_data()

    while ground_truth.get_timestamp() < start_timestamp:
        ground_truth.get_next_data()

    initial_index = dead_reckoning.timestamp_index

    error_x = []
    error_y = []

    while dead_reckoning.get_timestamp() <= end_timestamp:
        ground_truth_value = ground_truth.get_data_closest_to_timestamp(
            dead_reckoning.get_timestamp())
        if ground_truth_value is None:
            dead_reckoning.get_next_data()
            continue
        real_x, real_y = ground_truth_value[0], ground_truth_value[1]
        dead_reckoning.get_next_data()  # Just to update the dead_reckoning index
        x, y, _ = estimate[dead_reckoning.timestamp_index - initial_index]
        error_x.append(x - real_x)
        error_y.append(y - real_y)

    return error_x, error_y


def calculate_error_between_ground_truth_and_odometry(
        start_timestamp, end_timestamp):
    ground_truth = GroundTruth()
    dead_reckoning = DeadReckoning()

    while dead_reckoning.get_timestamp() < start_timestamp:
        dead_reckoning.get_next_data()

    while ground_truth.get_timestamp() < start_timestamp:
        ground_truth.get_next_data()

    error_x = []
    error_y = []

    while dead_reckoning.get_timestamp() <= end_timestamp:
        ground_truth_value = ground_truth.get_data_closest_to_timestamp(
            dead_reckoning.get_timestamp())
        if ground_truth_value is None:
            dead_reckoning.get_next_data()
            continue
        real_x, real_y = ground_truth_value[0], ground_truth_value[1]
        x, y, _ = dead_reckoning.get_next_data()
        error_x.append(x - real_x)
        error_y.append(y - real_y)

    return error_x, error_y
