import abc


class Sensor(abc.ABC):
    def __init__(self, filename):
        self.filename = filename
        self._timestamps = []
        self.timestamp_index = 0

        self._read_data_from_file()
        self.timedelta = self._calculate_timedelta()

    def _read_data_from_file(self):
        with open(file=self.filename, mode='rb') as f:
            for line in f.readlines():
                data = [
                    val.rstrip('\\n\'').lstrip('b\'')
                    for val in str(line).split(' ')]
                self._append_data(data)

    @abc.abstractmethod
    def _append_data(self, data):
        pass

    def _calculate_timedelta(self):
        return (self._timestamps[1] - self._timestamps[0])

    def get_timestamp(self):
        return self._timestamps[self.timestamp_index]

    @abc.abstractmethod
    def get_next_data(self):
        pass

    def get_data_closest_to_timestamp(self, timestamp):
        if self.get_timestamp() > timestamp:
            return None

        while self.get_timestamp() <= timestamp:
            data = self.get_next_data()

        return data

    def get_data_until_timestamp(self, timestamp):
        data = []

        while self.get_timestamp() <= timestamp:
            data.append(self.get_next_data())

        return data

    def synchronize_data(self, other_sensor):
        original_timestamp_index = self.timestamp_index
        other_sensor_original_timestamp_index = other_sensor.timestamp_index

        first_way_sensor_increment = 0
        first_way_other_sensor_increment = 0

        while self.get_timestamp() < other_sensor.get_timestamp():
            self.timestamp_index += 1
            first_way_sensor_increment += 1

        while other_sensor.get_timestamp() < self.get_timestamp():
            other_sensor.timestamp_index += 1
            first_way_other_sensor_increment += 1

        first_way_difference = self.get_timestamp() - other_sensor.get_timestamp()

        self.timestamp_index = original_timestamp_index
        other_sensor.timestamp_index = other_sensor_original_timestamp_index
        second_way_sensor_increment = 0
        second_way_other_sensor_increment = 0

        while other_sensor.get_timestamp() < self.get_timestamp():
            other_sensor.timestamp_index += 1
            second_way_other_sensor_increment += 1

        while self.get_timestamp() < other_sensor.get_timestamp():
            self.timestamp_index += 1
            second_way_sensor_increment += 1

        second_way_difference = self.get_timestamp() - other_sensor.get_timestamp()

        self.timestamp_index = original_timestamp_index
        other_sensor.timestamp_index = other_sensor_original_timestamp_index

        if abs(first_way_difference) <= abs(second_way_difference):
            self.timestamp_index += first_way_sensor_increment
            other_sensor.timestamp_index += first_way_other_sensor_increment
        else:
            self.timestamp_index += second_way_sensor_increment
            other_sensor.timestamp_index += second_way_other_sensor_increment

        print('Sensors synchronized! Timestamps are {} us and {} us'.format(
            self.get_timestamp(), other_sensor.get_timestamp()))
