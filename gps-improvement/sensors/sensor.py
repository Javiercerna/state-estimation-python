import abc
import os.path as path


class Sensor(abc.ABC):
    def __init__(self, filename):
        """
        Constructor for the Sensor abstract base class.

        :param filename: filename to read data from. It needs to start with
                         'dataset/' so it reads from the correct folder.
        """
        self.filename = path.abspath(
            path.join(path.dirname(__file__), '..', filename))
        self._timestamps = []
        self.timestamp_index = 0

        self._read_measurements_from_file()
        self.timedelta = self._calculate_timedelta()

    def _read_measurements_from_file(self):
        """
        Reads data from the class variable "filename". It goes line by line,
        storing the data according to the _append_measurement method, which 
        needs to be implemented for each subclass.
        """
        with open(file=self.filename, mode='r') as f:
            for line in f.read().splitlines():
                measurement = [float(value) for value in line.split(' ')]
                self._append_measurement(measurement)

    @abc.abstractmethod
    def _append_measurement(self, data):
        """
        Abstract method. Each subclass must implement how it stores a single
        row from the dataset.

        :param data: Measurement from a single row of the dataset (list)
        """
        pass

    def _calculate_timedelta(self):
        """
        Calculates the timedelta (or timestamp difference, or sampling time),
        between two consecutive measurements.

        :return: timedelta [us]
        """
        return (self._timestamps[1] - self._timestamps[0])

    def get_timestamp(self):
        """
        Gets the current timestamp according to the internal timestamp index.

        :return: timestamp [us]
        """
        return self._timestamps[self.timestamp_index]

    @abc.abstractmethod
    def get_next_measurement(self):
        """
        Abstract method. Each subclass must implement how it returns the next
        measurement corresponding to the dataset. It must also make sure to 
        update the timestamp index.

        Must include: self.timestamp_index += 1
        """
        pass

    def get_measurement_closest_to_timestamp(self, timestamp):
        """
        Gets the measurement which is closest to the timestamp given as a 
        parameter.

        :param timestamp: target timestamp which the measurement should match
                          (or exceed) [us]
        :return: single measurement, which is defined for each subclass of 
                 Sensor from the abstract method get_next_measurement. None if
                 the current timestamp already exceeds the target timestamp.
        """
        if self.get_timestamp() > timestamp:
            return None

        while self.get_timestamp() <= timestamp:
            measurement = self.get_next_measurement()

        return measurement

    def get_measurements_until_timestamp(self, timestamp):
        """
        Gets all the measurement starting from the current timestamp, until
        it exceeds the timestamp given as a parameter.

        :param timestamp: last timestamp for which measurements should be 
                          taken [us]
        :return: list of measurements, which are defined for each subclass of 
                 Sensor from the abstract method get_next_measurement.
        """
        measurements = []

        while self.get_timestamp() <= timestamp:
            measurements.append(self.get_next_measurement())

        return measurements

    def synchronize_data(self, other_sensor, verbose=False):
        """
        Varies the timestamp between two sensors (sensor1, sensor2), until it 
        finds the first index when one is ahead of the other. Then, it repeats 
        the process in the reverse order (sensor2, sensor1) and uses the order
        where the difference between timestamps is the minimum.

        :param other_sensor: object instantiated from subclass of Sensor
        """
        # Synchronizes in normal order (sensor, other_sensor)
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

        # Synchronizes in reverse order (other_sensor, sensor)
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

        # Restores the timestamp indexes to their original values
        self.timestamp_index = original_timestamp_index
        other_sensor.timestamp_index = other_sensor_original_timestamp_index

        # Checks which order gives the minimum difference in timestamp, and
        # updates the timestamp indexes according to that order
        if abs(first_way_difference) <= abs(second_way_difference):
            self.timestamp_index += first_way_sensor_increment
            other_sensor.timestamp_index += first_way_other_sensor_increment
        else:
            self.timestamp_index += second_way_sensor_increment
            other_sensor.timestamp_index += second_way_other_sensor_increment

        if verbose:
            print('Sensors synchronized! Timestamps are {} us and {} us'.format(
                self.get_timestamp(), other_sensor.get_timestamp()))
