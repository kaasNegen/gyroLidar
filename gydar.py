import glob
import json
import serial
import rplidar

from platform import platform
from threading import Thread

import sys

PLATFORM = str(platform()).lower()


class Gyroscope(object):
    port = ''
    baudrate = 9600
    _active_serial_connection = None

    def __init__(self, port, baudrate=9600, timeout=1):
        """Initilize GyroScope object for communicating with the sensor.

        Parameters
        ----------
        :port : str
            Serial port name to which sensor is connected
        :baudrate : int, optional
            Baudrate for serial connection (the default is 115200)
        :timeout : float, optional
            Serial port connection timeout in seconds (the default is 1)
        """

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.connect()

    def connect(self):
        """Connects to the serial port with the name `self.port`. If it was
        connected to another serial port disconnects from it first."""
        if self._active_serial_connection is not None:
            self.disconnect()
        try:
            self._active_serial_connection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        except serial.SerialException as err:
            raise GyroscopeError("Failed to connect to the sensor due to: {}".format(err))

    def disconnect(self):
        """Disconnects from the serial port"""
        if self._active_serial_connection is None:
            return
        self._active_serial_connection.close()

    def iterate_data(self):
        if self._active_serial_connection is None:
            raise GyroscopeError('No active serial connection!')

        read_data = ''

        while True:
            if self._active_serial_connection.in_waiting:
                c = chr(self._active_serial_connection.read()[0])

                if len(read_data) > 0 or c == '{':
                    read_data += c

                if c == '}':
                    data = self.process_data_string(read_data)
                    read_data = ''
                    if data is not None:
                        yield data

    @staticmethod
    def process_data_string(data_string):
        try:
            parsed = json.loads(data_string)
            return parsed['distance'], parsed['degrees']
        except json.decoder.JSONDecodeError:
            return None


"""COMBINING OF THE TWO SENSORS"""


class ConnectionStates(object):
    NONE_CONNECTED = 0b00
    GYRO_CONNECTED = 0b01
    LIDAR_CONNECTED = 0b10
    BOTH_CONNECTED = 0b11  # actually just GYRO_CONNECTED & LIDAR_CONNECTED (bitwise)


class Gydar(object):
    """Class to combine the GyroScope and RPLidar objects to run them in their respective threads."""
    gyro_port = '/dev/ttyUSB0' if PLATFORM.startswith('linux') else 'COM1'
    lidar_port = '/dev/ttyUSB1' if PLATFORM.startswith('linux') else 'COM2'
    connected = ConnectionStates.NONE_CONNECTED

    lidar = None
    gyro = None

    raw_lidar_output = [None] * 361
    raw_gyro_output = (None, None)  # x, y, z degrees?

    lidar_thread = None
    gyro_thread = None

    def __init__(self):
        ports = serial_ports()
        if len(ports) < 2 :
            print('COULDNT FIND MORE THAN 1 COM/USB PORTS!')
        else:
            self.lidar_port = ports[0]
            self.gyro_port = ports[1]
            print('Found USB/COM ports:', ports)

        pass

    def __str__(self):
        return "LIDAR: {}, GYRO: {}, CONNECTED: {}".format(self.lidar_port, self.gyro_port, self.connected)

    def set_lidar_port(self, port_name):
        self.lidar_port = port_name

    def set_gyro_port(self, port_name):
        self.gyro_port = port_name

    def connect(self):
        self.connect_gyro()
        self.connect_lidar()

    def connect_lidar(self):
        try:
            self.lidar = rplidar.RPLidar(self.lidar_port)
        except rplidar.RPLidarException as e:
            print('LIDAR COULDNT CONNECT TO ', self.lidar_port, e)
            return

        self.connected = self.connected | ConnectionStates.LIDAR_CONNECTED

        self.lidar_thread = Thread(target=lidar_loop, args=(self.lidar, self))  # create thread
        self.lidar_thread.start()

    def disconnect_lidar(self, no_join=False):

        self.lidar.stop()
        self.lidar.disconnect()

        self.connected = self.connected & ~ConnectionStates.LIDAR_CONNECTED

        if not no_join:
            self.lidar_thread.join()  # wait for thread to finish

    def connect_gyro(self):
        try:
            self.gyro = Gyroscope(self.gyro_port)
        except GyroscopeError as e:
            print('GYRO COULDNT CONNECT TO ', self.gyro_port, e)
            return

        self.connected = self.connected | ConnectionStates.GYRO_CONNECTED

        self.gyro_thread = Thread(target=gyro_loop, args=(self.gyro, self))  # create thread
        self.gyro_thread.start()

    def disconnect_gyro(self, no_join=False):
        self.connected = self.connected & ~ConnectionStates.GYRO_CONNECTED

        if not no_join:
            self.gyro_thread.join()  # wait for thread to finish


"""THREADED FUNCTIONS"""


def lidar_loop(lidar: rplidar.RPLidar, gydar: Gydar):
    while gydar.connected & ConnectionStates.LIDAR_CONNECTED:
        try:
            for measurements in lidar.iter_scans(max_buf_meas=5000):
                for measurement in measurements:
                    # measurement = (quality, angle, distance)
                    gydar.raw_lidar_output[round(measurement[1])] = measurement[2]

                if not gydar.connected & ConnectionStates.LIDAR_CONNECTED:
                    break

        except rplidar.RPLidarException as e:
            gydar.disconnect_lidar(no_join=True)
            raise GydarError('Connection with Lidar closed unexpectedly!', 'LIDAR', lidar.port) from e


def gyro_loop(gyro: Gyroscope, gydar: Gydar):
    while gydar.connected & ConnectionStates.GYRO_CONNECTED:
        try:
            for measurement in gyro.iterate_data():
                # measurement = (quality, angle, distance)
                gydar.raw_gyro_output = measurement

                if not gydar.connected & ConnectionStates.LIDAR_CONNECTED:
                    break

        except GyroscopeError as e:
            gydar.disconnect_gyro(no_join=True)
            raise GydarError('Connection with Lidar closed unexpectedly!', 'GYROSCOPE', gyro.port) from e


"""EXCEPTIONS"""


class GydarError(Exception):
    """Custom Gydar Exception"""

    def __init__(self, message, sensor_type='', port=''):
        super(GydarError, self).__init__(message)
        self.port = port
        self.type = sensor_type

    def __str__(self):
        return "{} \n\t{} PORT: {} ".format(super(), self.type, self.port)


class GyroscopeError(Exception):
    """Custom Gyroscope Exception (empty, for separate excepting)"""
    pass


def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result
