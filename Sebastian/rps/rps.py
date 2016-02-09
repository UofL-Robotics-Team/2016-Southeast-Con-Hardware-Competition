import serial
import messageutils

"""Relative Positioning System (RPS)

See readme.md for detailed description of the RPS as a whole.
"""

__author__ = 'Alex Bennett'
__version__ = '1.00'


class RPS:
    def __init__(self, com_port, baud_rate):
        self._com_port = com_port
        self._baud_rate = baud_rate
        self._serial = None
        self._ready = False

    def connect(self):
        try:
            # Connect to serial
            self._serial = serial.Serial(self._com_port, baudrate=self._baud_rate, timeout=5)

            # Set ready status
            self._ready = True
        except serial.SerialException:
            messageutils.print_error('Could not connect to Arduino serial ports')

    def travel(self, x_dist, y_dist):
        if not self._ready:
            messageutils.print_error('Serial not connected')
            return

        # First get current x/y position from sensors
        try:
            self._sensor_serial.write('p\n')
        except serial.SerialException:
            messageutils.print_error('Serial exception thrown')

    def zero_position(self):
        pass
