import serial
import messageutils
import time

"""Relative Positioning System (RPS)

See readme.md for detailed description of the RPS.
"""

__author__ = 'Alex Bennett'
__version__ = '1.00'


class RPS:
    def __init__(self, drive_com_port, drive_baud_rate, sensor_com_port, sensor_baud_rate):
        # Drive Arduino serial configuration
        self._drive_com_port = drive_com_port
        self._drive_baud_rate = drive_baud_rate
        self._drive_serial = None

        # Sensor Arduino serial configuration
        self._sensor_com_port = sensor_com_port
        self._sensor_baud_rate = sensor_baud_rate
        self._sensor_serial = None

        self._ready = False

    def connect(self):
        try:
            # Connect to serial
            self._drive_serial = serial.Serial(self._drive_com_port, baudrate=self._drive_baud_rate, timeout=5, write_timeout=5)
            self._sensor_serial = serial.Serial(self._sensor_com_port, baudrate=self._sensor_baud_rate, timeout=5, write_timeout=5)

            # Sleep while connection completes
            time.sleep(1)

            # Set ready status
            self._ready = True

            # Print status
            messageutils.print_info('Arduino serial ports successfully connected')
        except serial.SerialException:
            messageutils.print_error('Could not connect to Arduino serial ports')

    def disconnect(self):
        try:
            # Disconnect serial
            self._drive_serial.close()
            self._sensor_serial.close()

            # Set ready status
            self._ready = False

            # Print status
            messageutils.print_info('Arduino serial ports successfully disconnected')
        except serial.SerialException:
            messageutils.print_error('Could not disconnect from Arduino serial ports')

    def get_position(self):
        # Check RPS status before continuing
        if not self._ready:
            messageutils.print_error('Serial not connected')
            return
        try:
            # Flush serial
            self._sensor_serial.flushInput()
            self._sensor_serial.flushOutput()

            # Sleep for data to transfer
            time.sleep(0.1)

            # Grab the current position
            self._sensor_serial.write('p\n')

            # Parse position data
            x, y, r = self._sensor_serial.readline().replace('\r\n', '').split(',')

            # Return position information
            return float(x), float(y), float(r)
        except serial.SerialException:
            messageutils.print_error('Serial exception thrown')

    def enable_motors(self):
        # Check RPS status before continuing
        if not self._ready:
            messageutils.print_error('Serial not connected')
            return
        try:
            # Flush serial
            self._sensor_serial.flushInput()
            self._sensor_serial.flushOutput()

            # Sleep for data to transfer
            time.sleep(0.1)

            # Send the enable command
            self._drive_serial.write('e\n')
        except serial.SerialException:
            messageutils.print_error('Serial exception thrown')

    def disable_motors(self):
        # Check RPS status before continuing
        if not self._ready:
            messageutils.print_error('Serial not connected')
            return
        try:
            # Flush serial
            self._sensor_serial.flushInput()
            self._sensor_serial.flushOutput()

            # Sleep for data to transfer
            time.sleep(0.1)

            # Send the enable command
            self._drive_serial.write('d\n')
        except serial.SerialException:
            messageutils.print_error('Serial exception thrown')

    def zero_position(self):
        # Check RPS status before continuing
        if not self._ready:
            messageutils.print_error('Serial not connected')
            return
        try:
            # Flush serial
            self._sensor_serial.flushInput()
            self._sensor_serial.flushOutput()

            # Sleep for data to transfer
            time.sleep(0.1)

            # Send the zero command
            self._sensor_serial.write('z\n')
        except serial.SerialException:
            messageutils.print_error('Serial exception thrown')

    def set_velocity(self, x_rate, y_rate):
        # Check RPS status before continuing
        if not self._ready:
            messageutils.print_error('Serial not connected')
            return
        try:
            # Flush serial
            self._sensor_serial.flushInput()
            self._sensor_serial.flushOutput()

            # Sleep for data to transfer
            time.sleep(0.1)

            # Set velocities
            self._drive_serial.write('f%d r%d\n' % (y_rate, x_rate))

            # Sleep to allow transfer to complete
            time.sleep(0.1)
        except serial.SerialException:
            messageutils.print_error('Serial exception thrown')

    def set_rotation_rate(self, rotation_rate):
        # Check RPS status before continuing
        if not self._ready:
            messageutils.print_error('Serial not connected')
            return
        try:
            # Flush serial
            self._sensor_serial.flushInput()
            self._sensor_serial.flushOutput()

            # Sleep for data to transfer
            time.sleep(0.1)

            # Set rotation rate
            self._drive_serial.write('c%d\n' % rotation_rate)
        except serial.SerialException:
            messageutils.print_error('Serial exception thrown')

    def travel(self, rate, x_dist, y_dist):
        # Check RPS status before continuing
        if not self._ready:
            messageutils.print_error('Serial not connected')
            return

        # Status update
        messageutils.print_info('Traveling %d forward/back and %d right/left' % (y_dist, x_dist))

        # Calculate slow rate ahead of time
        if rate >= 500:
            slow_rate = 500
        else:
            slow_rate = rate

        correction_rate = 200

        # Zero position
        self.zero_position()

        # Get current position
        x, y, r = self.get_position()

        # Handle X positioning
        if x_dist != 0:
            if x_dist > 0:
                # Set the velocity
                self.set_velocity(rate, 0)

                # Travel until distance has been reached
                while x < x_dist - 3:
                    x, y, r = self.get_position()
                    messageutils.print_info('X: ' + str(x) + ', Y: ' + str(y) + ', R: ' + str(r))

                # Status update
                messageutils.print_info('Approaching target')

                # Slow down for accuracy
                self.set_velocity(slow_rate, 0)

                # Travel until distance has been reached
                while x < x_dist:
                    x, y, r = self.get_position()
                    messageutils.print_info('X: ' + str(x) + ', Y: ' + str(y) + ', R: ' + str(r))

                # Status update
                messageutils.print_info('Target met')
            elif x_dist < 0:
                # Set the X velocity
                self.set_velocity(-rate, 0)

                # Travel until distance has been reached
                while x > x_dist + 3:
                    x, y, r = self.get_position()
                    messageutils.print_info('X: ' + str(x) + ', Y: ' + str(y) + ', R: ' + str(r))

                # Status update
                messageutils.print_info('Approaching target')

                # Slow down for accuracy
                self.set_velocity(-slow_rate, 0)

                # Travel until distance has been reached
                while x > x_dist:
                    x, y, r = self.get_position()
                    messageutils.print_info('X: ' + str(x) + ', Y: ' + str(y) + ', R: ' + str(r))

                # Status update
                messageutils.print_info('Target met')

            # Distance met, stop moving
            self.set_velocity(0, 0)

            # Determine X error
            x, y, r = self.get_position()

            if abs(y) > 0:
                messageutils.print_info('Correcting Y error')

                # Correct x change
                if y > 0:
                    self.set_velocity(0, -correction_rate)
                    while y > 0:
                        x, y, r = self.get_position()
                        messageutils.print_info('X: ' + str(x) + ', Y: ' + str(y) + ', R: ' + str(r))
                elif y < 0:
                    self.set_velocity(0, correction_rate)
                    while y < 0:
                        x, y, r = self.get_position()
                        messageutils.print_info('X: ' + str(x) + ', Y: ' + str(y) + ', R: ' + str(r))

            # Final stop
            self.set_velocity(0, 0)

        # Zero position
        self.zero_position()

        # Get current position
        x, y, r = self.get_position()

        # Handle Y positioning
        if y_dist != 0:
            if y_dist > 0:
                # Set the velocity
                self.set_velocity(0, rate)

                # Travel until distance has been reached
                while y < y_dist - 3:
                    x, y, r = self.get_position()
                    messageutils.print_info('X: ' + str(x) + ', Y: ' + str(y) + ', R: ' + str(r))

                # Status update
                messageutils.print_info('Approaching target')

                # Slow down for accuracy
                self.set_velocity(0, slow_rate)

                # Travel until distance has been reached
                while y < y_dist:
                    x, y, r = self.get_position()
                    messageutils.print_info('X: ' + str(x) + ', Y: ' + str(y) + ', R: ' + str(r))

                # Status update
                messageutils.print_info('Target met')
            elif y_dist < 0:
                # Set the X velocity
                self.set_velocity(0, -rate)

                # Travel until distance has been reached
                while y > y_dist + 3:
                    x, y, r = self.get_position()
                    messageutils.print_info('X: ' + str(x) + ', Y: ' + str(y) + ', R: ' + str(r))

                # Status update
                messageutils.print_info('Approaching target')

                # Slow down for accuracy
                self.set_velocity(0, -slow_rate)

                # Travel until distance has been reached
                while y > y_dist:
                    x, y, r = self.get_position()
                    messageutils.print_info('X: ' + str(x) + ', Y: ' + str(y) + ', R: ' + str(r))

                # Status update
                messageutils.print_info('Target met')

            # Distance met, stop moving
            self.set_velocity(0, 0)

            # Determine X error
            x, y, r = self.get_position()

            if abs(x) > 0:
                messageutils.print_info('Correcting X error')

                # Correct x change
                if x > 0:
                    self.set_velocity(-correction_rate, 0)
                    while x > 0:
                        x, y, r = self.get_position()
                        messageutils.print_info('X: ' + str(x) + ', Y: ' + str(y) + ', R: ' + str(r))
                elif x < 0:
                    self.set_velocity(correction_rate, 0)
                    while x < 0:
                        x, y, r = self.get_position()
                        messageutils.print_info('X: ' + str(x) + ', Y: ' + str(y) + ', R: ' + str(r))

            # Final stop
            self.set_velocity(0, 0)

    def rotate(self, rate, degrees):
        # Check RPS status before continuing
        if not self._ready:
            messageutils.print_error('Serial not connected')
            return

        # Get initial angle
        x, y, r = self.get_position()

        initial_r = r

        if degrees > 0:
            self.set_rotation_rate(rate)

            while r < initial_r + degrees:
                x, y, r = self.get_position()
                messageutils.print_info('X: ' + str(x) + ', Y: ' + str(y) + ', R: ' + str(r))

            self.set_rotation_rate(0)
        elif degrees < 0:
            self.set_rotation_rate(-rate)

            while r > initial_r + degrees:
                x, y, r = self.get_position()
                messageutils.print_info('X: ' + str(x) + ', Y: ' + str(y) + ', R: ' + str(r))

            self.set_rotation_rate(0)

        messageutils.print_info("Turn complete")
