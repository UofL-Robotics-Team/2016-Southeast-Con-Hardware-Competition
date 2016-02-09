from acs import ACS
from mvs import MVS
from rps import RPS
import RPi.GPIO as GPIO

"""Sebastian Brain

This is the main program that interfaces with all applicable models to power Sebastian.
"""

__author__ = 'Alex Bennett'
__version__ = '1.00'

class Sebastian:
    def __init__(self, court_toggle_swtch_pin, start_btn_pin, go_led_pin):
        # Initialize all modules
        self._acs = ACS('COM_PORT')
        self._mvs = MVS()
        self._rps = RPS('COM_PORT')

        # Save pins
        self._court_toggle_swtch_pin = court_toggle_swtch_pin
        self._start_btn_pin = start_btn_pin
        self._go_led_pin = go_led_pin

        # Setup GPIO
        GPIO.setup(self._court_toggle_swtch_pin, GPIO.IN)
        GPIO.setup(self._start_btn_pin, GPIO.IN)
        GPIO.setup(self._go_led_pin, GPIO.OUT)

        # Check which side of the court we're on
        if GPIO.input(self._court_toggle_swtch_pin):
            self._court_side = 1
        else:
            self._court_side = 0

    def run(self):
        # Wait for start button press
        GPIO.wait_for_edge(self._start_btn_pin, GPIO.RISING)

        # ======================= #
        # === START MAIN LOOP === #
        # ======================= #
        while True:

            # Tell RPS to drive


# Define pins
court_toggle_swtch_pin = 22
start_btn_pin = 23

# First initialize Sebastian class
sebastian = Sebastian()

# Run Sebastian
sebastian.run()
