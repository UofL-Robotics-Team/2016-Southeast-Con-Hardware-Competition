from rps import RPS
import time

# Create RPS
rps = RPS('COM5', 115200, 'COM3', 115200)

rps.connect()
rps.travel(150, 10, 0)
time.sleep(2)
rps.travel(150, -10, 0)
rps.disconnect()