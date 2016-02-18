from rps import RPS

# Create RPS
rps = RPS('COM5', 115200, 'COM3', 115200)

rps.connect()
rps.travel(300, 20, 0)
rps.travel(200, -20, 0)
rps.disconnect()
