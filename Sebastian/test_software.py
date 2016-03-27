from rps import RPS

# Create RPS
rps = RPS('COM5', 115200, 'COM3', 115200)

rps.connect()
rps.enable_motors()

for i in range(0, 4):
    rps.travel(900, 0, 35)
    rps.travel(900, 0, -35)

rps.disable_motors()
#rps.travel(900, 0, 20)
#rps.travel(900, -20, 0)
#rps.travel(900, 0, -20)
#rps.travel(900, 20, 0)
rps.disconnect()
