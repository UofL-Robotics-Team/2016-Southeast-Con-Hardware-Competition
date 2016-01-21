Relative Positioning System (RPS)
---
######Version: 1.00
######Description:
The Relative Positioning System (abbreviated RPS) combines the sensor arduino and drive controller arduino to provide a package capable of traveling to specific locations on the gameboard while maintaining positional accuracy.

- Interfaces with two arduinos (one for controlling the wheel motors and the other for handling sensors) via Serial communication
	- The sensor arduino talks to the MPU-6050 and ADNS-9800
	- The drive controller arduino talks to the A4988 stepper motor drivers
- Communicates with a parent program via Serial