Relative Positioning System (RPS)
---
######Version: 1.00
######Description:
- Utilizes the MPU-6050 and ADNS-9800 to handle all positioning of the robot on the game board
- Communicates with a parent program via Serial

The Relative Positioning System (abbreviated RPS) relies on a set of pre-programmed nodes that the robot will use as a path to drive when commanded. All nodes are created before the start of the game and each node will be traveled to upon receiving the appropriate command from the parent program. Once the node has been reached the RPS will return a success signal, at which point the parent program and proceed to the next task.
