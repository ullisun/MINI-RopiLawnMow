# MINI-RopiLawnMow

![P1230626](https://github.com/ullisun/MINI-RopiLawnMow/assets/86979044/c1fe0fa5-532a-42ac-b058-2ecc7cfcc6ca)

# This is the indoor development platform of my [RoπLawnMow](https://githib.com/ullisun/RopiLawnMow) Project

This script supplies the “drive.py” file with drive commands
are stored in separate driving files. e.g. B. cm_vor or ccw90, cw90 .
The “drive.py” file processes these commands and reports a status when it has processed the drive commands
Meanwhile, this script (mower.py) scans the measured distance of the ToF sensors, which is saved by the laser.py script in the /run/shm/distance file. If the distance is too short, the drive.py is interrupted and the file “mover.py” provides a new driving command. At the moment it runs until the battery is empty
  To do:
  1. Before starting the new forward drive, check that there is no obstacle in front
  2. Implement obstacle detection with the camera
  3. Implement a selection of different mowing patterns, e.g. Edge cutting
