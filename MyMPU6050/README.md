# CODE for ESP32 D1 MINI Board
![P1230630](https://github.com/ullisun/MINI-RopiLawnMow/assets/86979044/e789c155-3dfc-4285-94c6-798d8b13a783)

This code provides on Serial Interface of the Gyro Data "Roll, Pitch and YAW".

This data will be read of MINIRopiLawnMows python file [readSWSerial.py](https://github.com/ullisun/MINI-RopiLawnMow/blob/main/robot/readSWSerial.py) and store it frequently (150ms) into /run/shm/heading of the pi

**todo:**
Soldering a taster and a LED to the pcb and update the code

**Goal:** Is the DI MINI Board in the area of the known WLAN, press the taster to log into the WLAN and update
the code if needed via Webserial interface. No USB cable is needed.

