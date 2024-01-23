#!/bin/bash
if pgrep -f "pigpiod" &> /dev/null; then
    echo "pigpiod Domäne lauft schon"
else
    echo "pigpiod wird jetzt gestartet"
    sudo pigpiod 
fi
if pgrep -f "laser" &> /dev/null; then
    echo "Tofs werden schon ausgelesen"
else
    echo "Tof Sensor Script wird gestartet"
    python /home/pi/dev/robot/laser.py  > /dev/null 2>&1 &
fi
if pgrep -f "readSWSerial" &> /dev/null; then
    echo "Gyro wird schon ausgelesen"
else
    echo "Gyro Script wird gestartet"
    python /home/pi/dev/robot/readSWSerial.py  > /dev/null 2>&1 &
fi
if pgrep -f ".PiMowBotIt_pyBLEremote." &> /dev/null; then
    echo "BLE FB Läuft schon"
else
    echo "BLE-FB Script wird gestartet"
    python /home/pi/dev/robot/.PiMowBotIt_pyBLEremote.py  > /dev/null 2>&1 &
fi
