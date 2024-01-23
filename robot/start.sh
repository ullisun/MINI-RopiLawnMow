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
if pgrep -f "drive.py" &> /dev/null; then
    echo "drive Script Läuft schon"
else
    echo "drive Script wird gestartet"
    python /home/pi/dev/robot/drive.py  > /dev/null 2>&1 &
fi
