#!/usr/bin/python

# MIT License
#
# Copyright (c) 2017 John Bryan Moore
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#  it is based on the code of the PiMowBotIT-SW and modified for 
#  own demands
#


import os
import signal
import sys
sys.path.append('/home/pi/dev/robot')
import VL53L0X
from gpiozero import LED
from time import sleep, strftime

XShutR = 22
XShutL = 23
XShutD = 24

# Handler zur Erkennung von kill SIGTERM und SIGINT
def signal_term_handler(signal, frame):
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_term_handler)

# Define delay between readings
delay = 0.3 # alle 300ms Abstandsmessung durchfuehren

# LiDAR available
LiDAR = False

# beide TOF-Sensoren per XSHUT deaktivieren (aktive-low)
xledR = LED(XShutR)
xledL = LED(XShutL)
xledD = LED(XShutD)   # optional fuer LiDAR

def shutd():
    if LiDAR:
        xledD.off() # GPIO24 = Low
    xledR.off() # GPIO22 = Low
    xledL.off() # GPIO23 = Low

shutd()

if LiDAR:
    tof_d = VL53L0X.VL53L0X(address=0x2D)

# Create first VL53L0X object passing new address
tof_l = VL53L0X.VL53L0X(address=0x2A)
# Create second VL53L0X object
tof_r = VL53L0X.VL53L0X(address=0x2B)

def init():
    xledL.on() # GPIO23 = High ToF_l-aktivieren
    sleep(0.50)
    tof_l.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
    timing = tof_l.get_timing()
    xledR.on() # GPIO22 = High ToF_r-aktivieren
    sleep(0.50)
    tof_r.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
    timing = tof_r.get_timing()
    if LiDAR:
            xledD.on() # GPIO24 = High ToF_d-aktivieren
            sleep(0.50)
            tof_d.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
            timing = tof_d.get_timing()

init()

def store(para):
    print(para)
    with open("/run/shm/distance","w") as f:
        f.write(para)

try:
   while True:
        DL = int (tof_l.get_distance() / 10)
        DR = int (tof_r.get_distance() / 10)
        store('{"left":'+str(DL)+',"right":'+str(DR)+'}')
        sleep(delay)      # wait 300ms

except KeyboardInterrupt:
   print(strftime('%d-%b-%Y/%H:%M:%S') + " [TOF meassuring interupted... ]")

finally:
   tof_l.stop_ranging()
   tof_r.stop_ranging()
   if LiDAR:
      tof_d.stop_ranging()
   shutd()
   print(strftime('%d-%b-%Y/%H:%M:%S') + " [TOF meassuring finished !!! ]")
