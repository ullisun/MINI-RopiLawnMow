#!/usr/bin/env python

# bb_serial.py
# 2014-12-23
# Public Domain
# https://github.com/acampos81/PySBUS/blob/fe08df3418f75d94ccdcff0ef19c9775c2c5803f/pigpio_sample.py

# bit bang receive of serial data



import sys
from time import time, sleep, strftime
import json
import pigpio
import signal

RX=4
#TX=26
msglen=35
baud = 9600
bits = 8
runtime=300
debug=True
ten_char_time = 100.0 / float(baud)
if ten_char_time < 0.1:
    ten_char_time = 0.1

def signal_term_handler(signal, frame):
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_term_handler)


def log(msg):
    msg = strftime('%d-%b-%Y/%H:%M:%S') + str(msg)
    if debug:
        print(msg)
    file="/home/pi/dev/logfile.txt"
    with open(file, 'a') as f:
        f.write(msg+"\n")


def storestream(para):
    #print("store " + para)
    file="/dev/shm/heading.txt"
    with open(file,"w") as f:
        f.write(para)
    print(para)

pi = pigpio.pi()

# fatal exceptions off (so that closing an unopened gpio doesn't error)

pigpio.exceptions = False
pi.bb_serial_read_close(RX)
# fatal exceptions on
pigpio.exceptions = True

# open a gpio to bit bang read the echoed data
pi.bb_serial_read_open(RX, baud, bits)


if __name__ =="__main__":
    
    try:
        n=0
        z=0
        storetime=time()
        while True: #(time()-start) < runtime:
           str_r=""
           count = 1
           text=""
           lt = 0
           while count: # read echoed serial data
              mtext=""
              text=""
              (count, data) = pi.bb_serial_read(RX)
              if count:
                 z=z+1 
                 #text= data.decode("utf-8", "ignore")
                 try:           
                    text= data.decode("utf-8","ignore").strip()
                 except:
                    continue
                 if "{" in text and "}" in text and len(text)> msglen:
                      mtext=text.strip()          
                      #print(text.strip(),"\t",mtext, "\t", len(mtext))      
                      try:
                        mtext=mtext.replace("'",chr(34))
                        print(mtext)
                        # hier wird nur getestet, ob das was abgespeichert werden soll
                        # wirklich ein json object ist. 
                        obj=json.loads(mtext)
                        if time()- storetime > 0.2:
                            storestream(mtext)
                            storetime=time()
                        #print(obj["RAW"], "\t",obj["Pitch"],  "\t",obj["YAW"] ) 
                      except:
                          pass                       
                        
              sleep(ten_char_time) # enough time to ensure more data
        print("secs={} zeilen={} bad={:.3f}%". format(runtime, z, n*100/z))
    except KeyboardInterrupt:
       print(" [ ESP Board Reading interrupted by keyboard]")
    except Exception as e:
       log (" [ Fehler beim oeffnen des seriellen Ports zum ESP Board ] " + str(e))
       print(" [ ESP Board Reading interrupted by error]", e)
    finally:
        log("[ ESP Board Reading finished ]")
        print("[ ESP Board Reading finished ]")
        # free resources
        pi.bb_serial_read_close(RX)
        pi.stop()