from time import time, sleep, strftime
import serial.tools.list_ports
import sys, os
import signal

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

def connect():
    if True: #try:
        ser = serial.Serial("/dev/ttyS0", 9600)        
        #if "ttyUSB" in port[0][0]: 
        #    #ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        #    ser = serial.Serial(port[0][0], 9600, timeout=1)
        #else:
        #    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        ser.reset_input_buffer()
        log(" Port  geoeffnet")
        return ser
    else:#except Exception as e:
        log (" [ Fehler beim oeffnen des seriellen Ports zum ESP Board ] " + str(e))
        return None

def storestream(para):
    #print("store " + para)
    file="/dev/shm/heading.txt"
    with open(file,"w") as f:
        f.write(para)
    print(para)
        
       
if __name__ =="__main__":
    debug=True
    port = [tuple(p) for p in list(serial.tools.list_ports.comports())]
    try:
        ser=connect()
        storetime=time()
        while True:
            if  ser==None:
                log(" Neue Verbindung erforderlich") 
                ser=connect()
            line = ser.readline()
            try:
                Line=line.decode('utf-8').rstrip()
                Line= Line.replace("'",chr(34))   #hier muss das ' in " ungewandelt werden
                if time()- storetime > 0.2:
                    storestream(Line)
                    #print(Line)
                    storetime=time()
            except Exception as e:
                log( "Fehler "+ e)
            sleep(0.10)
        log(" [Konnte die Schnittstelle nicht oeffnen] ")

    except KeyboardInterrupt:
       print(" [ ESP Board Reading interrupted by keyboard]")
    except Exception as e:
       log (" [ Fehler beim oeffnen des seriellen Ports zum ESP Board ] " + str(e))
       print(" [ ESP Board Reading interrupted by error]", e)
    finally:
        log("[ ESP Board Reading finished ]")
        print("[ ESP Board Reading finished ]")
        
