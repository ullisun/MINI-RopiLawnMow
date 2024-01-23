from time import time, sleep, strftime
import sys, os
import signal
import json

'''
This script supplies the “drive.py” file with drive commands
are stored in separate driving files. e.g. B. cm_vor or ccw90, cw90 .
The “drive.py” file processes these commands and reports a status when 
it has processed the drive commands. Meanwhile, this script (mower.py) 
scans the measured distance of the ToF sensors, which is saved by 
the laser.py script in the /run/shm/distance file. If the distance 
is too short, the drive.py will be interrupted and the file “mover.py” 
provides a new driving command. 
In the moment it runs until the battery is empty
To do:
1. Before starting the new forward drive, check that there is no obstacle in front
2. Implement obstacle detection with the camera
3. Implement a selection of different mowing patterns, e.g. Edge cutting ...

'''

def signal_term_handler(signal, frame):
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_term_handler)

def cleanup():
    with open("/run/shm/stop","w") as f:
                f.write("")
    if os.path.isfile("/run/shm/ready"):
       os.remove("/run/shm/ready") 
    if os.path.isfile("/run/shm/drivechain"):
       os.remove("/run/shm/drivechain")       
    log("[ Main Script Mower finished ]")


def log(msg):
    msg = strftime('%d-%b-%Y/%H:%M:%S') +" " + str(msg)
    if debug:
        print(msg)
    file="/home/pi/dev/logfile.txt"
    with open(file, 'a') as f:
        f.write(msg+"\n")

def readDistance():
    file="/run/shm/distance"
    cnt=""    
    if os.path.isfile(file):
        with open(file,"r")as f:
            cnt=f.read().strip()
        return cnt

def checkdrive():
    if os.path.isfile("/run/shm/drivechain"):
        return True
    else:
        return False
    
        
def readRPM():     
    file="/run/shm/rpm"  
    cnt=";"
    if os.path.isfile(file):
        with open(file,"r")as f:
            cnt=f.read().strip()
        return cnt
    else:
        return cnt  

debug=True
old_l=999 
old_r=999
ready=True

def main():
    global old_l,old_r,ready
    with open("/run/shm/ready","w") as f:
            f.write("") 
    while True:
        l=999
        r=999
        rpm="0;0"
        lrpm=0
        rrpm=0
        dist=readDistance()
        rpm=readRPM()
        isdriveactive=checkdrive()
        try:
            obj=json.loads(dist)
            l=obj["left"]
            r=obj["right"]
            #print(l,"\t",r,"\t",lrpm,"\t",rrpm)
        except:
           continue
        try:
            lrpm=int(rpm.split(";")[0])
            rrpm=int(rpm.split(";")[0])
            print(l,"\t",r,"\t",lrpm,"\t",rrpm)            
        except Exception as e:
            log(str(e))
            continue
        #print(l,"\t",r,"\t",old_l,"\t",old_r,"\t",lrpm,"\t",rrpm,"\t",ready)
        
        # here starts the measurement of the ToFs
        # ToFs returns a value of 819 if no obstical is detected 
        if old_l < 819  and old_r < 819:
            if (l < old_l or r < old_r) and (l <= 20 or r<=20) and (lrpm +  rrpm) > 50 and isdriveactive==True:
                with open("/run/shm/stop","w") as f:              # stops the drive.py
                    f.write("")
                    sleep(1)
                if l + 3 < r:            
                    with open("/run/shm/drivechain","w") as f:    # provide a cw90 in the drivechain
                        f.write("cw90")
                    ready=True
                    #print("Fahre cw90")
                elif r +3  < l:            
                    with open("/run/shm/drivechain","w") as f:    # provide a ccw90 in the drivechain
                        f.write("ccw90")
                    ready=True
                    #print("Fahre ccw90")
                else:
                   with open("/run/shm/drivechain","w") as f:     # provide a ccw180 in the drivechain 
                        f.write("ccw180")
                   ready=True
                   #print("Drehe")
        
        # check if the drive script is ready to accept new commands               
        if os.path.isfile("/run/shm/ready"):    
            sleep(1)
            with open("/run/shm/drivechain","w") as f:
               f.write("cm_vor")                                  # provide a forward drive in the drivechain    
            ready=False    
        old_l=l 
        old_r=r            
        sleep(0.05)
            
if __name__ =="__main__":
    try:
        main()
    except KeyboardInterrupt:
       print(" [ Main Script Mower interrupted by keyboard]")
    except Exception as e:
       log (" [ Fehler im Main Script Mower ] " + str(e))
    finally:
        cleanup()        
