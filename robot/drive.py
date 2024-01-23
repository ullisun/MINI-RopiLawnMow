from time import time, sleep, strftime
import json
import sys, os
import signal
import pigpio
import read_RPM

'''
damit dieses Script funktioniert muss zuvor das script readSerial.py gestartet sein
es muus auch sudo pigpiod ausgeführt werden. 
'''

class Serdata():
   def __init__(self):
        self.error=0        
        self.stream=""
        self.targeterror=0
        
   def writedata(self, data):
        self.stream=data
  
   def getdata(self, a):
       print(a)
       m = Serdata.stream
       return m  

class YAWPID:
    
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0
        self.currentval=0
        

    def update(self, soll, ist , dt):
        error = soll - ist
        #print("PIDerror ", error)
        if error < -180:
            error=error + 360
        elif error > 180:
            error= error - 360    
        #print("PID Corected error ", error)    
        derivative = (error - self.last_error) / dt
        self.integral += error * dt
        output = (self.Kp * error + self.Ki * self.integral + self.Kd * derivative)*1.4
        self.last_error = error
        if output > 60:
            output=60
        if output < -60:
            output= -60
        return output


class Motor:
    def __init__(self, PinPwm, PinDir, PinBreak, MaxSpeed, MainDir):
        self.PinPwm=PinPwm     
        self.PinDir=PinDir
        self.PinBreak=PinBreak
        self.MaxSpeed=MaxSpeed
        self.SollRPM=0       # Wert der als SollRPM im Programm für den Motor aktuell definiert wird 
        self.MainDir=MainDir # Richtung die der Motor vorrangig dreht Links und Rechts ist unetrschiedlich
        self.pwmValue=0      # Wert, der am PinPwm benötigt wird
        self.pwmoldValue=0   # Wert, der zuvor am PinPwm an stand 
        self.dir=0           #Richtung die er drehen soll
        self.Tic=0           #Tics vom RAD
        self.gesamtTic=0     # Alle Tics des Rades 
        self.RPM=0           # RPMs
        self.Rev=0           #Umdrehungen
        self.targetTics=0    # Tics die ein Rad nach vorgabe machen muss
        self.sollspeed=0
        self.oldspeed=0
        pi.set_mode(self.PinDir, pigpio.OUTPUT)
        pi.set_mode(self.PinBreak, pigpio.OUTPUT)
        pi.write(self.PinBreak,0)
        pi.write(self.PinDir,0)
        self.MStop=False
    
    def getdirection(self,value):
        if value >= 0:
            self.dir=1
        else:
            self.dir=-1    
        return self.dir    
        
    def speed(self,value):
        pi.write(self.PinBreak,0)
        if value > self.MaxSpeed:
           value = self.MaxSpeed
        if value < self.MaxSpeed*-1:
           value = self.MaxSpeed*-1
        if value > 0:
             self.ffd()
        if value < 0:
             self.back()
        pi.set_PWM_dutycycle(self.PinPwm, abs(value))
    
    def stop(self):
        #print("#####################################")
        #print("####   STOP  ",  self.PinPwm, "   ##########")
        #print("#####################################")
        pi.set_PWM_dutycycle(self.PinPwm, 0)
        pi.write(self.PinBreak,0)
        pi.write(self.PinDir,0)
        
    def ffd(self):
        if self.MainDir=="F":
            pi.write(self.PinDir,0)
            pi.write(self.PinBreak,1)
        if self.MainDir=="B":
            pi.write(self.PinDir,0)
            pi.write(self.PinBreak,1)
    def back(self):
        if self.MainDir=="F":
            pi.write(self.PinDir,1)
            pi.write(self.PinBreak,0)
        if self.MainDir=="B":
            pi.write(self.PinDir,1)
            pi.write(self.PinBreak,0)


class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0
        self.deltaTime=time()
        self.startTime=time()
        self.current_value=0
        self.output=0
        self.derivative=0
        self.motorvalue=0
        

    def update(self, error, rpm, dt, maxspeed):
        self.motorvalue = self.motorvalue + error
        if self.motorvalue > maxspeed:
           self.motorvalue=maxspeed
        return self.motorvalue
    
    def oldupdate(self, error, rpm, dt, m):
        dtn=time()-self.deltaTime
        #print(dtn-dt)
        self.derivative = (error - self.last_error) / dtn
        self.integral = self.integral + error * dtn
        output = self.Kp * error + self.Ki * self.integral + self.Kd * self.derivative
        self.last_error = error
        self.current_value = self.current_value + output * dtn
        self.motorvalue = self.motorvalue + self.current_value
        if self.motorvalue > 245:
            self.motorvalue=245
        self.deltaTime=time()
        return self.motorvalue


# Callbacks for interrupt

def L_Hall(GPIO, level, tick):
    if last[lbtn] is not None:
      diff = pigpio.tickDiff(last[lbtn], tick)
      if level==1:
          motorl.Tic += 1
          motorl.gesamtTic +=1
      if motorl.gesamtTic>=ticksperRev:
         motorl.gesamtTic=motorl.gesamtTic-ticksperRev
         motorl.Rev +=1
    last[lbtn] = tick

def R_Hall(GPIO, level, tick):
   if last[rbtn] is not None:
      diff = pigpio.tickDiff(last[rbtn], tick)
      if level==1:
        motorr.Tic +=1
        motorr.gesamtTic +=1
      if motorr.Tic>=ticksperRev:
         motorr.Tic=motorr.Tic-ticksperRev
         motorr.Rev +=1
   last[rbtn] = tick


def signal_term_handler(signal, frame):
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_term_handler)

lbtn=17
rbtn=18
PI=3.141592653589793
ticksperRev=26 #1093   #597
wheeldiameter=9   #cm
i=0
last = [None]*19

#RPML=0
#RPMR=0 

pi=pigpio.pi()

pi.set_mode(lbtn,pigpio.INPUT)
pi.set_mode(rbtn,pigpio.INPUT)

motorr=Motor(PinPwm=16,PinDir=21,PinBreak=20,MaxSpeed=245,MainDir="F")
motorl=Motor(PinPwm=13,PinDir=26,PinBreak=19,MaxSpeed=245,MainDir="B")

HallL=pi.callback(lbtn, pigpio.RISING_EDGE, L_Hall)
motorl.RPM = read_RPM.reader(pi, lbtn,pulses_per_rev=ticksperRev,min_RPM=0.0)
HallR=pi.callback(rbtn, pigpio.RISING_EDGE, R_Hall)
motorr.RPM = read_RPM.reader(pi, rbtn,pulses_per_rev=ticksperRev,min_RPM=0.0)

motorl.stop()
motorr.stop()
start= time()
tripPath=os.path.abspath(".")+"/trips/"


d=Serdata()
d.error=0
d.stream=""
debug=True

def dprint(msg):
    if debug==True:
        print(msg)
    else:
        return

def storeRPM(links,rechts):
    with open("/run/shm/rpm","w")as f:
        f.write(str(links)+';'+str(rechts))

def getYAW():
    n=0
    oldstream=d.stream
    while True:    
        try:    
            with open("/dev/shm/heading.txt", "r") as f:
                tmp=f.read()
            #print(tmp)    
            tmp=json.loads(tmp)
            #tmp=s["YAW"]
            #print(tmp)
            d.stream=tmp
            break        
        except:
            n+=1
            print("Fehler beim lesen von YAW")
            if n>15:
                tmp=None
                d.stream=oldstream
                break         
    return tmp

def stop():
    #print("Stop")
    motorl.stop()
    motorr.stop()
    storeRPM(0,0)

def readvalue(para,oldspeed):   
    if para=="r":
        strfile='/run/shm/.PiMowBot_Motor.right'
    if para=="l":
        strfile='/run/shm/.PiMowBot_Motor.left'
    if os.path.isfile(strfile):
         with open (strfile,"r") as File:
            value=File.read()
         try:
            value = float(value)
         except:
            value = oldspeed
    else:
       value=0
    return value

def value2rpm(value):
    # war eigentlich zur Umwandlung der values aus den Motorsteuerungsdateien gedacht
    # da sollten dann die SOLL RPM herauskommen. das ist aber für die Handsteuerung zu indirekt    
    # Nun werden die Werte der PiMowbot Steuerdatei in den maxwert der für die pwm steuerung umgewandelt 
    rpm_min= 0
    rpm_max= 245
    val_min= 10
    val_max=99
    if value < 10:
        rpm=0
    else:
        rpm=(value-val_min)*(rpm_max-rpm_min)/(val_max-val_min)+rpm_min   
    #print(value , " gemappt =", int(rpm))    
    return int(rpm)
    

def drive(rpml,rpmr,argu1,argu2):
    if os.path.isfile("/run/shm/stop"):
        stop()
        return -999
            
    rotate=False          # Schalter vor der Auswertung von argu2 auf False gesetzt
    straight=False        # Schalter vor der Auswertung von argu2 auf False gesetzt
    cm=False              # Schalter vor der Auswertung von argu2 auf False gesetzt
    motorl.gesamtTic=0
    motorr.gesamtTic=0
    motorl.targetTics=0
    motorr.targetTics=0
    motorr.pwmValue=1
    motorl.pwmValue=1 
    motorl.pwmoldValue=0
    motorr.pwmoldValue=0
    motorr.dir=1
    motorl.dir=1
    correction=0
    pidl = PID(Kp=0.8, Ki=0.1, Kd=0.9)
    pidr = PID(Kp=0.8, Ki=0.1, Kd=0.9)
    pidyaw = YAWPID(Kp=0.5, Ki=0.2, Kd=0.2)
    dt = 0.1
    m=0
    br=0

    if abs(rpml)==0:
        motorl.targetTics=0
    if rpml<0:
        motorl.dir=-1
    if abs(rpmr)==0:
        motorr.targetTics=0    
    if rpmr<0:
        motorr.dir=-1
    
      
    
    if argu2[0]=="r":
        rotate=True
        target=int(argu1)
        if target > 0:
            motorl.dir=1
            motorr.dir=-1
        if target < 0:
            motorl.dir=-1
            motorr.dir=1
        #actYAW = gyroheading - offset
        actYAW =0
    
    elif argu2.startswith("s") and rpml > 0:
        straight=True
        rpmr=rpml
        tics = int((int(argu1) / (wheeldiameter * PI)) * ticksperRev)
        motorl.targetTics=tics
        motorr.targetTics=tics

    elif argu2[0]=="c":
        cm=True
        tics = int((int(argu1) / (wheeldiameter * PI)) * ticksperRev)
        motorl.targetTics=tics
        motorr.targetTics=tics
    else:
        motorl.targetTics=int(argu1)
        motorr.targetTics=int(argu2)
      
    getYAW()
    offset=d.stream["YAW"]
    gyroheading=offset
               
    
    if straight == True:
        _tmp=argu2.replace("s","")
        if len(_tmp)>0:
            straightdirection= float(_tmp)
        else:
            straightdirection= 0
        if os.path.isfile("/run/shm/gyroOffset"):
            with open("/run/shm/gyroOffset","r",)as f:                              
                offset=round(float(f.read()),2)                                   
                _offset= offset 
            if offset < 0:                                                         
                offset=offset+straightdirection
            elif offset >=0:
                offset=offset+straightdirection
        if straightdirection ==0:
             getYAW()
             offset=d.stream["YAW"]
             _offset= offset
             dprint("Soll Richtung auf dem Hinweg " +str(_offset)+ " und Sollrichtung nach dem Wendemannöver "+ str(round(offset,2)))
        
        if offset > 180:
             offset= offset-360
        elif offset < -180:
             offset= offset+360    
        with open("/run/shm/gyroOffset","w",)as f:
            f.write(str(round(offset,2)))
        
        dprint("File gyroOffset erstellt mit der aktuellen Ausrichtung vom Gyro: "+ str(round(offset,2)))
        error=offset-gyroheading
        dprint("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")         #controll ausgabe um zu sehen ob die Berechnung des
        dprint ("Error vor Korrektur " +str(round(error,2)))                                 # neuen Winkels richtig durchgeführt wurde 
        if error < -180:
            error=error + 360
        elif error > 180:
            error= error - 360
        dprint ("Error nach Korrektur " +str(round(error,2)))
        dprint("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")         # ende  Kontroll Ausgabe
         
        if abs(error) >=30:  # ist er Fehler größer 30 dann ist was falsch gelaufen
            dprint("Da error größer 30 ist, wird der offset zwangsweise auf Heading gesetzt")
            offset=gyroheading
        getYAW()            
        dprint("Offset aus File = "+str(round(offset,2)) + "\t gyroheading=" + str(d.stream["YAW"]))
        dprint("Starte nun nach dem Wendemannöver in Sollrichtung " + str(offset) +  " derzeitige Fehlausrichtung: " +str(round(error,2)))  
    else:
        getYAW()    
        #print("offset = ",offset, "\t gyroheading=",d.stream["YAW"])    

    motorl.SollRPM=abs(rpml)
    motorr.SollRPM=abs(rpmr)
    cancel=False
    done=True
    i=0    
    errorLast=0
    olddeltatime=time()
    motorl.MStop = False
    motorr.MStop = False
    timestamp=time()
              
    while motorl.MStop == False and motorr.MStop == False:  # time()-start < 30:
        sleep(dt)
        ### Sobald die Stop Datei existiert muss angehalten werden und die Drivechain gelöscht werden
        if os.path.isfile("/run/shm/stop"):
           print("Stop")
           #os.remove("/run/shm/stop") 
           stop()
           if os.path.isfile("/run/shm/drivechain"):
                os.remove("/run/shm/drivechain")
           return -999
           
        if straight== True:
            getYAW()            
            correction = pidyaw.update(0,d.stream["YAW"]-offset,dt)
            dprint("Fahre in aktueller Richtung: " +str(d.stream["YAW"]) + "\t Error= " +str(round(pidyaw.last_error,2)) + "\t YAW PID Korrektur=" +str(round(correction,2)))
                        
        if rotate == True:
            getYAW()
            gyroheading=d.stream["YAW"]
            if d.stream["YAW"] < 0:
                gyroheading= d.stream["YAW"]+360;         # Korrektur für den Bereich von 6:00 - 24:00 Uhr
            actYAW=round(gyroheading - offset,0)
            if actYAW > 180:
                actYAW=actYAW-360
            if actYAW < 0:
                actYAW=actYAW+360     
            #print(d.stream["YAW"], " Heading= ", gyroheading, " ActHeading= ", round(actYAW,1), " Target ",target, " noch " , round(target-actYAW,0)) 
                 
        links= int(motorl.RPM.RPM())
        rechts= int(motorr.RPM.RPM())
        errorL=motorl.SollRPM+ round(correction,1) - links
        errorR=motorr.SollRPM- round(correction,1) - rechts
        storeRPM(links,rechts)
              
        
        if rotate == True:
            #float AngleDifference(float a, float b) {
            #return (a - b + 540) % 360 - 180
            deltatime=time()-olddeltatime
            error= (target-actYAW +540)%360 - 180
            errordelta=(error-errorLast+540)%360-180
            #errorLast = error
            errorRateofChange=errordelta/deltatime
                        
            
            if abs(error)<40:
                motorl.SollRPM=9
                motorr.SollRPM=9
            if abs(error)<18:
                motorl.SollRPM=7
                motorr.SollRPM=7         
                     
            
            motorl.pwmValue = pidl.update(errorL,links,  dt, motorl.MaxSpeed)
            motorr.pwmValue = pidr.update(errorR,rechts, dt, motorr.MaxSpeed)
            getYAW()            
            #dprint("error:" +str(round(error,2)) + "\t Errorlast: " + str(errorLast) + "\t TimeDelta: " + str(round(time()-timestamp,1)) + "\t Errordelta: " + str(errordelta) + "\t ErrorchangeRate= "+ str(errorRateofChange) + "\t Richtung: " + str(d.stream["YAW"]) + "\t RPML= " + str(links) + "\t motorl.SollRPM= " + str( motorl.SollRPM)) 
            
                            
            if errorLast < 0 and error >= 0 and time()-timestamp > 1.5:
                d.targeterror=actYAW-target
                getYAW()
                dprint("errorlast < 0 and error >= 0 Richtung: " + str(d.stream["YAW"]))
                stop()
                motorr.MStop=True
                motorl.MStop=True
                break
            if error <= 0 and errorLast > 0 and time()-timestamp > 1.5:
                d.targeterror=actYAW-target
                getYAW()
                dprint("error <= 0 and errorlast > 0 Richtung: " + str(d.stream["YAW"]))
                stop()
                motorr.MStop=True
                motorl.MStop=True
                break     
            
            if abs(error) < 2  :          #  if target > 0 and actYAW >= target:
                d.targeterror=actYAW-target
                #print("newdirektion = 0 error = ", actYAW-target, "GyroHeading ", gyroheading)
                getYAW()   
                dprint("Fertig abs(errordelta) < 2  Richtung: " + str(d.stream["YAW"]))
                stop()
                motorr.MStop=True
                motorl.MStop=True
            
            errorLast = error
            olddeltatime=time()

        else:
            motorl.pwmValue = pidl.update(errorL,links,  dt, motorl.MaxSpeed)
            motorr.pwmValue = pidr.update(errorR,rechts, dt, motorr.MaxSpeed)
        
            if motorr.gesamtTic >= motorr.targetTics: #880/2: #setRev * ticksperRev:
                motorr.pwmValue=0
                motorr.pwmoldValue=0
                #print("Rechtsstop")
                stop()
                motorr.MStop=True
                motorl.MStop=True
                motorl.gesamtTic=motorl.targetTics+1 
                # gl=motorl.targetTics+1
                break
            
            if motorl.gesamtTic >= motorl.targetTics: #880/2: #setRev * ticksperRev:
                motorl.pwmValue=0
                motorl.pwmoldValue=0
                #print("Linksstop")
                stop()
                motorl.MStop=True
                motorr.MStop=True
                motorr.gesamtTic=motorr.targetTics+1
                #gr=motorr.targetTics+1
                break
        
        
        getYAW() 
        ausgabe= str(round(time()-start,2))+"\t"+ str(round(d.stream["YAW"]-offset,1))+"\t"+ str(round(correction,1))+"\t"+str(round(motorl.SollRPM+correction,1))+"\t"+ str(links)+"\t"+str(round(motorl.pwmValue,0))
        ausgabe=ausgabe+"\t"+   str(round(motorr.SollRPM-correction,1)) +"\t"+ str(rechts)+"\t"+str(round(motorr.pwmValue,0))+ "\t Tics: "+str(motorl.gesamtTic) +" "+ str(motorr.gesamtTic)  #, ";", rechts, "; Ticsl ;", gl, "; Ticsr ;", gr)  
        ausgabe=ausgabe.replace(".",",")        
        #print(ausgabe)        
        try:
            if motorl.pwmValue != motorl.pwmoldValue:
                motorl.speed(int(motorl.pwmValue*motorl.dir))    
                motorl.pwmoldValue=motorl.pwmValue
            if motorr.pwmValue != motorr.pwmoldValue:
                motorr.speed(int(motorr.pwmValue*motorr.dir))
                motorr.pwmoldValue=motorr.pwmValue
        except:
            print(motorl.pwmValue, "  ",motorr.pwmValue)
            done=False
            cancel=True
    
    return done       


def readfromfile(path, file):
    erg=True
    if os.path.isfile("/run/shm/ready"):
           os.remove("/run/shm/ready")        
    try:
         
       with open(path+file,"r") as f:      #with open("/home/pi/RopiLawnMow/trips/cw_change","r") as f:    
           m=[_.rstrip('\n') for _ in f]
           
       for zeile in m:
            if os.path.isfile("/run/shm/stop"):
                print("Stop")
                stop()
                os.remove("/run/shm/stop")
                os.remove("/run/shm/drivechain")
                break
            if zeile.startswith("#")==False:
                print("Aktueller Fahrbefehl: " + zeile)
            if zeile.count(",")==3 and zeile.startswith("#")==False:
                strdata=zeile.split(",")
                erg=drive(int(strdata[0]),int(strdata[1]),int(strdata[2]),strdata[3])  #erg=drive(-12,12,300,300) für zurück mit 12 RPML, vor mit 12 RPMR, jewils mit 300 tics
                print("Abgeschlossen bei GyroAusrichtung: "+ str(round(d.stream["YAW"],1)))                    
                #if erg==-999:
                #    if os.path.isfile("/run/shm/drivechain"):
                #       os.remove("/run/shm/drivechain")
                #    if os.path.isfile("/run/shm/stop"):
                #       os.remove("/run/shm/stop")
                #    break   
                                    
                if erg == False:
                    return# raise "Fehler"
            elif zeile.count(",")== 0 and zeile.startswith("#")==False:
                dprint("Fertig")                
            else:
                if zeile.startswith("#"):
                    print("Kommentar: " + zeile)
                else:
                    print("Fehler in : " + zeile)
            #print("Sleep ",time())            
            sleep(0.5)
            #print("Wakeup ",time())
       # hier wird nun die Datei gelöscht in der die Befehlsabfolge für die Driveprozedur stand        
       if os.path.isfile("/run/shm/drivechain"):
           os.remove("/run/shm/drivechain")
       with open("/run/shm/ready","w") as f:
            f.write("") 
        
    except OSError:
        print("[ MotorControl Management: MotorControl interupted Tripfile not found or not specified ]")
        return -1        
    except KeyboardInterrupt:
        print("[ MotorControl Management: MotorControl interupted... ]")
        stop()
        



def decode_Parameter(argu):
        print(argu)       
        file = argu[1]
        if os.path.exists(tripPath+file)==True and len(argu)==2:
            readfromfile(tripPath,file)
        elif len(argu)==2 and os.path.exists(tripPath+file)==False:
            print("Datei " + argu[1] + " ist nicht vorhanden")
            return
        else:
            drive(int(argu[1]),int(argu[2]),int(argu[3]),argu[4])            
            #print("Fehler")
            return       
                    
        
if __name__ =="__main__":
    
    
    try:
        Fehler=False
        start=time()  
        if len(sys.argv)<2:
            print("Es wurden keine Parameter übergeben")
            #Fehler = True
            #print("Fehler ", Fehler)
        else:        
            decode_Parameter(sys.argv)
         
        while True:
           if os.path.isfile("/run/shm/drivechain"):
                with open("/run/shm/drivechain","r") as f:
                    File= f.read()
                    File= File.rstrip("\n")
                    print("File=" + tripPath + File)
                if os.path.isfile(tripPath + File):
                    if os.path.isfile("/run/shm/stop"):
                        os.remove("/run/shm/stop")
                    print("rufe nun die Datei auf: "+ tripPath + File)
                    readfromfile(tripPath,File)
                else:
                    print(tripPath + File + " existiert nicht" )
                    if os.path.isfile("/run/shm/drivechain"):
                        os.remove("/run/shm/drivechain") 
           
           else:   # Darf nur funktionieren wenn es keine drivechain Datei gibt.     
                          
               motorl.sollspeed= readvalue("l",motorl.oldspeed)               
               motorr.sollspeed= readvalue("r",motorr.oldspeed) 
               if motorl.sollspeed != motorl.oldspeed or motorr.sollspeed != motorr.oldspeed: 
                   motorl.pwmValue=value2rpm(abs(motorl.sollspeed))
                   motorr.pwmValue=value2rpm(abs(motorr.sollspeed))
                   motorl.oldspeed=motorl.sollspeed
                   motorr.oldspeed=motorr.sollspeed
                   motorl.dir=motorl.getdirection(motorl.sollspeed)
                   motorr.dir=motorr.getdirection(motorr.sollspeed)                    
                   
                   if abs(motorl.sollspeed)< 5:
                        motorl.pwmValue=0
                   if abs(motorr.sollspeed)< 5:
                        motorr.pwmValue=0
                               
                   try:
                    if motorl.pwmValue != motorl.pwmoldValue:
                        motorl.speed(int(motorl.pwmValue*motorl.dir))    
                        motorl.pwmoldValue=motorl.pwmValue
                    if motorr.pwmValue != motorr.pwmoldValue:
                        motorr.speed(int(motorr.pwmValue*motorr.dir))
                        motorr.pwmoldValue=motorr.pwmValue
                   except:
                    print(motorl.pwmValue, "  ",motorr.pwmValue)
           sleep(0.2)      
    
    except KeyboardInterrupt:
        print("[ MotorControl Management: MotorControl interupted... ]")
    finally:
        print("[ MotorControl Management: MotorControl finished !!! ]")
        stop()
        HallL.cancel()
        HallR.cancel()
        motorl.RPM.cancel()
        motorr.RPM.cancel()
        pi.stop()    

