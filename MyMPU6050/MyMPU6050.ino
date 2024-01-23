#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <WebSerial.h>

//#include "MPU6050_6Axis_MotionApps_V6_12.h"

#include "MPU6050_6Axis_MotionApps612.h"
#include "Wire.h"

AsyncWebServer server(80);

const char* ssid = "MySSID"; 
const char* password = "MyPWD"; 

//const uint16_t websockets_server_port = 8080; // Enter server port
const int LED = 2;

MPU6050 mpu;


//TwoWire I2CPi = TwoWire(0);
byte data_to_echo = 0;

#define INTERRUPT_PIN 26  // use pin 26; 2 on Arduino Uno & most boards
#define LED_PIN 12        // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
bool firstRun = true;
unsigned long lastout = 0;
float corYAW = 0.0;
float actYAW = 0.0;
float offset = 0.0;
float YAW = 0.0;
float Roll = 0.0;
float Pitch = 0.0;
float gyroheading = 0.0;
String Ausgabe = "";

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
bool WLAN=false;
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer
int i = 0;


//orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 gy;       // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector

float euler[3];  // [psi, theta, phi]    Euler angle container
float ypr[3];    // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================




volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void WebrecvMsg(uint8_t* data, size_t len) {
  WebSerial.println("Received Data...");
  String d = "";
  for (int i = 0; i < len; i++) {
    d += char(data[i]);
  }

  if (d.startsWith("N")) {
    if (actYAW < 0) {
      corYAW = abs(actYAW);
      WebSerial.print("der aktuelle Kurs=");
      WebSerial.print(actYAW);
      WebSerial.print(" Korrektur Kurs=");
      WebSerial.println(corYAW);
    } else {
      corYAW = abs(actYAW) * (-1);
      WebSerial.print("der aktuelle Kurs=");
      WebSerial.print(actYAW);
      WebSerial.print(" Korrektur Kurs=");
      WebSerial.println(corYAW);
    }
    WebSerial.print("der aktuelle Kurs=");
    WebSerial.print(actYAW);
    WebSerial.print(" Neuer Kurs=");
    WebSerial.println(actYAW + corYAW);
  }
}


void calcnew() {
  if (actYAW < 0) {
    corYAW = abs(actYAW);
    Serial.print("der aktuelle Kurs=");
    Serial.print(actYAW);
    Serial.print(" Korrektur Kurs=");
    Serial.println(corYAW);
  } else {
    corYAW = abs(actYAW) * (-1);
    Serial.print("der aktuelle Kurs=");
    Serial.print(actYAW);
    Serial.print(" Korrektur Kurs=");
    Serial.println(corYAW);
  }
  Serial.print("der aktuelle Kurs=");
  Serial.print(actYAW);
  Serial.print(" Neuer Kurs=");
  Serial.println(actYAW + corYAW);
}
 void wifiSetup(){
  WiFi.mode(WIFI_STA);
  Serial.print("suche "); Serial.println(ssid);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("Das WLAN ist nicht erreichbar");
      
    }else{
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    pinMode(27, OUTPUT);
    digitalWrite(27, HIGH);
    //pinMode(LED, OUTPUT);
    //digitalWrite(LED, HIGH);
    
    // WebSerial is accessible at "<IP Address>/webserial" in browser
    //server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) request->send(200, "text/plain", "Hi! This is a sample response.");
    AsyncElegantOTA.begin(&server);    // Start AsyncElegantOTA
    WebSerial.begin(&server);
    WebSerial.msgCallback(WebrecvMsg);
    server.begin();
    Serial.println(F("HTTP server started"));
    WLAN=true;
    }
 }
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  Wire.begin(21,22);//(21,22); // (16, 17);
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(9600);
  Serial.print("\n");
  
  
 
    delay(200);    
  




// initialize device
Serial.println(F("Initializing I2C devices..."));
mpu.initialize();
//pinMode(INTERRUPT_PIN, INPUT);

// verify connection
Serial.println(F("Testing device connections..."));
Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

// wait for ready
Serial.println(F("\nSend any character to begin DMP programming and demo: "));
// load and configure the DMP
Serial.println(F("Initializing DMP..."));
devStatus = mpu.dmpInitialize();


// Original Offsets
// ermittelte Offsets mit em Zero Sketch
/*
mpu.setXGyroOffset(114);
mpu.setYGyroOffset(37);
mpu.setZGyroOffset(29);
mpu.setXAccelOffset(110);
mpu.setYAccelOffset(-2814);
mpu.setZAccelOffset(1832);
*/




// make sure it worked (returns 0 if so)
if (devStatus == 0) {
  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateAccel(15);   //ermitteln nach 50 loops den offset
  mpu.CalibrateGyro(15);    //ermitteln nach 50 loops den offset 
  Serial.println();
  mpu.PrintActiveOffsets();
  // turn on the DMP, now that it's ready
  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);

  // enable Arduino interrupt detection
  Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
  Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
  Serial.println(F(")..."));
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();

  // set our DMP Ready flag so the main loop() function knows it's okay to use it
  Serial.println(F("DMP ready! Waiting for first interrupt..."));
  dmpReady = true;

  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
} else {
  // ERROR!
  // 1 = initial memory load failed
  // 2 = DMP configuration updates failed
  // (if it's going to break, usually the code will be 1)
  Serial.print(F("DMP Initialization failed (code "));
  Serial.print(devStatus);
  Serial.println(F(")"));
}
// configure LED for output
pinMode(LED, OUTPUT);
pinMode(12,INPUT_PULLUP);
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if ((WLAN==false) and (digitalRead(12) == LOW)){
    Serial.println("####################################     Suche WLAN   ##############################");
    wifiSetup();
  }

  if (Serial.available() > 0) {
    // read the incoming byte:
    byte incomingByte = Serial.read();
    // say what you got:
    //Serial.print("I received: ");
    //Serial.println(incomingByte);
    if (incomingByte == 110) {
      offset = YAW;
      //Serial.print("Correction= ");
      //Serial.println(offset);
      //delay(3);
      //calcnew();
    }
    if (incomingByte == 82) {
      Serial.print("Restart");
      digitalWrite(LED, LOW);
      delay(2000);

      ESP.restart();
    }
  }


  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    YAW = (ypr[0] * 180 / M_PI);  // Wert den der MPU nach initialisierung ausgibt
    gyroheading = YAW;

    if (YAW < 0) {
      gyroheading = YAW + 360;  // Korrektur für den Bereich von 6:00 - 24:00 Uhr
    }
    actYAW = gyroheading - offset;  // Wenn gedreht wurde wird offset (offset=YAW) vom Heading abgezogenund actYaw ist wieder 0
    if (actYAW > 180) {             // Wieder eine Korrektur damit actYAW immer im Bereich von -180 bis 180 bleibt
      actYAW = actYAW - 360;
    }
    Roll = ypr[2] * 180 / M_PI;
    Pitch = ypr[1] * 180 / M_PI;
    Ausgabe = "{'Roll':" + String(Roll) + ",'Pitch':" + String(Pitch) + ",'YAW':" + String(YAW) + "}";

    if (millis() - lastout > 150) {
      Serial.println(Ausgabe);
      lastout = millis();
    }

    //}
    // blink LED to indicate activity
    //blinkState = !blinkState;
    //digitalWrite(LED_PIN, blinkState);
  }
}

void receiveData(int bytecount) {
  for (int i = 0; i < bytecount; i++) {
    data_to_echo = 1;  //Wire.read();
  }
}
void sendData() {
  //Wire.write(data_to_echo);
}
