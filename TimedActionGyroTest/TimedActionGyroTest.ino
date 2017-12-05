#include <TimedAction.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define angleEpsilon 0.8

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


//------------------------VARIABLES-----------------------//

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];       // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//TIMEDACTION
void RobotMove();
void UpdateGyro();
TimedAction gyroCheck = TimedAction(10,UpdateGyro);
TimedAction robotMove = TimedAction(100, RobotMove);
int robotStep;

//Robot Control Variables
bool robotReady;
bool turning;
bool moving;
float targetAngle;
Servo leftServo;
Servo rightServo;
unsigned long moveTime = 0;
float startypr[3] = {0,0,0};
int moveTimes[2] = {0, 0};

//BT and BC
const int triggerPin = 8;
SoftwareSerial btSerial(4, 2); // RX, TX
SoftwareSerial bcSerial(10, 11); // RX, TX


//Barcode and Message Variables
byte tempMessage[22];
int actualSize;

void setup() {
  //Initialize BC and BT Serials//
  ////////////////////////////////
  //Initialize Trigger Pin.
  pinMode(triggerPin, OUTPUT);
  //Floating high, pull low for trigger.
  digitalWrite(triggerPin, HIGH);
  
  // Open serial communications:
  Serial.begin(9600);
  Serial.println("Input AT Commands.");

  // Initializing Software Serial Ports.
  btSerial.begin(9600);
  bcSerial.begin(9600);
  ////////////////////////////////

  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  // initialize serial communication
   Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
    
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  /*
  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  */
  while (Serial.available() && Serial.read()); // empty buffer (again)

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // Gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(20700);
  mpu.setYGyroOffset(5682);
  mpu.setZGyroOffset(-2430);
  mpu.setZAccelOffset(1788); // 1688 factory default for test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

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

  //Initialize Robot Components
  leftServo.attach(9);
  rightServo.attach(6);
  resetServos();
  robotReady = true;

  Serial.println(moveTime);
  Serial.println(moveTimes[0]);
}

void loop() {
  gyroCheck.check();
  robotMove.check();
}

void UpdateGyro(){
  //Return if programming failed
  if (!dmpReady) return;

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  //Serial.println(fifoCount);
  // check for overflow (this should never happen unless our code is too inefficient)
  if (fifoCount >= 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
        
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("y: ");
    Serial.println(YawAngle(getYaw(ypr)));
}

/*void RobotMove(){
  if(robotStep == 0){
      forward(-1, 0, -8000, true);
  }else{
    resetServos();
  }
  if(robotReady){
    robotStep ++;
  }
    
}*/

void RobotMove(){
  switch (robotStep){
    //Initializing Readings
    case 0:
      resetServos();
      robotReady = true;
      moving = false;
      robotMove.setInterval(30000);
      break;
    case 1:
      startypr[0] = ypr[0];
      startypr[1] = ypr[1];
      startypr[2] = ypr[2];
      robotReady = true;
      robotMove.setInterval(500);
      Serial.println("///////////////" + String(YawAngle(getYaw(startypr))));
      moveTime = millis();
      break;
      
    case 2:
      //forward(-1, 0.7, 30000, true);
      forward(1, 0.7, 3750, true);
      break;
      
    case 3:
      turn(-90, -1);
      break;
      
    case 4:
      forward(1, -89, 3700, true);
      break;
      
    case 5:
      resetServos();
      TriggerBarcode();
      robotMove.setInterval(1000);
      break;
    case 6:
      CheckBT();
      robotMove.setInterval(500);
      resetServos();
      break;

    case 7:
      forward(-1, -90, 880, false);
      break;
    case 8:
      turn(180, -1);
      break;
    case 9:
      forward(1, 182, 2200, false);
      break;

    case 10:
      turn(-90, 1);
      break;

    case 11:
      forward(1, -90, 1200, false);
      break;  

    case 12:
      turn(180, -1);
      break;

    //RESET GYRO READINGS
    case 13:
      forward(-1, 180, 1000, false);
      break;
    case 14:
      startypr[0] = ypr[0];
      startypr[1] = ypr[1];
      startypr[2] = ypr[2];
      robotReady = true;
      robotMove.setInterval(500);
      Serial.println("///////////////" + String(YawAngle(getYaw(startypr))));
      break;

    case 15:
      forward(1, 0, 900, false);
      break;

    case 16:
      turn(-90, -1);
      break;

    case 17:
      resetServos();
      TriggerBarcode();
      robotMove.setInterval(1000);
      break;
    case 18:
      CheckBT();
      robotMove.setInterval(500);
      resetServos();
      break;
    
    case 19:
      turn(0, 1);
      break;

    case 20:
      forward(-1, 0, 950, false);
      break;

    case 21:
      turn(-90, -1);
      break;

    case 22:
      resetServos();
      TriggerBarcode();
      robotMove.setInterval(1000);
      break;
    case 23:
      CheckBT();
      robotMove.setInterval(500);
      resetServos();
      break;
    
    default:
      Serial.println("DEFAULT");
      break;
  }
  if(robotReady){
    robotStep++;
  }
}


float getYaw(float yprr[]){
  float temp = yprr[0] * 180/M_PI;
  return temp + 180;
}

float YawAngle(float ang){
  if(ang > 360){
    ang -= 360;
  }else if(ang < 0){
    ang += 360;
  }
  return ang;
}


/////////////////////////////
///Robot Control Functions///
/////////////////////////////
void resetServos(){
  rightServo.write(89);
  leftServo.write(89.7);
}

void forward(int dir, float rotation, unsigned long timeF, boolean offset){
  if(!moving){
    targetAngle = YawAngle(YawAngle(getYaw(startypr)) + rotation);
    Serial.println("//////Forward at: " + String(targetAngle));
    robotMove.setInterval(5);
    if(offset){
      leftServo.write(90 + ((20)*dir));
      rightServo.write(90 - ((90)*dir));
    }else{
      leftServo.write(90 + ((90)*dir));
      rightServo.write(90 - ((88.5)*dir));
    }
    
    
    Serial.println("TimeF: " + String(timeF));
    Serial.println(millis());
    moveTime = millis() + timeF;
    Serial.println(moveTime);
    
    Serial.println("The Time is " + String(millis()) + ", stopping at " + (moveTime));
    robotReady = false;
    moving = true;
  }else{
    Serial.print("moving | "); Serial.print(millis()); Serial.print(" | "); Serial.println(moveTime);
    if(millis() > moveTime){
      resetServos();
      robotReady = true;
      moving = false;
    }
    if(abs((YawAngle(getYaw(ypr)) - targetAngle)) > 0.4){
      Serial.println("Compensating for crooked movement.");
      if(dir > 0){
        if(YawAngle(getYaw(ypr)) > targetAngle){
          Serial.println("Moving Left");
          leftServo.write(90 + ((25)*dir));
          rightServo.write(90 - ((90)*dir));
        }else{
          Serial.println("Moving Right");
          leftServo.write(90 + ((90)*dir));
          rightServo.write(90 - ((25)*dir));
        }
      }else{
        if(YawAngle(getYaw(ypr)) > targetAngle){
          Serial.println("Moving Left");
          leftServo.write(90 + ((90)*dir));
          rightServo.write(90 - ((25)*dir));
        }else{
          Serial.println("Moving Right");
          leftServo.write(90 + ((20)*dir));
          rightServo.write(90 - ((95)*dir));
        }
      }
    }
  }
}

void turn(float turnTarget, int dir){
  if(!turning){
    targetAngle = YawAngle(YawAngle(getYaw(startypr)) + turnTarget);
    Serial.println("//////////Turning to" + String(targetAngle));
    robotMove.setInterval(5);
    if(dir == 0){
      if(YawAngle(getYaw(ypr)) - targetAngle > 0){
        dir = -1;
      }else{
        dir = 1;
      }
    }
    if(dir != 0){
      rightServo.write(90 + (6*dir));
      leftServo.write(90 + (6*dir));
      robotReady = false;
      turning = true;
    }else{
      if(abs((YawAngle(getYaw(ypr)) - targetAngle)) > 5){
        if(dir == 1){
          rightServo.write(90 + (4*dir));
          leftServo.write(90 + (10*dir));
        }else{
          rightServo.write(90 + (10*dir));
          leftServo.write(90 + (6*dir));
        }
        robotReady = false;
        turning = true;
      }else{
        robotReady = true;
        turning = false;
      }
    }
  }else if(abs((YawAngle(getYaw(ypr)) - targetAngle)) < angleEpsilon){
    robotReady = true;
    turning = false;
    resetServos();
    robotMove.setInterval(500);
  }
}

void checkRotation(float rotation){
  float rotTarget = YawAngle(YawAngle(getYaw(startypr)) + rotation);
  float diff = YawAngle(getYaw(ypr)) - rotTarget;
  robotReady = false;
  robotMove.setInterval(500);
  Serial.println("/////Checked Rotation, Difference = " + String(diff));
  if(abs(diff) > 5){
    int dir = 1;
    if(diff > 0){
      dir = -1;
    }
    turn(rotation, dir);
  }else{
    robotReady = true;
  }
}


/////////////////////////////
////Barcode and Bluetooth////
/////////////////////////////
void TriggerBarcode(){
  //****************
  //TESTING CODE: Triggers scanner
  //****************
  digitalWrite(triggerPin, LOW);
  delay(200);
  digitalWrite(triggerPin, HIGH);
}

boolean CheckBT(){
  int tries = 0;
  //If the barcode scanner sends something, send the message to bluetooth
  while(tries < 5){
    if(bcSerial.available()){
      String barcodeMessage = SerialReadToString();
  
      if(barcodeMessage.length() == 8){
        GenerateMessage(barcodeMessage);
        delay(40);
        SendMessage();
      }
      return true;
    }else{
      delay(200);
      tries++;
    }
  }

  return false;
}

void SendMessage(){
  //Copy temp array to a new array of the right size
  byte messageArray[actualSize];
  for(int i=0; i<actualSize; i++){
    messageArray[i] = tempMessage[i];
    
    Serial.println("Message [" + (String)i + "] is: " + tempMessage[i]);
  }
  //send the array with the right size
  btSerial.write(messageArray, actualSize);
}

////////////////////////////////////
////////////// MSG GEN /////////////
////////////////////////////////////
String SerialReadToString(){
  String outStr;
  /*while (bcSerial.available()){
    outStr += bcSerial.read();
    Serial.println(outStr);
    delay(50);
  }*/
  outStr = bcSerial.readString();
  outStr = outStr.substring(0, 8);

  Serial.println("String read from register: " + outStr);
  return outStr;
}


void GenerateMessage(String val){ //Only up to 8 character messages
  actualSize = 9 + 2 + val.length() + 3; //9 basic bytes + 2 opening bytes + message length + title length
  char charArray[val.length()];
  
  val.toCharArray(charArray, val.length()+1);
  
  //Payload size (2 bytes)
  tempMessage[0] = (byte)actualSize - 2;
  tempMessage[1] = 0x00;
  
  //lego header (4 bytes)
  tempMessage[2] = 0x01;
  tempMessage[3] = 0x00;
  tempMessage[4] = 0x81;
  tempMessage[5] = 0x9E;
  
  //Title size (1 bytes)
  tempMessage[6] = (byte)4;
  
  //Title (4 bytes)
  tempMessage[7] = 0x61;
  tempMessage[8] = 0x62;
  tempMessage[9] = 0x63;
  tempMessage[10] = 0x00;
  
  //Value size (2 bytes)
  tempMessage[11] = (byte)((int)(val.length()+1));
  tempMessage[12] = 0x00;
  
  //Value bytes: (variable size)
  for(int i=0; i<val.length(); i++){
    byte b = (byte)charArray[i];
    tempMessage[13+i] = b;
    Serial.println("tempMessage [" + String((int)(13+i)) + "] is: " + b);
  }
  tempMessage[actualSize-1] = 0x00;
}

