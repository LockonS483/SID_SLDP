#include <TimedAction.h>
// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>

const int MPU_addr=0x68;  // I2C address of the MPU-6050
const int sensitivity = 250;
const int offset = 1880;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float curRotation;

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}

void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  /*AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)*/
  
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.println("GyZ = "); Serial.println(GyZ);
  
  GyZ -= offset;
  //Deg/sec = GyZ / Sensitivity
  float angSpeed = GyZ / sensitivity;
  //deltaTime = 50 (0.05sec)
  curRotation += GyZ * (0.05);
  /*Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);*/
  
  Serial.print("Offset GyZ = "); Serial.println(GyZ);
  Serial.println("Current Rotation (degrees): " + String(curRotation));
  
  delay(50);
}
