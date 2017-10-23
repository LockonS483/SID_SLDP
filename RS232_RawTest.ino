#include <SoftwareSerial.h>

SoftwareSerial sSerial(10, 11); // RX, TX

void setup() {
  // put your setup code here, to run once:
  pinMode(8, OUTPUT);
  Serial.begin(9600);

  // set the data rate for the SoftwareSerial port
  sSerial.begin(9600);
  Serial.println("Setup Complete.");
}

void loop() {
  /*if(digitalRead(8)==1){
    digitalWrite(8, LOW);
  }else{
    digitalWrite(8, HIGH);
  }*/


  while (sSerial.available()) {
    Serial.write(sSerial.read());
  }
  
  Serial.println("Read Complete");

  digitalWrite(8, LOW);
  //Serial.println(digitalRead(8));
  //Serial.write("Test");
  delay(500);
  digitalWrite(8,HIGH);
  delay(500);
}
