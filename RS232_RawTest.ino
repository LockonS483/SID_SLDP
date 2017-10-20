#include <SoftwareSerial.h>

SoftwareSerial sSerial(10, 11); // RX, TX

void setup() {
  // put your setup code here, to run once:
  pinMode(8, OUTPUT);
  Serial.begin(9600);

  // set the data rate for the SoftwareSerial port
  sSerial.begin(115200);
  Serial.println("Setup Complete.");
}

void loop() {
  /*if(digitalRead(8)==1){
    digitalWrite(8, LOW);
  }else{
    digitalWrite(8, HIGH);
  }*/

  if (sSerial.available()) {
    Serial.write(sSerial.read());
  }else{
    Serial.println("sSerial:Nothing to Read");
  }
  
  digitalWrite(8, LOW);
  Serial.println(digitalRead(8));
  delay(500);
}
