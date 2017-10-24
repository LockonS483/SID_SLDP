#include <SoftwareSerial.h>

const int triggerPin = 8;

SoftwareSerial btSerial(4, 2); // RX, TX
SoftwareSerial bcSerial(10, 11); // RX, TX

String command = ""; // Stores response of the HC-06 Bluetooth device
//byte readIn[];

//0x12 0x00 0x01 0x00 0x81 0x9E 0x05 0x70 0x69 0x6E 0x67 0x00 0x06 0x00 0x68 0x65 0x6C 0x6C 0x6F 0x00
byte messageBytes[] = {0x12, 0x00, 0x01, 0x00, 0x81, 0x9E, 0x05, 0x70, 0x69, 0x6E, 0x67, 0x00, 0x06, 0x00, 0x68, 0x65, 0x6C, 0x6C, 0x6F, 0x00};

byte tempMessage[22];
int actualSize;
//Barcode A - 
//Barcode B - 52421454
//Barcode C - 
//Barcode D - 

void setup() {
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

  delay(150);

  GenerateMessage("52421454");
  delay(50);
  SendMessage();
}

////////////////////////////////////
///////// SOFTSERIAL READ //////////
////////////////////////////////////
String SerialReadtoString(SoftwareSerial reg){
  String outStr;
  while (reg.available()){
    outStr += reg.read();
  }
  Serial.println("String read from register: " + outStr);
  return outStr;
}

////////////////////////////////////
////////////// MSG GEN /////////////
////////////////////////////////////
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


////////////////////////////////////
///////////// MSG SEND /////////////
////////////////////////////////////
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
/////////////// LOOP ///////////////
////////////////////////////////////
void loop() {
  // Read device output if available.
  if (btSerial.available()) {
    while(btSerial.available()) { // While there is more to be read, keep reading.
      command += (byte)btSerial.read();
      command += (" ");
      //Delay cushion for Serial read
      delay(10);
    }

    Serial.println(command);
    command = ""; // No repeats
  }

  // Read user input if available.
  if (Serial.available()){
    delay(10); // The delay is necessary to get this working!
    btSerial.write((byte)Serial.read());
    /*for(int i=0; i<sizeof(messageBytes); i++){
      btSerial.write(messageBytes[i]);
      Serial.print("Send: " + messageBytes[i]);
    }
    Serial.println("wee");
    */
  }
}
