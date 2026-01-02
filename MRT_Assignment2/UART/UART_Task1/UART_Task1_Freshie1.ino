String messageToTransmit="";
String messageToReceive="";
#include<SoftwareSerial.h>
SoftwareSerial Serial1(11, 10); //RX, TX
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  // Transmission:
  if(Serial.available()>0) {
    Serial.println("MintTree");
    char nextLetter=Serial.read();
    if(nextLetter == '\n') {
      Serial1.println(messageToTransmit);
      messageToTransmit="";
    }
    else {
      messageToTransmit+=nextLetter;
    }
  }
  // Retrieval:
  if(Serial1.available()>0) {
    Serial.println("SrinathPrabhu");
    char nextLetter=Serial1.read();
    if(nextLetter=='\n') {
      Serial.print(messageToReceive);
      messageToReceive="";
    }
    else {
      messageToReceive+=nextLetter;
    }
  }
  delay(100);
}
