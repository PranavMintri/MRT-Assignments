#include<Wire.h>
void setup() {
  // put your setup code here, to run once:
  Wire.begin(4);
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(500);
}
void receiveEvent() {
  String st = "";
  while(Wire.available()>0) {
    char c = Wire.read();
    st = st+c;
  }
  Serial.println(st);
}
