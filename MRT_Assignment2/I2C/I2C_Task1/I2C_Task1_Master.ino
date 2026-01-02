#include<Wire.h>
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(4); // transmits to device #4
  Wire.write("ALL IS WELL");
  Wire.endTransmission();
}
