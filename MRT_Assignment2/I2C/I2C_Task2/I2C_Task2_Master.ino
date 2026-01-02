#include<Wire.h>
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.requestFrom(4, 4); // requesting 4 bytes from device #4
  int32_t value1=0;
  byte b0A, b1A, b2A, b3A, b0B, b1B, b2B, b3B;
  if(Wire.available() >= 4) {
    b0A = Wire.read();
    b1A = Wire.read();
    b2A = Wire.read();
    b3A = Wire.read();
    value1 = ((int32_t)b0A << 24) |
                    ((int32_t)b1A << 16) |
                    ((int32_t)b2A << 8)  |
                    (int32_t)b3A;
    Wire.beginTransmission(6);
    Wire.write(b0A);
    Wire.write(b1A);
    Wire.write(b2A);
    Wire.write(b3A);
    Wire.endTransmission();
  }
  Wire.requestFrom(6, 4); // requesting 4 bytes from device #6
  int32_t value2=0;
  if(Wire.available() >= 4) {
    b0B = Wire.read();
    b1B = Wire.read();
    b2B = Wire.read();
    b3B = Wire.read();
    value2 = ((int32_t)b0B << 24) |
                    ((int32_t)b1B << 16) |
                    ((int32_t)b2B << 8)  |
                    (int32_t)b3B;
    Wire.beginTransmission(4);
    Wire.write(b0B);
    Wire.write(b1B);
    Wire.write(b2B);
    Wire.write(b3B);
    Wire.endTransmission();
  }
  int32_t x = value1 + value2;
  int32_t x2 = x*x;
  Serial.println(x2);
  delay(500);
}
