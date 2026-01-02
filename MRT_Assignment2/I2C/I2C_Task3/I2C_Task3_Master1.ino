#include<Wire.h>
bool roleChangeRequested=false;
int32_t a, b;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Wire.requestFrom(0x06, 4);
  a = 0, b = 0;
  byte b0A, b1A, b2A, b3A, b0B, b1B, b2B, b3B;
  if(Wire.available()>=4) {
    b0A = Wire.read();
    b1A = Wire.read();
    b2A = Wire.read();
    b3A = Wire.read();
    a = ((int32_t)b0A << 24) |
                    ((int32_t)b1A << 16) |
                    ((int32_t)b2A << 8)  |
                    (int32_t)b3A;
  }
  Wire.requestFrom(0x07, 4);
  if(Wire.available()>=4) {
    b0B = Wire.read();
    b1B = Wire.read();
    b2B = Wire.read();
    b3B = Wire.read();
    b = ((int32_t)b0B << 24) |
                    ((int32_t)b1B << 16) |
                    ((int32_t)b2B << 8)  |
                    (int32_t)b3B;
    Wire.beginTransmission(0x05);
    Wire.write('s');
    Wire.endTransmission();
    Wire.end();
    Wire.begin(0x04);
    Wire.onReceive(receiveEvent);
  }
}
void receiveEvent(int howMany) {
  if (Wire.available()) {
    byte control = Wire.read();
    if (control == 's') {   // example control byte
      roleChangeRequested = true;
    }
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  if(roleChangeRequested==true) {
    while(digitalRead(SDA_PIN)==LOW || digitalRead(SCL_PIN) == LOW) {}
    Wire.end();
    Wire.begin();
    Wire.beginTransmission(0x00);
    int32_t aplusb = a+b;
    byte b0 = (aplusb >> 24) & 0xFF;
    byte b1 = (aplusb >> 16) & 0xFF;
    byte b2 = (aplusb >> 8) & 0xFF;
    byte b3 = aplusb & 0xFF;
    Wire.write(b0);
    Wire.write(b1);
    Wire.write(b2);
    Wire.write(b3);
    Wire.endTransmission();
  }
}
