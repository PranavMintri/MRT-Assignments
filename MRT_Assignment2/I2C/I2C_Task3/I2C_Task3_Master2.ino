#include<Wire.h>
bool roleChangeRequested=false;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(0x05);
  Wire.onReceive(receiveEvent);
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

  if(roleChangeRequested) {
    while(digitalRead(SDA_PIN)==LOW || digitalRead(SCL_PIN) == LOW) {}
    Wire.end();
    Wire.begin();
    Wire.requestFrom(0x06, 4);
    int32_t c = 0, d = 0;
    byte b0C, b1C, b2C, b3C, b0D, b1D, b2D, b3D;
    if(Wire.available()>=4) {
      b0C = Wire.read();
      b1C = Wire.read();
      b2C = Wire.read();
      b3C = Wire.read();
      c = ((int32_t)b0C << 24) |
                    ((int32_t)b1C << 16) |
                    ((int32_t)b2C << 8)  |
                    (int32_t)b3C;
    }
    Wire.requestFrom(0x07, 4);
    if(Wire.available()>=4) {
      b0D = Wire.read();
      b1D = Wire.read();
      b2D = Wire.read();
      b3D = Wire.read();
      d = ((int32_t)b0D << 24) |
                    ((int32_t)b1D << 16) |
                    ((int32_t)b2D << 8)  |
                    (int32_t)b3D;
    }
    int32_t cplusd = c+d;
    byte b0 = (cplusd >> 24) & 0xFF;
    byte b1 = (cplusd >> 16) & 0xFF;
    byte b2 = (cplusd >> 8) & 0xFF;
    byte b3 = cplusd & 0xFF;
    Wire.beginTransmission(0x00);
    Wire.write(b0);
    Wire.write(b1);
    Wire.write(b2);
    Wire.write(b3);
    Wire.endTransmission();
    roleChangeRequested=false;
    Wire.beginTransmission(0x04);
    Wire.write('s');
    Wire.endTransmission();
    Wire.end();
    Wire.begin(0x05);
  }

}
