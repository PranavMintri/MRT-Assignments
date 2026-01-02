#include <Wire.h>

int32_t b = 0, d = 0, aplusb = 0, cplusd = 0, a = 0, c = 0;
bool goBack = false;

void setup() {
  Serial.begin(9600);
  Wire.begin(0x07);                 // Slave 2 address
  Wire.onReceive(receiveEvent);     // Register receive handler
  Wire.onRequest(requestEvent);     // Register request handler
}

void receiveEvent(int howMany) {
  if (howMany >= 8) { // expecting 8 bytes: aplusb and cplusd
    byte b0AB = Wire.read();
    byte b1AB = Wire.read();
    byte b2AB = Wire.read();
    byte b3AB = Wire.read();
    aplusb = ((int32_t)b0AB << 24) | ((int32_t)b1AB << 16) | ((int32_t)b2AB << 8) | (int32_t)b3AB;

    byte b0CD = Wire.read();
    byte b1CD = Wire.read();
    byte b2CD = Wire.read();
    byte b3CD = Wire.read();
    cplusd = ((int32_t)b0CD << 24) | ((int32_t)b1CD << 16) | ((int32_t)b2CD << 8) | (int32_t)b3CD;

    goBack = true;
  }
}

void requestEvent() {
  // Send b and d when master requests
  byte b0B = (b >> 24) & 0xFF;
  byte b1B = (b >> 16) & 0xFF;
  byte b2B = (b >> 8) & 0xFF;
  byte b3B = b & 0xFF;
  Wire.write(b0B); Wire.write(b1B); Wire.write(b2B); Wire.write(b3B);

  byte b0D = (d >> 24) & 0xFF;
  byte b1D = (d >> 16) & 0xFF;
  byte b2D = (d >> 8) & 0xFF;
  byte b3D = d & 0xFF;
  Wire.write(b0D); Wire.write(b1D); Wire.write(b2D); Wire.write(b3D);
}

void loop() {
  if (goBack) {
    a = aplusb - b;
    c = cplusd - d;
    Serial.print("a = "); Serial.print(a);
    Serial.print(", b = "); Serial.print(b);
    Serial.print(", c = "); Serial.print(c);
    Serial.print(", d = "); Serial.println(d);
    goBack = false;
  }
}