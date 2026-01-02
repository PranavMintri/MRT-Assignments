#include<Wire.h>
int32_t a = 0, c = 0, aplusb = 0, cplusd = 0, b = 0, d = 0;
bool goBack = false;

void setup() {
  Serial.begin(9600);
  Wire.begin(0x06);                 // Slave 1 address
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
  // Send a and c when master requests
  byte b0A = (a >> 24) & 0xFF;
  byte b1A = (a >> 16) & 0xFF;
  byte b2A = (a >> 8) & 0xFF;
  byte b3A = a & 0xFF;
  Wire.write(b0A); Wire.write(b1A); Wire.write(b2A); Wire.write(b3A);

  byte b0C = (c >> 24) & 0xFF;
  byte b1C = (c >> 16) & 0xFF;
  byte b2C = (c >> 8) & 0xFF;
  byte b3C = c & 0xFF;
  Wire.write(b0C); Wire.write(b1C); Wire.write(b2C); Wire.write(b3C);
}

void loop() {
  if (goBack) {
    b = aplusb - a;
    d = cplusd - c;
    Serial.print("a = "); Serial.print(a);
    Serial.print(", b = "); Serial.print(b);
    Serial.print(", c = "); Serial.print(c);
    Serial.print(", d = "); Serial.println(d);
    goBack = false;
  }
}


