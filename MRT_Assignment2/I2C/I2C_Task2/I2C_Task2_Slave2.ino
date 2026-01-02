#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Wire.begin(6);                // join I2C bus as slave with address 6
  Wire.onRequest(requestEvent); // callback when master requests data
  Wire.onReceive(receiveEvent); // callback when master sends data
}

void loop() {
  delay(500);
}

// Called when master requests data
void requestEvent() {
  int32_t value = 1234; // example value to send
  byte b0 = (value >> 24) & 0xFF;
  byte b1 = (value >> 16) & 0xFF;
  byte b2 = (value >> 8) & 0xFF;
  byte b3 = value & 0xFF;
  Wire.write(b0);
  Wire.write(b1);
  Wire.write(b2);
  Wire.write(b3);
}

// Called when master sends data
void receiveEvent(int howMany) {
  if (howMany >= 4) {
    byte b0A = Wire.read();
    byte b1A = Wire.read();
    byte b2A = Wire.read();
    byte b3A = Wire.read();
    int32_t value1 = ((int32_t)b0A << 24) |
                     ((int32_t)b1A << 16) |
                     ((int32_t)b2A << 8)  |
                     (int32_t)b3A;
    Serial.println(value1); // print the integer received from master
  }
}