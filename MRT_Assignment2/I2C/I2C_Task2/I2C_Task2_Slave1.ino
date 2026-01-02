#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Wire.begin(4);                // join I2C bus as slave with address 4
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
    byte b0B = Wire.read();
    byte b1B = Wire.read();
    byte b2B = Wire.read();
    byte b3B = Wire.read();
    int32_t value2 = ((int32_t)b0B << 24) |
                     ((int32_t)b1B << 16) |
                     ((int32_t)b2B << 8)  |
                     (int32_t)b3B;
    Serial.println(value2); // print the integer received from master
  }
}