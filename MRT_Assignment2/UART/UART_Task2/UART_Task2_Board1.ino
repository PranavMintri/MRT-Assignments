#include <SoftwareSerial.h>
SoftwareSerial Serial1(11, 10); // RX, TX

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String msg1 = Serial.readStringUntil('\n');
    int x = msg1.toInt();
    int y = x + 1;
    Serial1.println(y);          // Send as text with newline
  }

  if (Serial1.available() > 0) {
    String msg = Serial1.readStringUntil('\n');
    int z = msg.toInt();
    Serial.println(z);           // Print result
  }
}
