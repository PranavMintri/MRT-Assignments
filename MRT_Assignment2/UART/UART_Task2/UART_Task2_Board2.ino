#include <SoftwareSerial.h>
SoftwareSerial Serial1(11, 10); // RX, TX

void setup() {
  Serial1.begin(9600);
}

void loop() {
  if (Serial1.available() > 0) {
    String msg2 = Serial1.readStringUntil('\n');
    int xc = msg2.toInt();
    int xc2 = xc * xc;
    Serial1.println(xc2);          // Send square back
  }
}
