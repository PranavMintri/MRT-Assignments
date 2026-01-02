#include<SPI.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin(); // Start SPI as master.
  pinMode(SS, OUTPUT); // SS pin must be output.
  digitalWrite(SS, HIGH); // Deselect slave.
}

void loop() {
  // put your main code here, to run repeatedly:
  SPISettings myDeviceSettings(4000000, MSBFIRST, SPI_MODE0); // Define settings once
  SPI.beginTransaction(myDeviceSettings);
  digitalWrite(SS, LOW); // Select slave.
  String st = "ALL IS WELL";
  for(int i=0; i<st.length(); i++) {
    SPI.transfer(st[i]);
  }
  digitalWrite(SS, HIGH); // Deselect slave.
  SPI.endTransaction();
  delay(100);
}
