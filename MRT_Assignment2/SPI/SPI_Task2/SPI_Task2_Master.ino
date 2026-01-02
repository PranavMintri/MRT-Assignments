#include<SPI.h>
int SS_A = 10, SS_B = 9;
SPISettings settings(4000000, MSBFIRST, SPI_MODE0);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();
  pinMode(SS_A, OUTPUT);
  pinMode(SS_B, OUTPUT);
  digitalWrite(SS_A, HIGH);
  digitalWrite(SS_B, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()>0) {
    String st = Serial.readStringUntil('\n');
    int32_t x = st.toInt();
    if(x%2==0) {
      SPI.beginTransaction(settings);
      digitalWrite(SS_A, LOW); // Selecting Slave 1
      byte b0 = (x >> 24) & 0xFF;
      byte b1 = (x >> 16) & 0xFF;
      byte b2 = (x >> 8) & 0xFF;
      byte b3 = x & 0xFF;
      byte r0 = SPI.transfer(b0);
      byte r1 = SPI.transfer(b1);
      byte r2 = SPI.transfer(b2);
      byte r3 = SPI.transfer(b3);
      digitalWrite(SS_A, HIGH);
      SPI.endTransaction();
      int32_t numberReceived = ((int32_t)r0 << 24) |
                     ((int32_t)r1 << 16) |
                     ((int32_t)r2 << 8)  |
                     (int32_t)r3;
      Serial.println(numberReceived);
    }
    else {
      SPI.beginTransaction(settings);
      digitalWrite(SS_B, LOW); // Selecting Slave 2
      byte b0 = (x >> 24) & 0xFF;
      byte b1 = (x >> 16) & 0xFF;
      byte b2 = (x >> 8) & 0xFF;
      byte b3 = x & 0xFF;
      byte r0 = SPI.transfer(b0);
      byte r1 = SPI.transfer(b1);
      byte r2 = SPI.transfer(b2);
      byte r3 = SPI.transfer(b3);
      digitalWrite(SS_B, HIGH);
      SPI.endTransaction();
      int32_t numberReceived = ((int32_t)r0 << 24) |
                     ((int32_t)r1 << 16) |
                     ((int32_t)r2 << 8)  |
                     (int32_t)r3;
      Serial.println(numberReceived);
    }
  }

}
