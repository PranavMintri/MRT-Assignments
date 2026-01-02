#include<SPI.h>
byte integerReader[4];
int index=0;
int receivedNumber = 0;
byte replyBytes[4];
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(MISO, OUTPUT);
  SPI.begin();
  SPCR |= _BV(SPE); // Enable SPI in slave mode
  // SPCR = SPI Control Register (controls the configuration of the SPI hardware)
  // SPE = SPI Enable bit inside SPCR (when set to 1, it turns on the SPI hardware, and when cleared (0), SPI is disabled)
  // _BV(x) = Bit Value
  // This statement sets the SPE bit to 1, without changing the other bits, anmd enables the SPI hardware.
  SPI.attachInterrupt();
}
ISR(SPI_STC_vect) {
  integerReader[index]=SPDR;
  SPDR = replyBytes[index % 4];
  index++;
  if(index==4) {
    receivedNumber = ((int32_t)integerReader[0] << 24) |
                     ((int32_t)integerReader[1] << 16) |
                     ((int32_t)integerReader[2] << 8)  |
                     (int32_t)integerReader[3];
    receivedNumber = receivedNumber + 1;
    index=0;
    replyBytes[0] = (receivedNumber >> 24) & 0xFF;
    replyBytes[1] = (receivedNumber >> 16) & 0xFF;
    replyBytes[2] = (receivedNumber >> 8)  & 0xFF;
    replyBytes[3] = receivedNumber & 0xFF;

    // Load the corresponding reply byte into SPDR
  }
}
void loop() {
  // put your main code here, to run repeatedly:

}
