#include<SPI.h>
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();
  SPCR |= _BV(SPE); // Enable SPI in slave mode
  // SPCR = SPI Control Register (controls the configuration of the SPI hardware)
  // SPE = SPI Enable bit inside SPCR (when set to 1, it turns on the SPI hardware, and when cleared (0), SPI is disabled)
  // _BV(x) = Bit Value
  // This statement sets the SPE bit to 1, without changing the other bits, anmd enables the SPI hardware.
  SPI.attachInterrupt(); // Enable SPI interrupt
}

ISR(SPI_STC_vect) {
  received = SPDR; // SPDR = SPI Data Register, this gets the received byte.
  Serial.write(received); // Prints character directly.
}

void loop() {
  // put your main code here, to run repeatedly:

}
