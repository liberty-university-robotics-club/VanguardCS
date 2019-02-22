#include "DualVNH5019MotorShield.h"
#include <SPI.h>

DualVNH5019MotorShield md;

int right = 0;
int left = 0;

//============

void stopIfFault() {
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
    
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

//============

void setup() {
  Serial.begin(115200);
  Serial.println("Motor control - Slave");
  md.init();
  
  
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SS, INPUT);

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // turn on interrupts
  SPCR |= _BV(SPIE);

}

//============

// SPI interrupt routine
ISR (SPI_STC_vect) {
  
  int right = SPDR;
  int left = SPDR;

  md.setM1Speed(right);
  md.setM2Speed(left);
  stopIfFault();

}

//============

void loop() {
  
}
