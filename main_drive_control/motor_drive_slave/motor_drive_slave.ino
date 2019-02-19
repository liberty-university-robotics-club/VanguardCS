#include "DualVNH5019MotorShield.h"
#include <SPI.h>

DualVNH5019MotorShield md;

char buf [100];
volatile byte pos;
volatile bool process_it;

void stopIfFault() {
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    delay(1000);
    
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();
  
  // turn on SPI in slave mode
  SPCR |= bit (SPE);

  // have to send on master in, *slave out*
  pinMode (MISO, OUTPUT);

  // get ready for an interrupt
  pos = 0;   // buffer empty
  process_it = false;

  // now turn on interrupts
  SPI.attachInterrupt();

}

// SPI interrupt routine
ISR (SPI_STC_vect)
{
byte c = SPDR;  // grab byte from SPI Data Register

  // add to buffer if room
  if (pos < sizeof buf)
    {
    buf [pos++] = c;

    // example: newline means time to process buffer
    if (c == '\n')
      process_it = true;

    }  // end of room available
}  // end of interrupt routine SPI_STC_vect

void loop() {

  if (process_it)
    {
    buf [pos] = 0;
    Serial.println (buf);
    pos = 0;
    process_it = false;
    }  // end of flag set

  

  
  
  
  
}
