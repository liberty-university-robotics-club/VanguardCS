#include "DualVNH5019MotorShield.h"
#include <Wire.h>

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
  Serial.println("Motor control - 0xB");
  md.init();
  
  Wire.begin(0xB);                // join i2c bus with address 0xB
  Wire.onReceive(receiveEvent);   // register event

}

//============

void receiveEvent(int howMany){
  while(1 < Wire.available()){
    int right = Wire.read();
    Serial.println("right: ");
    Serial.print(right);
  }
  int left = Wire.read();
  Serial.println("left: ");
  Serial.print(left);

  md.setSpeeds(right, left);
}

//============

void loop() {
  stopIfFault();
}
