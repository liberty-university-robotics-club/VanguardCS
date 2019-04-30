#include "DualVNH5019MotorShield.h"
#include <Wire.h>

DualVNH5019MotorShield md;

int right;
int left;
int rightN;
int leftN;

//============

void stopIfFault() {
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    delay(1000);
    return;
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    delay(1000);
    return;
  }
}

//============

void setup() {
  Serial.begin(115200);
  Serial.println("Motor control - 0xA");
  md.init();
  
  Wire.begin(0xA);                // join i2c bus with address 0xA
  Wire.onReceive(receiveEvent);   // register event
}

//============

void receiveEvent(int howMany){
  while (Wire.available()){
  right = Wire.read();
  rightN = Wire.read();
  left = Wire.read();
  leftN = Wire.read();
  }
  // set the speeds to negative if condition is true
  if (rightN == 1){
    right = right * -1;
  }
  if (leftN == 1){
    left = left * -1;
  }
  Serial.print("right: ");
  Serial.println(right);
  Serial.print("left: ");
  Serial.println(left);

  md.setSpeeds(right, left);
}

//============

void loop() {
  stopIfFault();
}
