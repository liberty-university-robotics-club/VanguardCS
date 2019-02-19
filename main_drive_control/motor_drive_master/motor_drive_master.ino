#include "DualVNH5019MotorShield.h"
#include <SPI.h>

DualVNH5019MotorShield md;

const int buttonPinF = A0;
const int buttonPinB = A1;
const int speedSensor = A2;
const int eStop = A3;

int buttonFState = 0;
int buttonBState = 0;
int speedValue = 0;
int eStatus = 0;

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
  SPI.begin();
  md.init();
  pinMode(buttonPinF, INPUT);
  pinMode(buttonPinB, INPUT);
  pinMode(eStop, INPUT);

}

void loop() {
  eStatus = digitalRead(eStop);
  buttonFState = digitalRead(buttonPinF);
  buttonBState = digitalRead(buttonPinB);
  speedValue = map(analogRead(speedSensor), 0, 1023 , 0, 400);

  Serial.print("M1 current: ");
  Serial.println(md.getM1CurrentMilliamps());
  Serial.print("M2 current: ");
  Serial.println(md.getM2CurrentMilliamps());
  Serial.println(speedValue);
  
  if (eStatus == HIGH) {
    md.setBrakes(400, 400);
    stopIfFault();
  }
  if (buttonFState == HIGH) {
    for(int i = 0 ;  i <= speedValue; i++){
    md.setSpeeds(i, i);
    stopIfFault();
    }
  }
  if (buttonFState == LOW) {
    for(int i = 0 ;  i >= speedValue; i--){
    md.setSpeeds(i, i);
    stopIfFault();
    }
  }
  if (buttonFState == HIGH) {
    for(int i = 0 ;  i <= speedValue; i++){
    md.setSpeeds(-i, -i);
    stopIfFault();
    }
  }
  if (buttonFState == LOW) {
    for(int i = 0 ;  i >= speedValue; i--){
    md.setSpeeds(-i, -i);
    stopIfFault();
    }
  }
}
