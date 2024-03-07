#include "MotorClass.h"
#include "Arduino.h"

//Constructor
MotorClass::MotorClass(int _pwmPin, int _pin1, int _pin2) {
  pwmPin = _pwmPin;
  pin1 = _pin1;
  pin2 = _pin2;
  state = 0;
  lastState = 0;
  speed = 0.0;
}

void MotorClass::begin() {
  //Attach Hardware Pins
  pinMode(pwmPin, OUTPUT);
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
}

//Getter Methods
int MotorClass::getPWMPin() {
  return pwmPin;
}

int MotorClass::getPin1() {
  return pin1;
}

int MotorClass::getPin2() {
  return pin2;
}

//Motor Control Methods
struct MotorClass::stateVector MotorClass::getStateVector(int newState) {
  int stateMatrix[4][2] {
    {1, 1}, //Short Break = 0
    {0, 0}, //Stop = 1
    {1, 0}, //CW = 2
    {0, 1}  //CCW = 3
  };

  struct stateVector stateVec;
  stateVec.a = stateMatrix[newState][0];
  stateVec.b = stateMatrix[newState][1];

  return stateVec;
}

void MotorClass::safetyBrake() {
  if (state + lastState == 5 && state != lastState) {
    digitalWrite(pin1, 1);
    digitalWrite(pin2, 1);
    delay(100);
  }

  return;
}

void MotorClass::rotate(int newState, double newSpeed) {
  //Update Parameters
  lastState = state;
  state = newState;
  speed = abs(newSpeed);
  struct stateVector stateVec = getStateVector(state);

  //Apply Safety Brakes and Rotate Motor
  safetyBrake();
  analogWrite(pwmPin, speed);
  digitalWrite(pin1, stateVec.a);
  digitalWrite(pin2, stateVec.b);
}
