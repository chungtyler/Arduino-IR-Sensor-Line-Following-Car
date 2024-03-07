#include "EncoderClass.h"
#include "Arduino.h"

EncoderClass* EncoderClass::instance = nullptr;

//Constructor
EncoderClass::EncoderClass(int _pinA, int _pinB, double _PPR) {
  pinA = _pinA;
  pinB = _pinB;
  PPR = _PPR;
  pulse = 0;
}

void EncoderClass::begin() {
  instance = this;

  //Attach Hardware Pins
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA), interruptHandler, RISING);
}

//Interrupt Handler Methods
void EncoderClass::interruptHandler() {
  if (instance != nullptr) {
    instance->updateCount();
  }
}

//Getter Methods
int EncoderClass::getPinA() {
  return pinA;
}

int EncoderClass::getPinB() {
  return pinB;
}

double EncoderClass::getPPR() {
  return PPR;
}

//Encoder Counter Functions
void EncoderClass::updateCount() {
  //Get number of encoder counts
  if (digitalRead(pinB) > 0) {
    pulse++;
  } else {
    pulse--;
  }
}
