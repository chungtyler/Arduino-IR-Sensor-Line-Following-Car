#include "PIDClass.h"
#include "Arduino.h"

//Constructor
PIDClass::PIDClass(double _Kp, double _Ki, double _Kd, double _dt) {
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
  dt = _dt;
  totalError = 0.0;
  lastError = 0.0;
  errorThreshold = 3.0;
}

//Setter
void PIDClass::setErrorThreshold(double newErrorThreshold) {
  errorThreshold = newErrorThreshold;
}

//Getter
double PIDClass::getTotalError() {
  return totalError;
}

//PID Control Methods
int PIDClass::toggleIntegrator(double error) {
  if (abs(error) > errorThreshold) {
    return 1;
  }
  totalError = 0;
  return 0;
}

double PIDClass::saturationFilter(double controlCommand, double MIN, double MAX) {
  double saturatedCommand = constrain(controlCommand, MIN, MAX);
  return saturatedCommand;
}

double PIDClass::PIDSignal(double input, double setpoint) {
  //Calculate Error
  double error = setpoint - input;

  //Calculate PID Signal
  double P = Kp * error;
  double I = Ki * totalError * dt * toggleIntegrator(error);
  double D = Kd * (error - lastError) / dt;

  //Update Saved Variables
  totalError += error;
  lastError = error;

  double signal = P + I + D;
  return signal;
}
