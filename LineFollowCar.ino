#include "MotorClass.h"
#include "PIDClass.h"

double dt = 0.01;

MotorClass motorA(10, 8, 9);
MotorClass motorB(11, 12, 13);
PIDClass PIDA(100.0, 0.0, 1.0, dt);
PIDClass PIDB(100.0, 0.0, 1.0, dt);
double KpLarge = 255;

int IRA = 7;//Right Sensor
int IRB = 6;//Middle Sensor
int IRC = 5;//Left Sensor
double baseSpeed = 255;

void setup() {
  Serial.begin(9600);
  motorA.begin();
  motorB.begin();
  pinMode(IRA, INPUT);
  pinMode(IRB, INPUT);
}

void loop() {
  int IRASignal = digitalRead(IRA);
  int IRBSignal = digitalRead(IRB);
  int IRCSignal = digitalRead(IRC); //= digitalRead(IRC);

  // double motorSignalA;
  // double motorSignalB;

  // if ((IRCSignal == HIGH) & (IRASignal == HIGH) & (IRBSignal == LOW)) { //Hard right
  //   double KpA = PIDA.Kp;
  //   double KpB = PIDB.Kp;
  //   PIDA.Kp = KpLarge;
  //   PIDB.Kp = KpLarge;
  //   motorSignalA = baseSpeed - PIDA.PIDSignal(-IRASignal, 0);
  //   motorSignalB = baseSpeed - PIDB.PIDSignal(-IRBSignal, 0);
  //   PIDA.Kp = KpA;
  //   PIDB.Kp = KpB;
  // } else if ((IRCSignal == HIGH) & (IRASignal == HIGH) & (IRBSignal == HIGH)) { //Forward
  //   motorSignalA = baseSpeed - PIDA.PIDSignal(-IRASignal, 0);
  //   motorSignalB = baseSpeed - PIDB.PIDSignal(-IRBSignal, 0);
  // } else if ((IRCSignal == LOW) & (IRASignal == LOW) & (IRBSignal == LOW)) { //Forward
  //   motorSignalA = baseSpeed - PIDA.PIDSignal(-IRASignal, 0);
  //   motorSignalB = baseSpeed - PIDB.PIDSignal(-IRBSignal, 0);
  // }

  // setMotorSpeed(motorSignalA, motorSignalB);
  setCarState(IRASignal, IRBSignal, IRCSignal)
  Serial.print("CMD: ");
  Serial.println(IRBSignal);
  delay(dt * 1000);
}

void setMotorSpeed(double motorSignalA, double motorSignalB) {
  motorSignalA = PIDA.saturationFilter(motorSignalA, -255, 255);
  motorSignalB = PIDB.saturationFilter(motorSignalB, -255, 255);
  motorA.rotate(getMotorState(motorSignalA), motorSignalA);
  motorB.rotate(getMotorState(motorSignalB), motorSignalB);
}

void carForward() {
  double motorSignalA = baseSpeed - PIDA.PIDSignal(-IRASignal, 0);
  double motorSignalB = baseSpeed - PIDB.PIDSignal(-IRBSignal, 0);
  setMotorSpeed(motorSignalA, motorSignalB);
}

void carTurn() {
  double KpA = PIDA.Kp;
  double KpB = PIDB.Kp;
  PIDA.Kp = KpLarge;
  PIDB.Kp = KpLarge;
  double motorSignalA = baseSpeed - PIDA.PIDSignal(-IRASignal, 0);
  double motorSignalB = baseSpeed - PIDB.PIDSignal(-IRBSignal, 0);
  PIDA.Kp = KpA;
  PIDB.Kp = KpB;
  setMotorSpeed(motorSignalA, motorSignalB);
}

void setCarState(signalA, signalB, signalC) {
  if ((signalA == HIGH) & (signalA == HIGH) & (signalA == HIGH)) { //Forward
    carForward();
  } else if ((signalA == LOW) & (signalA == HIGH) & (signalA == HIGH)) { //Hard Turn Left
    carTurn();
  } else if ((signalA == LOW) & (signalA == HIGH) & (signalA == HIGH)) { //Hard Turn Right
    carTurn();
  } else { //Forward
    carForward();
  }
}

int getMotorState(double value) {
  if (value > 1) {
    return 3;
  } else if (value < -1) {
    return 2;
  }
  return 0;
}