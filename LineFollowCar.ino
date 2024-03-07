#include "MotorClass.h"
#include "PIDClass.h"

double dt = 0.001;

MotorClass motorA(10, 8, 9);
MotorClass motorB(11, 12, 13);
PIDClass PIDA(180.0, 150.0, 0.001, dt);
PIDClass PIDB(100.0, 0.0, 1.0, dt);
double KpLarge = 255*2;

int IRA = 7;//Right Sensor
int IRB = 6;//Middle Sensor
int IRC = 5;//Left Sensor
double baseSpeed = 190;
void setup() {
  Serial.begin(9600);
  motorA.begin();
  motorB.begin();
  pinMode(IRA, INPUT);
  pinMode(IRB, INPUT);
  pinMode(IRC, INPUT);
  PIDA.setErrorThreshold(0);
}
int IRASignal;
int IRBSignal;
int IRCSignal;
void loop() {
  IRASignal = digitalRead(IRA);
  IRBSignal = digitalRead(IRB);
  IRCSignal = digitalRead(IRC); //= digitalRead(IRC);

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

  //setMotorSpeed(255, 255);
  setCarState(IRASignal, IRBSignal, IRCSignal);
  //Serial.print("CMD: ");
  //Serial.println(IRCSignal);
  delay(dt * 1000);
}

void setMotorSpeed(double motorSignalA, double motorSignalB) {
  motorSignalA = PIDA.saturationFilter(motorSignalA, -255, 255);
  motorSignalB = PIDB.saturationFilter(motorSignalB, -255, 255);
  Serial.print("MotorA: ");
  Serial.print(motorSignalA);
  Serial.print(" || MotorB: ");
  Serial.println(motorSignalB);
  motorA.rotate(getMotorState(motorSignalA), motorSignalA);
  motorB.rotate(getMotorState(motorSignalB), motorSignalB);
}

void carForward() {
  setMotorSpeed(baseSpeed, baseSpeed);
}

void carPIDControl() {
  int direction = IRASignal-IRCSignal;
  double signal = PIDA.PIDSignal(direction, 0);
  double motorSignalA = baseSpeed;
  double motorSignalB = baseSpeed;
  //Serial.println(signal);
  if (direction > 0) {
    motorSignalA = motorSignalA + signal;
  } else if (direction < 0) {
    motorSignalB = motorSignalB - signal;
  } else {

  }
  //double motorSignalA = baseSpeed - PIDA.PIDSignal(-IRASignal, 0);
  //double motorSignalB = baseSpeed - PIDB.PIDSignal(-IRCSignal, 0);
  setMotorSpeed(motorSignalA, motorSignalB);
}

void carPIDControl2() {
  int direction = IRASignal-IRCSignal;
  double signal = PIDA.PIDSignal(abs(direction), 0);
  double motorSignalA = baseSpeed + signal * direction;
  double motorSignalB = baseSpeed + signal * -direction;
  setMotorSpeed(motorSignalA, motorSignalB);
}

void carHardTurn(int isRightTurn) {
  double KpA = PIDA.Kp;
  double KpB = PIDB.Kp;
  PIDA.Kp = KpA*(1-isRightTurn) + KpLarge*isRightTurn;
  PIDB.Kp = KpB*isRightTurn + KpLarge*(1-isRightTurn);
  double motorSignalA = baseSpeed - PIDA.PIDSignal(-IRASignal, 0);
  double motorSignalB = baseSpeed - PIDB.PIDSignal(-IRCSignal, 0);
  PIDA.Kp = KpA;
  PIDB.Kp = KpB;
  int direction = IRASignal - IRCSignal;
  setMotorSpeed(255*(-direction), 255*(direction));
}

void setCarState(int signalA, int signalB, int signalC) {
  carPIDControl();
  // if ((signalA == HIGH) & (signalB == HIGH) & (signalC == HIGH)) { //Forward
  //   carForward();
  // } else if ((signalA == LOW) & (signalB == HIGH) & (signalC == HIGH)) { //Turn Left
  //   carPIDControl();
  // } else if ((signalA == HIGH) & (signalB == HIGH) & (signalC == LOW)) { //Turn Left
  //   carPIDControl();
  // } else if ((signalA == LOW) & (signalB == LOW) & (signalC == HIGH)) { //Hard Turn Left
  //   carPIDControl();//carHardTurn(0);
  // } else if ((signalA == HIGH) & (signalB == LOW) & (signalC == LOW)) { //Hard Turn Right
  //   carPIDControl();//carHardTurn(1);
  // } else { //Forward
  //   carForward();
  // }
}

int getMotorState(double value) {
  if (value > 1) {
    //Serial.println("We good");
    return 3;
  } else if (value < -1) {
    //Serial.println("BRUH");
    return 2;
  }
  return 0;
}