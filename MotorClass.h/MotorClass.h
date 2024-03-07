#ifndef MotorClass_h
#define MotorClass_h

class MotorClass {
  public:
    struct stateVector {
      int a;
      int b;
    };
    int state;
    int lastState;
    double speed;

    //Constructor
    MotorClass(int pwmPin, int pin1, int pin2);

    void begin();

    //Getter Methods
    int getPWMPin();
    int getPin1();
    int getPin2();

    //Motor Control Methods
    struct stateVector getStateVector(int newState);
    void safetyBrake();
    void rotate(int newState, double newSpeed);

  private:
    int pwmPin;
    int pin1;
    int pin2;
};

#endif
