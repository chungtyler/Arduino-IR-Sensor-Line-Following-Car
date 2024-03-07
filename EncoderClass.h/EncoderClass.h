#ifndef EncoderClass_h
#define EncoderClass_h

class EncoderClass {
  public:
    volatile int pulse;

    //Constructor
    EncoderClass(int _pinA, int _pinB, double _PPR);

    void begin();

    //Getter Methods
    int getPinA();
    int getPinB();
    double getPPR();

  private:
    int pinA;
    int pinB;
    int PPR;

    static EncoderClass* instance;
    static void interruptHandler();
    void updateCount();
};

#endif
