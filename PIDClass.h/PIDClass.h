#ifndef PIDClass_h
#define PIDClass_h

class PIDClass {
  public:
    double Kp;
    double Ki;
    double Kd;
    //Constructor
    PIDClass(double Kp, double Ki, double Kd, double dt);

    //Setter
    void setErrorThreshold(double newErrorThreshold);

    //Getter
    double getTotalError();

    double PIDSignal(double input, double setpoint);
    double saturationFilter(double controlCommand, double MIN, double MAX);

  private:
    double dt;
    double totalError;
    double lastError;
    double errorThreshold;

    //PID Control Functions
    int toggleIntegrator(double error);
};

#endif
