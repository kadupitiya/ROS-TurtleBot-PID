//
// Created by kadupitiya on 10/10/18.
//

#ifndef PID_H
#define PID_H
class PIDImpl;
class PID
{
public:
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // dt -  loop interval time
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable
    PID( double dt, double max, double min, double Kp, double Kd, double Ki );

    // Returns the manipulated variable given a setpoint and current process value
    double calculate( double setpoint, double pv );
    ~PID();

private:
    PIDImpl *pimpl;
};


#endif //PID_H
