// #pragma once
// #include "clock.h"
// #include <cmath>
// #include <iostream>

// class PID {
// public:
//   PID(double dt, double max, double min, double Kp, double Ki, double Kd);
//   double calculate(double setpoint, double pv, double dt, double& P, double& I, double& D, double gyroVelocity);
//   void button(int buttonState);

// private:
//   double _dt;
//   double _max;
//   double _min;
//   double _Kp;
//   double _Kd;
//   double _Ki;
//   int error_pointer;
//   double _pre_error;
//   double _integral;
//   int oldState;
//   static const int arrLen = 2;
//   double error_arr[arrLen];
// };
