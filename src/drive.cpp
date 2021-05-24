#include "drive.h"
#include "variables.h"
#include <cmath>
#include <iostream>

void* drive(void* arg) {

  Position* pos = (Position*)arg;
  bool init = true;
  Clock clock;
  while (Global::buttonPress == 0) {
    usleep(50 * 1000);
  }

  if (Global::buttonPress == 2) {
    // com.Run();
    // Command::SetRun(&pos->speed, &pos->turn, &Global::gyroAngle, -90, &Global::interupt, .4);
    // Command::SetRun(&pos->speed, &pos->turn, &Global::gyroAngle, 180, &Global::interupt, .25);
  }
  while (true) {
    // printf("buttonPress %d\n",buttonPress);
    if (Global::buttonPress == 0)
      init = true;
    if (Global::buttonPress == 1 && init) { // initialize loop when first button pressed;
      init = false;
      printf("button press: INIT!\n");
      Global::driveAngle = -Global::gyroAngle;
    }
    // if (updated) {
    //   // update stuff when target is seen
    // }
    if (Global::buttonPress == 1) {
      pos->speed = 0.0;
      // if(turn>=0)
      // pos->turn = std::min(sqrt(abs(turn/1.5)),0.7);
      // else
      // pos->turn = std::max(-sqrt(abs(turn/1.5)),-0.7);
      pos->turn = Global::turn;
    } else {
      pos->speed = 0.0;
      pos->turn = 0.0;
    }
    if (clock.getTimeAsMillis() > 100 && Global::interupt) {
      printf("turn: %5.2f angle: %6.2f ; P %5.2f, I %5.2f, D %5.2f, GZ: %5.2f gyro: %7.2f\n", pos->turn, pos->alpha1, Global::P, Global::I, Global::D, Global::gyroVelocity, Global::gyroAngle);
      clock.restart();
    }
    // if (updated) {
    //   Global::driveAngle = alphaGlobal + (-Global::gyroAngle); // calculate the needed position for the turn
    //   updated = false;
    // }
    // move = (pos->dist > 75) ? true : false;
    usleep(50);
  }
  return 0;
}

void* movePID(void* arg) {

  PID* drivePID;
  Clock clock;
  double turnLoc, dt;
  if (Switches::InitPID[0] == -1)
    Switches::InitPID[0] = 0.0275;
  if (Switches::InitPID[1] == -1)
    Switches::InitPID[1] = 0.0;
  if (Switches::InitPID[2] == -1)
    Switches::InitPID[2] = 0.001;

  // something on the internet that doesnt seem to work?
  // double Tu = 4.5/1.;
  // double Ku = 0.0325;
  // double Ku = 0.035;
  // InitPID[0] = Ku * 0.6;
  // InitPID[1] = 1.2 * Ku / Tu;
  // InitPID[2] = 3 * Ku * Tu / 40;

  drivePID = new PID(0.0, 1, -1, Switches::InitPID[0], Switches::InitPID[1], Switches::InitPID[2]);

  while (true) {
    dt = clock.getTimeAsMicros();
    clock.restart();
    if (Global::buttonPress == 2)
      drivePID->button(1);
    else
      drivePID->button(Global::buttonPress);
    turnLoc = drivePID->calculate(Global::driveAngle, -Global::gyroAngle, dt, Global::P, Global::I, Global::D, Global::gyroVelocity);
    // printf("gyroVel: %.2f\n",Global::gyroVelocity);
    // turnLoc = drivePID->calculate(0,OffSetX,dt,&P,&I,&D);
    Global::turn = turnLoc;
    usleep(10000);
  }
  return 0;
}

PID::PID(double dt, double max, double min, double Kp, double Ki, double Kd) {
  _dt = dt;
  _max = max;
  _min = min;
  _Kp = Kp;
  _Kd = Kd;
  _Ki = Ki;
  error_pointer = -1;
  _pre_error = 0;
  _integral = 0;
  oldState = 0;
  for (unsigned int i = 0; i < arrLen; i++)
    error_arr[i] = 0;
}

double PID::calculate(double setpoint, double pv, double dt, double& P, double& I, double& D, double gyroVelocity) {
  _dt = dt;
  double error = setpoint - pv;
  error_pointer = (error_pointer + 1) % arrLen;
  error_arr[error_pointer] = error;
  double Pout = _Kp * error;
  if (abs(gyroVelocity) < 1 && abs(error) < 5) {
    Pout = error * (_Kp)*8;
    if (abs(Pout) > .25)
      Pout = copysign(.25, Pout);
  }
  _integral += error * _dt;
  double Iout = _Ki * _integral;
  double Dout = (error_arr[error_pointer] - error_arr[(error_pointer + arrLen + 1) % arrLen]) * _Kd;
  double output = Pout + Iout + Dout;
  if (abs(Pout) > 1.2)
    Pout = copysign(1.2, Pout);
  if (abs(Iout) > 1.2)
    Iout = copysign(1.2, Iout);
  if (abs(Dout) > 1.2)
    Dout = copysign(1.2, Dout);
  // printf("fix-gyro:%6.2f, %6.2f, errorPID: %6.2f, Pout: %6.2f, Iout: %6.2f, Dout:%6.2f, output:%6.2f\n", setpoint, pv, error, Pout, Iout, Dout, output);
  P = Pout;
  I = Iout;
  D = Dout;
  if (output > _max)
    output = _max;
  else if (output < _min)
    output = _min;
  _pre_error = error;
  return output;
}

void PID::button(int buttonState) {

  if (buttonState == 1 && oldState == 0) {
    _integral = 0;
    _pre_error = 0;
    for (unsigned int i = 0; i < arrLen; i++)
      error_arr[i] = 0;
  }
  oldState = buttonState;
}
