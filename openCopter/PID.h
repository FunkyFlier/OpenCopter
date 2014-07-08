#ifndef PID_h
#define PID_h


#include <Arduino.h>
#include "AUXMATH.h"

class PID{
public:
  PID(float*, float*, float*,boolean*,float*, float*, float*, float*,float*,float,float);
  void calculate();
  void reset();
  float integralLimitHigh;
  float integralLimitLow;
  float outputLimitHigh;
  float outputLimitLow;
  float *dt;
  float *setPoint;
  float *actual;
  float *adjustment;
  boolean *integrate;
  float *kp;
  float *ki;
  float *kd;
  float *fc;
  float prevError;
  float prevActual;
  float error;
  float iError;
  float dError;
  float dErrorPrev;
private:
  

};
class ALT{
public:
  ALT(float*, float*, float*,boolean*,float*, float*, float*, float*,float*,float,float,float*);
  void calculate();
  void reset();
  float integralLimitHigh;
  float integralLimitLow;
  float outputLimitHigh;
  float outputLimitLow;
  float *dt;
  float *setPoint;
  float *actual;
  float *adjustment;
  boolean *integrate;
  float *kp;
  float *ki;
  float *kd;
  float *fc;
  float *multiplier;
  float prevError;
  float prevActual;
  float error;
  float iError;
  float dError;
  float dErrorPrev;
private:
  

};
class YAW{
public:
  YAW(float*, float*, float*,boolean*,float*, float*, float*, float*,float*,float,float);
  void calculate();
  void reset();
private:
  float integralLimitHigh;
  float integralLimitLow;
  float outputLimitHigh;
  float outputLimitLow;
  float *dt;
  float *setPoint;
  float *actual;
  float *adjustment;
  boolean *integrate;
  float *kp;
  float *ki;
  float *kd;
  float *fc;
  float prevError;
  float prevActual;
  float error;
  float iError;
  float dError;
  float dErrorPrev;
  int singularityState;
  float PIDAngle;
  float errorDiff;

};


#endif

