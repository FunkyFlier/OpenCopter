#include "PIDL.h"

PID::PID(float *set,float *act, float *adj,boolean *intToggle,float *p, float *i, float *d,float *n,float *delta,float iLim,float lim){
  setPoint = set;
  actual = act;
  adjustment = adj;
  integrate = intToggle;
  kp = p;
  ki = i;
  kd = d;
  fc = n;
  integralLimitHigh = iLim;
  integralLimitLow = -1*iLim;
  outputLimitHigh = lim;
  outputLimitLow = -1*lim;
  dt = delta;
  prevActual = 0;
  dErrorPrev = 0;

}

void PID::calculate(){
  error = *setPoint - *actual;


  if (*integrate == true){
    iError += *ki * *dt * error;
  }
  if (iError > integralLimitHigh){
    iError = integralLimitHigh;
  }
  if (iError < integralLimitLow){
    iError = integralLimitLow;
  }

  dError = dErrorPrev - *fc * *dt * dErrorPrev + *kd * *fc * (error - prevError);

  *adjustment = *kp * error  + iError +  dError;

  if (*adjustment > outputLimitHigh){
    *adjustment  = outputLimitHigh;
  }
  if (*adjustment < outputLimitLow){
    *adjustment = outputLimitLow;
  }

  prevError = error;
  dErrorPrev = dError;
}

void PID::reset(){
  error = 0;
  iError = 0;
  dError = 0;
  *adjustment = 0;
  prevActual = *actual;
  prevError =0;
  dErrorPrev = 0;
}

YAW::YAW(float *set,float *act, float *adj,boolean *intToggle,float *p, float *i, float *d,float *n,float *delta,float iLim,float lim){
  setPoint = set;
  actual = act;
  adjustment = adj;
  integrate = intToggle;
  kp = p;
  ki = i;
  kd = d;
  fc = n;
  integralLimitHigh = iLim;
  integralLimitLow = -1*iLim;
  outputLimitHigh = lim;
  outputLimitLow = -1*lim;
  dt = delta;
  prevActual = 0;
  prevError =0;
  singularityState =0;
  dErrorPrev = 0;

}

void YAW::calculate(){
  PIDAngle = *actual;

  error = *setPoint - PIDAngle;
  errorDiff = prevError - error;

  if (errorDiff > 180.0){
    PIDAngle = *actual -360;
    error = *setPoint - PIDAngle;
  }
  if (errorDiff < -180.0){
    PIDAngle = *actual  +360;
    error = *setPoint - PIDAngle;
  }

  dError = dErrorPrev - *fc * *dt * dErrorPrev + *kd * *fc * (error - prevError);
  if (*integrate == true){
    iError += *ki * *dt * error;
  }

  if (iError > integralLimitHigh){
    iError = integralLimitHigh;
  }
  if (iError < integralLimitLow){
    iError = integralLimitLow;
  }

  *adjustment = *kp * error + iError +  dError;

  if (*adjustment > outputLimitHigh){
    *adjustment  = outputLimitHigh;
  }
  if (*adjustment < outputLimitLow){
    *adjustment = outputLimitLow;
  }
  prevActual = *actual;
  prevError = error;
  dErrorPrev = dError;

}

void YAW::reset(){
  error = 0;
  prevError=0;
  iError = 0;
  dError = 0;
  *adjustment = 0;
  prevActual = *actual;
  dErrorPrev = 0;
}

