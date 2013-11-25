// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//This code is provided under the GNU General Public Licence
//12/19/2012
//Gyro must be in RAD/s
//Sensors must be in the North East Down convention
#ifndef openIMUL_h
#define openIMUL_h
#include "AUXMATH.h"

#include <Arduino.h>

#define DECLINATION 3.66f

#define twoKpDef  (2.0f * 1.6f)	// 2 * proportional gain
#define twoKiDef  (2.0f * 0.1f)	// 2 * integral gain

#define v1_ 2.0f//GPS
#define v2_ 1.0f//baro
#define w_ 5.0f

#define PITCH_OFFSET 0.0f
#define ROLL_OFFSET 0.0f



class openIMU{
public:
  openIMU(float*, float*, float*, float*, float*, 
          float*, float*, float*, float*, float*, 
          float*, float*, float* , float*, float*, float*);
  void AHRSupdate(void);
  void GetEuler(void);
  void InitialQuat(void);
  void BaroKalUpdate(void);
  void AccKalUpdate(void);
  void GPSKalUpdate(void);
  float q0,q1,q2,q3;
  float pitch,roll,yaw;
  //float cosYaw,sinYaw;
  float XEst,YEst,ZEst;
  float velX,velY,velZ;
  float inertialX,inertialY,inertialZ;
private:
  float *gx;
  float *gy;
  float *gz;
  float *ax;
  float *ay;
  float *az;
  float *mx;
  float *my;
  float *mz;
  float *dt;
  //float *altDt;
  float *sax;//scaled accelerometer value
  float *say;
  float *saz;
  float *XRaw,*YRaw,*ZRaw;
  

  //IMU & AHRS vars
  float squareSum;
  float magnitude;
  float twoKp,twoKi;
  
  float integralFBx,integralFBy,integralFBz;

  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  //Altimeter vars

  float p11;
  float p12;
  float p21;
  float p22;
  
  float p13;
  float p14;
  float p23;
  float p24;
  
  float p15;
  float p16;
  float p25;
  float p26;
  float k1,k2,k3,k4,k5,k6;

};


#endif



