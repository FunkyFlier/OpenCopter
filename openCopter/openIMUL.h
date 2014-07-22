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

//#define DECLINATION 3.66f



#define betaMag 0.04f
#define betaAcc 0.04f


#define vXY 2.0f
#define vZ_Baro 2.5f
#define vZ_Ping 0.1f

#define w1XY 2.0f
#define w2XY 1.0f
#define w3XY 0.0001f


#define w1Z 0.5f
#define w2Z 0.15f
#define w3Z 0.0001f

#define LAG_SIZE 20

class openIMU{
public:
  openIMU(float*, float*, float*, float*, float*, 
  float*, float*, float*, float*, float*, 
  float*, float*, float* , float*, float*, float*);
  
  void AHRSupdate(void);
  void GetEuler(void);
  void GetPitch(void);
  void GetRoll(void);
  void GetYaw(void);
  void InitialQuat(void);
  void BaroKalUpdate(void);
  void PingKalUpdate(void);
  void AccKalUpdate(void);
  void GPSKalUpdate(void);
  float GetGravOffset(void);
  void UpdateLagIndex(void);
  void pUpateZ(void);
  void pUpateY(void);
  void pUpateX(void);
  void GetInertial(void);
  void AHRSStart(void);
  void AHRSEnd(void);
  void IMUupdate(void);
  
  float_u q0,q1,q2,q3;
  float_u pitch,roll,yaw;
  float_u XEst,YEst,ZEst;
  float_u velX,velY,velZ;
  float accelBiasX,accelBiasY,accelBiasZ;
  float_u inertialX,inertialY,inertialZ;
  float inertialZ_Grav;
  float gravityOffSet;
  boolean feedBack;
  float xError,yError;
  float XEstHist[LAG_SIZE],YEstHist[LAG_SIZE];
  uint8_t currentEstIndex,lagIndex,currentEstIndex_z,lagIndex_z;  
  float_u rawPitch,rawRoll;
  float_u pitchOffset,rollOffset;
  float_u declination;
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
  float *sax;//scaled accelerometer value
  float *say;
  float *saz;
  float *XRaw,*YRaw,*ZRaw;

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float  _4q0, _4q1, _4q2 ,_8q1, _8q2;
  float squareSum;
  float magnitude;

  //Position and velocity vars

  float p11X,p12X,p13X;
  float p21X,p22X,p23X;
  float p31X,p32X,p33X;

  float p11Y,p12Y,p13Y;
  float p21Y,p22Y,p23Y;
  float p31Y,p32Y,p33Y;

  float p11Z,p12Z,p13Z;
  float p21Z,p22Z,p23Z;
  float p31Z,p32Z,p33Z;

  float p11X_,p12X_,p13X_;
  float p21X_,p22X_,p23X_;
  float p31X_,p32X_,p33X_;

  float p11Y_,p12Y_,p13Y_;
  float p21Y_,p22Y_,p23Y_;
  float p31Y_,p32Y_,p33Y_;

  float p11Z_,p12Z_,p13Z_;
  float p21Z_,p22Z_,p23Z_;
  float p31Z_,p32Z_,p33Z_;

  float k1X,k2X,k3X;

  float k1Y,k2Y,k3Y;

  float k1Z,k2Z,k3Z;


};


#endif







