// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//This code is provided under the GNU General Public Licence
//12/19/2012
#include "openIMUL.h"
#include <Streaming.h>



openIMU::openIMU(float *gyroX, float *gyroY, float *gyroZ, float *accX, float *accY, 
float *accZ, float *scAccX, float *scAccY, float *scAccZ, float *magX, float *magY, float *magZ, float *XIn , float *YIn, float *ZIn, float *G_Dt, float *dec){
  //constructor for the 10DOF system
  gx = gyroX;
  gy = gyroY;
  gz = gyroZ;

  ax = accX;
  ay = accY;
  az = accZ;

  mx = magX;
  my = magY;
  mz = magZ;

  sax = scAccX;
  say = scAccY;
  saz = scAccZ;

  XRaw = XIn;
  YRaw = YIn;
  ZRaw = ZIn;

  dt = G_Dt;


  declination = dec;



  q0 = 1;
  q1 = 0;
  q2 = 0;
  q3 = 0;

  //Kalman stuff
  XEst = 0;
  velX = 0;

  p11X = 0.5;
  p12X = 0;
  p13X = 0;

  p21X = 0;
  p22X = 0.5;
  p23X = 0;

  p31X = 0;
  p32X = 0;
  p33X = 0.01;

  YEst = 0;
  velY = 0;

  p11Y = 0.5;
  p12Y = 0;
  p13Y = 0;

  p21Y = 0;
  p22Y = 0.5;
  p23Y = 0;

  p31Y = 0;
  p32Y = 0;
  p33Y = 0.01;

  velZ = 0;
  ZEst = 0;

  p11Z = 1;
  p12Z = 0;
  p13Z = 0;

  p21Z = 0;
  p22Z = 1;
  p23Z = 0;

  p31Z = 0;
  p32Z = 0;
  p33Z = 0.01;

  currentEstIndex = 0;
  lagIndex = 1;
  currentEstIndex_z = 0;
  lagIndex_z = 1;
}

float openIMU::GetGravOffset(void){
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;

  return (2 * (*sax * (q1q3 - q0q2) + *say * (q2q3 + q0q1) + *saz * (q0q0 - 0.5 + q3q3) )) ; 
  //return 9.8;
}
void openIMU::GetInertial(void){
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;   



  inertialX =  (2.0f * (*sax * (0.5f - q2q2 - q3q3) + *say * (q1q2 - q0q3) + *saz * (q1q3 + q0q2)) );
  inertialY = 2.0 * (*sax * (q1q2 + q0q3) + *say * (0.5f - q1q1 - q3q3) + *saz * (q2q3 - q0q1));
  inertialZ_Grav = -1.0 * (2 * (*sax * (q1q3 - q0q2) + *say * (q2q3 + q0q1) + *saz * (q0q0 - 0.5 + q3q3) ));
  inertialZ = inertialZ_Grav + gravityOffSet;  
}
void openIMU::pUpateX(void){
  velX = velX + (inertialX * *dt) + (accelBiasX * *dt);

  XEst = XEst + velX * *dt + (inertialX * *dt * *dt * 0.5) + (accelBiasX * *dt * *dt * 0.5);
  XEstHist[currentEstIndex] = XEst;

  p11X_ = p11X + *dt * p21X + *dt * w1XY + *dt * ((p32X * *dt * *dt)* 0.5 + p22X * *dt + p12X) + (*dt * *dt * p31X) * 0.5 + (*dt * *dt * ((p33X * *dt * *dt)/2 + p23X * *dt + p13X))* 0.5;
  p12X_ = p12X + *dt * p22X + *dt * ((p33X * *dt * *dt) * 0.5 + p23X * *dt + p13X) + (*dt * *dt *p32X) * 0.5;
  p13X_ = (p33X * *dt * *dt) * 0.5  + p23X * * dt + p13X;

  p21X_ = p21X + (*dt * *dt *(p23X + *dt * p33X)) * 0.5 + *dt * p31X + *dt * (p22X + *dt * p32X);
  p22X_ = p22X + *dt * p32X + *dt * w2XY + *dt * (p23X + *dt * p33X);
  p23X_ = p23X + *dt * p33X;

  p31X_ = (p33X * *dt * *dt) * 0.5 + p32X * *dt + p31X;
  p32X_ = p32X + *dt * p33X;
  p33X_ = p33X + *dt * w3XY;
  p11X = p11X_;
  p12X = p12X_;
  p13X = p13X_;

  p21X = p21X_;
  p22X = p22X_;
  p23X = p23X_;

  p31X = p31X_;
  p32X = p32X_;
  p33X = p33X_;  
}
void openIMU::pUpateY(void){
  velY = velY + (inertialY * *dt) + (accelBiasY * *dt);

  YEst = YEst + velY * *dt + (inertialY * *dt * *dt * 0.5) + (accelBiasY * *dt * *dt * 0.5);
  YEstHist[currentEstIndex] = YEst;

  p11Y_ = p11Y + *dt * p21Y + *dt * w1XY + *dt * ((p32Y * *dt * *dt)* 0.5 + p22Y * *dt + p12Y) + (*dt * *dt * p31Y) * 0.5 + (*dt * *dt * ((p33Y * *dt * *dt)/2 + p23Y * *dt + p13Y))* 0.5;
  p12Y_ = p12Y + *dt * p22Y + *dt * ((p33Y * *dt * *dt) * 0.5 + p23Y * *dt + p13Y) + (*dt * *dt *p32Y) * 0.5;
  p13Y_ = (p33Y * *dt * *dt) * 0.5  + p23Y * * dt + p13Y;

  p21Y_ = p21Y + (*dt * *dt *(p23Y + *dt * p33Y)) * 0.5 + *dt * p31Y + *dt * (p22Y + *dt * p32Y);
  p22Y_ = p22Y + *dt * p32Y + *dt * w2XY + *dt * (p23Y + *dt * p33Y);
  p23Y_ = p23Y + *dt * p33Y;

  p31Y_ = (p33Y * *dt * *dt) * 0.5 + p32Y * *dt + p31Y;
  p32Y_ = p32Y + *dt * p33Y;
  p33Y_ = p33Y + *dt * w3XY;  
  p11Y = p11Y_;
  p12Y = p12Y_;
  p13Y = p13Y_;

  p21Y = p21Y_;
  p22Y = p22Y_;
  p23Y = p23Y_;

  p31Y = p31Y_;
  p32Y = p32Y_;
  p33Y = p33Y_;  


}
void openIMU::pUpateZ(void){
  velZ = velZ + (inertialZ * *dt) + (accelBiasZ * *dt);

  ZEst = ZEst + velZ * *dt + (inertialZ * *dt * *dt * 0.5) + (accelBiasZ * *dt * *dt * 0.5);
  //ZEstHist[currentEstIndex_z] = ZEst;

  p11Z_ = p11Z + *dt * p21Z + *dt * w1Z + *dt * ((p32Z * *dt * *dt)* 0.5 + p22Z * *dt + p12Z) + (*dt * *dt * p31Z) * 0.5 + (*dt * *dt * ((p33Z * *dt * *dt)/2 + p23Z * *dt + p13Z))* 0.5;
  p12Z_ = p12Z + *dt * p22Z + *dt * ((p33Z * *dt * *dt) * 0.5 + p23Z * *dt + p13Z) + (*dt * *dt *p32Z) * 0.5;
  p13Z_ = (p33Z * *dt * *dt) * 0.5  + p23Z * * dt + p13Z;

  p21Z_ = p21Z + (*dt * *dt *(p23Z + *dt * p33Z)) * 0.5 + *dt * p31Z + *dt * (p22Z + *dt * p32Z);
  p22Z_ = p22Z + *dt * p32Z + *dt * w2Z + *dt * (p23Z + *dt * p33Z);
  p23Z_ = p23Z + *dt * p33Z;

  p31Z_ = (p33Z * *dt * *dt) * 0.5 + p32Z * *dt + p31Z;
  p32Z_ = p32Z + *dt * p33Z;
  p33Z_ = p33Z + *dt * w3Z;  

  p11Z = p11Z_;
  p12Z = p12Z_;
  p13Z = p13Z_;

  p21Z = p21Z_;
  p22Z = p22Z_;
  p23Z = p23Z_;

  p31Z = p31Z_;
  p32Z = p32Z_;
  p33Z = p33Z_;   
}

void openIMU::UpdateLagIndex(void){
  currentEstIndex++;

  lagIndex++;
  if (currentEstIndex == LAG_SIZE){
    currentEstIndex = 0;
  }
  if (lagIndex == LAG_SIZE){
    lagIndex = 0;
  }

  /*
  currentEstIndex_z++;
   
   lagIndex_z++;
   if (currentEstIndex_z == LAG_SIZE_Z){
   currentEstIndex_z = 0;
   }
   if (lagIndex_z == LAG_SIZE_Z){
   lagIndex_z = 0;
   }*/
}
void openIMU::AccKalUpdate(void){
  //first rotate the accelerometer reading from the body to the intertial frame
  // Auxiliary variables to avoid repeated arithmetic
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;   



  inertialX =  (2.0f * (*sax * (0.5f - q2q2 - q3q3) + *say * (q1q2 - q0q3) + *saz * (q1q3 + q0q2)) );
  inertialY = 2.0 * (*sax * (q1q2 + q0q3) + *say * (0.5f - q1q1 - q3q3) + *saz * (q2q3 - q0q1));
  inertialZ_Grav = -1.0 * (2 * (*sax * (q1q3 - q0q2) + *say * (q2q3 + q0q1) + *saz * (q0q0 - 0.5 + q3q3) ));
  inertialZ = inertialZ_Grav + gravityOffSet;

  //Kalman filter stuff - makes more sense looking at it in matrix form
  velX = velX + (inertialX * *dt) + (accelBiasX * *dt);

  XEst = XEst + velX * *dt + (inertialX * *dt * *dt * 0.5) + (accelBiasX * *dt * *dt * 0.5);
  XEstHist[currentEstIndex] = XEst;

  p11X_ = p11X + *dt * p21X + *dt * w1XY + *dt * ((p32X * *dt * *dt)* 0.5 + p22X * *dt + p12X) + (*dt * *dt * p31X) * 0.5 + (*dt * *dt * ((p33X * *dt * *dt)/2 + p23X * *dt + p13X))* 0.5;
  p12X_ = p12X + *dt * p22X + *dt * ((p33X * *dt * *dt) * 0.5 + p23X * *dt + p13X) + (*dt * *dt *p32X) * 0.5;
  p13X_ = (p33X * *dt * *dt) * 0.5  + p23X * * dt + p13X;

  p21X_ = p21X + (*dt * *dt *(p23X + *dt * p33X)) * 0.5 + *dt * p31X + *dt * (p22X + *dt * p32X);
  p22X_ = p22X + *dt * p32X + *dt * w2XY + *dt * (p23X + *dt * p33X);
  p23X_ = p23X + *dt * p33X;

  p31X_ = (p33X * *dt * *dt) * 0.5 + p32X * *dt + p31X;
  p32X_ = p32X + *dt * p33X;
  p33X_ = p33X + *dt * w3XY;


  velY = velY + (inertialY * *dt) + (accelBiasY * *dt);

  YEst = YEst + velY * *dt + (inertialY * *dt * *dt * 0.5) + (accelBiasY * *dt * *dt * 0.5);
  YEstHist[currentEstIndex] = YEst;

  p11Y_ = p11Y + *dt * p21Y + *dt * w1XY + *dt * ((p32Y * *dt * *dt)* 0.5 + p22Y * *dt + p12Y) + (*dt * *dt * p31Y) * 0.5 + (*dt * *dt * ((p33Y * *dt * *dt)/2 + p23Y * *dt + p13Y))* 0.5;
  p12Y_ = p12Y + *dt * p22Y + *dt * ((p33Y * *dt * *dt) * 0.5 + p23Y * *dt + p13Y) + (*dt * *dt *p32Y) * 0.5;
  p13Y_ = (p33Y * *dt * *dt) * 0.5  + p23Y * * dt + p13Y;

  p21Y_ = p21Y + (*dt * *dt *(p23Y + *dt * p33Y)) * 0.5 + *dt * p31Y + *dt * (p22Y + *dt * p32Y);
  p22Y_ = p22Y + *dt * p32Y + *dt * w2XY + *dt * (p23Y + *dt * p33Y);
  p23Y_ = p23Y + *dt * p33Y;

  p31Y_ = (p33Y * *dt * *dt) * 0.5 + p32Y * *dt + p31Y;
  p32Y_ = p32Y + *dt * p33Y;
  p33Y_ = p33Y + *dt * w3XY;
  //---------------------------------------------------------------------------------

  //velZ = velZ + (inertialZ_Grav * *dt) + (accelBiasZ * *dt);

  //ZEst = ZEst + velZ * *dt + (inertialZ_Grav * *dt * *dt * 0.5) + (accelBiasZ * *dt * *dt * 0.5);
  //---------------------------------------------------------------------------------
  velZ = velZ + (inertialZ * *dt) + (accelBiasZ * *dt);

  ZEst = ZEst + velZ * *dt + (inertialZ * *dt * *dt * 0.5) + (accelBiasZ * *dt * *dt * 0.5);
  //ZEstHist[currentEstIndex_z] = ZEst;

  p11Z_ = p11Z + *dt * p21Z + *dt * w1Z + *dt * ((p32Z * *dt * *dt)* 0.5 + p22Z * *dt + p12Z) + (*dt * *dt * p31Z) * 0.5 + (*dt * *dt * ((p33Z * *dt * *dt)/2 + p23Z * *dt + p13Z))* 0.5;
  p12Z_ = p12Z + *dt * p22Z + *dt * ((p33Z * *dt * *dt) * 0.5 + p23Z * *dt + p13Z) + (*dt * *dt *p32Z) * 0.5;
  p13Z_ = (p33Z * *dt * *dt) * 0.5  + p23Z * * dt + p13Z;

  p21Z_ = p21Z + (*dt * *dt *(p23Z + *dt * p33Z)) * 0.5 + *dt * p31Z + *dt * (p22Z + *dt * p32Z);
  p22Z_ = p22Z + *dt * p32Z + *dt * w2Z + *dt * (p23Z + *dt * p33Z);
  p23Z_ = p23Z + *dt * p33Z;

  p31Z_ = (p33Z * *dt * *dt) * 0.5 + p32Z * *dt + p31Z;
  p32Z_ = p32Z + *dt * p33Z;
  p33Z_ = p33Z + *dt * w3Z;

  p11X = p11X_;
  p12X = p12X_;
  p13X = p13X_;

  p21X = p21X_;
  p22X = p22X_;
  p23X = p23X_;

  p31X = p31X_;
  p32X = p32X_;
  p33X = p33X_;

  p11Y = p11Y_;
  p12Y = p12Y_;
  p13Y = p13Y_;

  p21Y = p21Y_;
  p22Y = p22Y_;
  p23Y = p23Y_;

  p31Y = p31Y_;
  p32Y = p32Y_;
  p33Y = p33Y_;

  p11Z = p11Z_;
  p12Z = p12Z_;
  p13Z = p13Z_;

  p21Z = p21Z_;
  p22Z = p22Z_;
  p23Z = p23Z_;

  p31Z = p31Z_;
  p32Z = p32Z_;
  p33Z = p33Z_;


  currentEstIndex++;

  lagIndex++;
  if (currentEstIndex == LAG_SIZE){
    currentEstIndex = 0;
  }
  if (lagIndex == LAG_SIZE){
    lagIndex = 0;
  }
  /*
  currentEstIndex_z++;
   
   lagIndex_z++;
   if (currentEstIndex_z == LAG_SIZE_Z){
   currentEstIndex_z = 0;
   }
   if (lagIndex_z == LAG_SIZE_Z){
   lagIndex_z = 0;
   }*/

}

void openIMU::GPSKalUpdate(){

  static float temp1,temp2,temp3;
  temp1 = p11X + vXY;
  temp2 = (p11X / temp1) - 1;
  temp3 = *XRaw - XEstHist[lagIndex];
  xError = temp3;
  k1X = p11X / temp1;
  k2X = p21X / temp1;
  k3X = p31X / temp1;

  XEst += k1X * temp3;
  velX += k2X * temp3;
  accelBiasX += k3X * temp3;

  p11X_ = -p11X * temp2;
  p12X_ = -p12X * temp2;
  p13X_ = -p13X * temp2;

  p21X_ = p21X - (p11X * p21X)/(temp1);
  p22X_ = p22X - (p12X * p21X)/(temp1);
  p23X_ = p23X - (p13X * p21X)/(temp1);

  p31X_ = p31X - (p11X * p31X)/(temp1);
  p32X_ = p32X - (p12X * p31X)/(temp1);
  p33X_ = p33X - (p13X * p31X)/(temp1);

  temp1 = p11Y + vXY;
  temp2 = (p11Y / temp1) - 1;
  temp3 = *YRaw - YEstHist[lagIndex];
  yError = temp3;
  k1Y = p11Y / temp1;
  k2Y = p21Y / temp1;
  k3Y = p31Y / temp1;

  YEst += k1Y * temp3;
  velY += k2Y * temp3;
  accelBiasY += k3Y * temp3;

  p11Y_ = -p11Y * temp2;
  p12Y_ = -p12Y * temp2;
  p13Y_ = -p13Y * temp2;

  p21Y_ = p21Y - (p11Y * p21Y)/(temp1);
  p22Y_ = p22Y - (p12Y * p21Y)/(temp1);
  p23Y_ = p23Y - (p13Y * p21Y)/(temp1);

  p31Y_ = p31Y - (p11Y * p31Y)/(temp1);
  p32Y_ = p32Y - (p12Y * p31Y)/(temp1);
  p33Y_ = p33Y - (p13Y * p31Y)/(temp1);

  p11X = p11X_;
  p12X = p12X_;
  p13X = p13X_;

  p21X = p21X_;
  p22X = p22X_;
  p23X = p23X_;

  p31X = p31X_;
  p32X = p32X_;
  p33X = p33X_;

  p11Y = p11Y_;
  p12Y = p12Y_;
  p13Y = p13Y_;

  p21Y = p21Y_;
  p22Y = p22Y_;
  p23Y = p23Y_;

  p31Y = p31Y_;
  p32Y = p32Y_;
  p33Y = p33Y_;

}


void openIMU::BaroKalUpdate(){
  static float temp1,temp2,temp3;
  temp1 = p11Z + vZ;
  temp2 = (p11Z / temp1) - 1;
  temp3 = *ZRaw - ZEst;
  //temp3 = *ZRaw - ZEstHist[lagIndex_z];
  k1Z = p11Z / temp1;
  k2Z = p21Z / temp1;
  k3Z = p31Z / temp1;

  ZEst += k1Z * temp3;
  velZ += k2Z * temp3;
  accelBiasZ += k3Z * temp3;

  p11Z_ = -p11Z * temp2;
  p12Z_ = -p12Z * temp2;
  p13Z_ = -p13Z * temp2;

  p21Z_ = p21Z - (p11Z * p21Z)/(temp1);
  p22Z_ = p22Z - (p12Z * p21Z)/(temp1);
  p23Z_ = p23Z - (p13Z * p21Z)/(temp1);

  p31Z_ = p31Z - (p11Z * p31Z)/(temp1);
  p32Z_ = p32Z - (p12Z * p31Z)/(temp1);
  p33Z_ = p33Z - (p13Z * p31Z)/(temp1);


  p11Z = p11Z_;
  p12Z = p12Z_;
  p13Z = p13Z_;

  p21Z = p21Z_;
  p22Z = p22Z_;
  p23Z = p23Z_;

  p31Z = p31Z_;
  p32Z = p32Z_;
  p33Z = p33Z_;

}


void openIMU::InitialQuat(){
  //calculate the pitch and roll from the accelerometer
  pitch = ToDeg(atan2(-1 * *ax,sqrt(*ay * *ay + *az * *az)));
  roll = ToDeg(atan2(*ay,*az));

  //tilt compensate the compass
  float xMag = (*mx * cos(ToRad(pitch))) + (*mz * sin(ToRad(pitch)));
  float yMag = -1 * ((*mx * sin(ToRad(roll))  * sin(ToRad(pitch))) + (*my * cos(ToRad(roll))) - (*mz * sin(ToRad(roll)) * cos(ToRad(pitch))));

  //calculate the yaw from the tilt compensated compass
  yaw = ToDeg(atan2(yMag,xMag));
  if (yaw < 0){
    yaw += 360;
  }


  //calculate the rotation matrix
  float cosPitch = cos(ToRad(pitch));
  float sinPitch = sin(ToRad(pitch));

  float cosRoll = cos(ToRad(roll));
  float sinRoll = sin(ToRad(roll));

  float cosYaw = cos(ToRad(yaw));
  float sinYaw = sin(ToRad(yaw));

  //need the transpose of the rotation matrix
  float r11 = cosPitch * cosYaw;
  float r21 = cosPitch * sinYaw;
  float r31 = -1.0 * sinPitch;

  float r12 = -1.0 * (cosRoll * sinYaw) + (sinRoll * sinPitch * cosYaw);
  float r22 = (cosRoll * cosYaw) + (sinRoll * sinPitch * sinYaw);
  float r32 = sinRoll * cosPitch;

  float r13 = (sinRoll * sinYaw) + (cosRoll * sinPitch * cosYaw);
  float r23 = -1.0 * (sinRoll * cosYaw) + (cosRoll * sinPitch * sinYaw);
  float r33 = cosRoll * cosPitch;

  //convert to quaternion
  q0 = 0.5 * sqrt(1 + r11 + r22 + r33);
  q1 = (r32 - r23)/(4 * q0);
  q2 = (r13 - r31)/(4 * q0);
  q3 = (r21 - r12)/(4 * q0);

  //normalize the quaternion
  recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void openIMU::AHRSStart(void){
  squareSum = *ax * *ax + *ay * *ay + *az * *az;
  qDot1 = 0.5f * (-q1 * *gx - q2 * *gy - q3 * *gz);
  qDot2 = 0.5f * (q0 * *gx + q2 * *gz - q3 * *gy);
  qDot3 = 0.5f * (q0 * *gy - q1 * *gz + q3 * *gx);
  qDot4 = 0.5f * (q0 * *gz + q1 * *gy - q2 * *gx);
  magnitude = sqrt(squareSum);
  //if ((magnitude < 10.78) && (magnitude > 8.82)   ){  
  if ((magnitude < 11.76) && (magnitude > 7.84) ){ 
    feedBack = true;
    // Normalise accelerometer measurement

    //recipNorm = InvSqrt(squareSum);
    recipNorm = 1 / magnitude;
    *ax *= recipNorm;
    *ay *= recipNorm;
    *az *= recipNorm;     
    // Normalise magnetometer measurement
    recipNorm = InvSqrt(*mx * *mx + *my * *my + *mz * *mz);
    *mx *= recipNorm;
    *my *= recipNorm;
    *mz *= recipNorm;   
    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * *mx;
    _2q0my = 2.0f * q0 * *my;
    _2q0mz = 2.0f * q0 * *mz;
    _2q1mx = 2.0f * q1 * *mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = *mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + *mx * q1q1 + _2q1 * *my * q2 + _2q1 * *mz * q3 - *mx * q2q2 - *mx * q3q3;
    hy = _2q0mx * q3 + *my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - *my * q1q1 + *my * q2q2 + _2q2 * *mz * q3 - *my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + *mz * q0q0 + _2q1mx * q3 - *mz * q1q1 + _2q2 * *my * q3 - *mz * q2q2 + *mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;


  }
  else{
    feedBack = false;
  }

}
void openIMU::AHRSEnd(){
  if (feedBack == true){

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - *ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - *ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - *mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - *my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - *mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - *ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - *ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - *az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - *mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - *my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - *mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - *ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - *ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - *az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - *mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - *my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - *mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - *ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - *ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - *mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - *my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - *mz);
    recipNorm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= betaMag * s0;
    qDot2 -= betaMag * s1;
    qDot3 -= betaMag * s2;
    qDot4 -= betaMag * s3;
    feedBack = false;
  }
  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * *dt;
  q1 += qDot2 * *dt;
  q2 += qDot3 * *dt;
  q3 += qDot4 * *dt;

  // Normalise quaternion
  recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}
void openIMU::IMUupdate(){
  squareSum = *ax * *ax + *ay * *ay + *az * *az;
  qDot1 = 0.5f * (-q1 * *gx - q2 * *gy - q3 * *gz);
  qDot2 = 0.5f * (q0 * *gx + q2 * *gz - q3 * *gy);
  qDot3 = 0.5f * (q0 * *gy - q1 * *gz + q3 * *gx);
  qDot4 = 0.5f * (q0 * *gz + q1 * *gy - q2 * *gx);
  magnitude = sqrt(squareSum);
  if ((magnitude < 11.76) && (magnitude > 7.84)){
    feedBack = true;
    recipNorm = 1 / magnitude;
    *ax *= recipNorm;
    *ay *= recipNorm;
    *az *= recipNorm;  
    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * *ax + _4q0 * q1q1 - _2q1 * *ay;
    s1 = _4q1 * q3q3 - _2q3 * *ax + 4.0f * q0q0 * q1 - _2q0 * *ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * *az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * *ax + _4q2 * q3q3 - _2q3 * *ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * *az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * *ax + 4.0f * q2q2 * q3 - _2q2 * *ay;
    recipNorm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= betaAcc * s0;
    qDot2 -= betaAcc * s1;
    qDot3 -= betaAcc * s2;
    qDot4 -= betaAcc * s3;   
  }
  else{
    feedBack = false;
  }
  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * *dt;
  q1 += qDot2 * *dt;
  q2 += qDot3 * *dt;
  q3 += qDot4 * *dt;

  // Normalise quaternion
  recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void openIMU::AHRSupdate() {
  squareSum = *ax * *ax + *ay * *ay + *az * *az;
  qDot1 = 0.5f * (-q1 * *gx - q2 * *gy - q3 * *gz);
  qDot2 = 0.5f * (q0 * *gx + q2 * *gz - q3 * *gy);
  qDot3 = 0.5f * (q0 * *gy - q1 * *gz + q3 * *gx);
  qDot4 = 0.5f * (q0 * *gz + q1 * *gy - q2 * *gx);
  magnitude = sqrt(squareSum);
  if ((magnitude < 11.76) && (magnitude > 7.84)){
    feedBack = true;
    // Normalise accelerometer measurement

    //recipNorm = InvSqrt(squareSum);
    recipNorm = 1 / magnitude;
    *ax *= recipNorm;
    *ay *= recipNorm;
    *az *= recipNorm;     
    // Normalise magnetometer measurement
    recipNorm = InvSqrt(*mx * *mx + *my * *my + *mz * *mz);
    *mx *= recipNorm;
    *my *= recipNorm;
    *mz *= recipNorm;   
    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * *mx;
    _2q0my = 2.0f * q0 * *my;
    _2q0mz = 2.0f * q0 * *mz;
    _2q1mx = 2.0f * q1 * *mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = *mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + *mx * q1q1 + _2q1 * *my * q2 + _2q1 * *mz * q3 - *mx * q2q2 - *mx * q3q3;
    hy = _2q0mx * q3 + *my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - *my * q1q1 + *my * q2q2 + _2q2 * *mz * q3 - *my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + *mz * q0q0 + _2q1mx * q3 - *mz * q1q1 + _2q2 * *my * q3 - *mz * q2q2 + *mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - *ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - *ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - *mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - *my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - *mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - *ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - *ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - *az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - *mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - *my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - *mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - *ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - *ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - *az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - *mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - *my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - *mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - *ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - *ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - *mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - *my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - *mz);
    recipNorm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= betaMag * s0;
    qDot2 -= betaMag * s1;
    qDot3 -= betaMag * s2;
    qDot4 -= betaMag * s3;

  }
  else{
    feedBack = false;
  }
  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * *dt;
  q1 += qDot2 * *dt;
  q2 += qDot3 * *dt;
  q3 += qDot4 * *dt;

  // Normalise quaternion
  recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

}

void openIMU::GetEuler(void){
  roll= ToDeg(FastAtan2(2 * (q0 * q1 + q2 * q3),1 - 2 * (q1 * q1 + q2 * q2))) - ROLL_OFFSET;

  pitch = ToDeg(asin(2 * (q0 * q2 - q3 * q1))) - PITCH_OFFSET;

  yaw = ToDeg(FastAtan2(2 * (q0 * q3 + q1 * q2) , 1 - 2* (q2 * q2 + q3 * q3)));
  yaw -= DECLINATION;

  if (yaw < 0){
    yaw +=360;
  }

}
void openIMU::GetPitch(void){
  pitch = ToDeg(asin(2 * (q0 * q2 - q3 * q1))) - PITCH_OFFSET;
}

void openIMU::GetRoll(void){
  roll= ToDeg(FastAtan2(2 * (q0 * q1 + q2 * q3),1 - 2 * (q1 * q1 + q2 * q2))) - ROLL_OFFSET;
}

void openIMU::GetYaw(void){
  yaw = ToDeg(FastAtan2(2 * (q0 * q3 + q1 * q2) , 1 - 2* (q2 * q2 + q3 * q3)));
  yaw -= DECLINATION;
  if (yaw < 0){
    yaw +=360;
  }
}















