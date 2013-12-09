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

  //set the feedback gain and initial quaternion
  twoKp = twoKpDef;
  twoKi = twoKiDef;

  integralFBx = 0.0f;
  integralFBy = 0.0f;
  integralFBz = 0.0f;

  q0 = 1;
  q1 = 0;
  q2 = 0;
  q3 = 0;
  //Kalman stuff
  XEst = 0;
  velX = 0;
  p11 = 1.0;
  p12 = 0;
  p21 = 0;
  p22 = 1.0;

  YEst = 0;
  velY = 0;
  p13 = 1.0;
  p14 = 0;
  p23 = 0;
  p24 = 1.0;

  velZ = 0;
  ZEst = 0;
  p15 = 1.0;
  p16 = 0;
  p25 = 0;
  p26 = 1.0;
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
  //remember gravity is sensed with the sign negated - that is why it is added into the equation instead of subtracted
  inertialX =  (2.0f * (*sax * (0.5f - q2q2 - q3q3) + *say * (q1q2 - q0q3) + *saz * (q1q3 + q0q2)) );
  inertialY = 2.0 * (*sax * (q1q2 + q0q3) + *say * (0.5f - q1q1 - q3q3) + *saz * (q2q3 - q0q1));
  //inertialZ = -1.0 * ((2 * (*sax * (q1q3 - q0q2) + *say * (q2q3 + q0q1) + *saz * (q0q0 - 0.5 + q3q3) )) + 9.815 );
  inertialZ = -1.0 * ((2 * (*sax * (q1q3 - q0q2) + *say * (q2q3 + q0q1) + *saz * (q0q0 - 0.5 + q3q3) )) - gravityOffSet);
  //Kalman filter stuff - makes more sense looking at it in matrix form
  velX += inertialX * *dt;
  XEst += velX * *dt;
  p11 = p11 + *dt * p21 + *dt * w_ + *dt * (p12 + *dt * p22);
  p12 = p12 + *dt * p22;
  p21 = p21 + *dt * p22;
  p22 = p22 + *dt * w_;

  velY += inertialY * *dt;
  YEst += velY * *dt;
  p13 = p13 + *dt * p23 + *dt * w_ + *dt * (p14 + *dt * p24);
  p14 = p14 + *dt * p24;
  p23 = p23 + *dt * p24;
  p24 = p24 + *dt * w_;

  velZ += inertialZ * *dt;
  ZEst += velZ * *dt;
  p15 = p15 + *dt * p25 + *dt * w_ + *dt * (p16 + *dt * p26);
  p16 = p16 + *dt * p26;
  p25 = p25 + *dt * p26;
  p26 = p26 + *dt * w_;

}

void openIMU::GPSKalUpdate(){
  static float temp;
  temp = p11 + v1_;
  k1 = p11 / temp;
  k2 = p21 / temp;
  velX += k2 * (*XRaw - XEst);
  XEst += k1 * (*XRaw - XEst);
  p22 = p22 - k2 * p12;//run P21 and P22 first because they depend on the pervious values for p11 and p21
  p21 = p21 - k2 * p11;
  p11 = -p11*(k1 - 1);
  p12 = -p12*(k1 - 1);

  temp = p13 + v1_;
  k3 = p13 / temp;
  k4 = p23 / temp;
  velY += k4 * (*YRaw - YEst);
  YEst += k3 * (*YRaw - YEst);
  p24 = p24 - k4 * p14;//run P21 and P22 first because they depend on the pervious values for p11 and p21
  p23 = p23 - k4 * p13;
  p13 = -p13*(k3 - 1);
  p14 = -p14*(k3 - 1);

}
void openIMU::BaroKalUpdate(){
  static float temp3;
  temp3 = p15 + v2_;
  k5 = p15 / temp3;
  k6 = p25 / temp3;
  velZ += k6 * (*ZRaw - ZEst);
  ZEst += k5 * (*ZRaw - ZEst);
  p26 = p26 - k6 * p16;//run P21 and P22 first because they depend on the pervious values for p11 and p21
  p25 = p25 - k6 * p15;
  p15 = -p15*(k5 - 1);
  p16 = -p16*(k5 - 1);

}


void openIMU::InitialQuat(){//fix - something is broken here
  //also call after acc init so the LPF has time to smooth the data
  int8_t sign;
  //this function does not have to be called
  //first calculate the pitch and roll from the accelerometer
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
}


void openIMU::AHRSupdate() {
  squareSum = *ax * *ax + *ay * *ay + *az * *az;

  magnitude = sqrt(squareSum);
  if ((magnitude < 10.78) && (magnitude > 8.82)){
    // Normalise accelerometer measurement
    recipNorm = InvSqrt(squareSum);
    *ax *= recipNorm;
    *ay *= recipNorm;
    *az *= recipNorm;     

    // Normalise magnetometer measurement
    recipNorm = InvSqrt(*mx * *mx + *my * *my + *mz * *mz);
    *mx *= recipNorm;
    *my *= recipNorm;
    *mz *= recipNorm;   

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

    // Reference direction of Earth's magnetic field
    //pretty neat that there are at least three different ways to generate the rotation matrix from quaternions
    hx = 2.0f * (*mx * (0.5f - q2q2 - q3q3) + *my * (q1q2 - q0q3) + *mz * (q1q3 + q0q2));
    hy = 2.0f * (*mx * (q1q2 + q0q3) + *my * (0.5f - q1q1 - q3q3) + *mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (*mx * (q1q3 - q0q2) + *my * (q2q3 + q0q1) + *mz * (0.5f - q1q1 - q2q2));


    /*hx = 2.0f * (*mx * (q0q0 - 0.5f + q1q1) + *my * (q1q2 - q0q3) + *mz * (q1q3 + q0q2));
     hy = 20.f * (*mx * (q1q2 + q0q3) + *my * (q0q0 - 0.5 + q2q2) + *mz * (q2q3 - q0q1) );
     bx = sqrt(hx * hx + hy * hy);
     bz = 2.0f * (*mx * (q1q3 - q0q2) + *my * (q2q3 + q0q1) + *mz * (q0q0 -0.5 + q3q3));*/


    /*hx = *mx * (q0q0 + q1q1 - q2q2 - q3q3) + *my * 2 * (q1q2 - q0q3) + *mz * 2 * (q0q2 + q1q3);
     hy = *mx * 2 * (q1q2 + q0q3) + *my * (q0q0 - q1q1 + q2q2 - q3q3) + *mz * 2 * (q2q3 - q0q1);
     bz = *mx * 2 * ( q1q3 - q0q2) + *my * 2 * (q0q1 + q2q3) + *mz * (q0q0 - q1q1 - q2q2 + q3q3); 
     bx = sqrt(hx * hx + hy * hy);*/



    // Estimated direction of gravity and magnetic field
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = 0.5 - q1q1 - q2q2;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex = (*ay * halfvz - *az * halfvy) + (*my * halfwz - *mz * halfwy);
    halfey = (*az * halfvx - *ax * halfvz) + (*mz * halfwx - *mx * halfwz);
    halfez = (*ax * halfvy - *ay * halfvx) + (*mx * halfwy - *my * halfwx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * *dt;	// integral error scaled by Ki
      integralFBy += twoKi * halfey * *dt;
      integralFBz += twoKi * halfez * *dt;
      *gx += integralFBx;	// apply integral feedback
      *gy += integralFBy;
      *gz += integralFBz;
    }
    else {
      integralFBx = 0.0f;	// prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    *gx += twoKp * halfex;
    *gy += twoKp * halfey;
    *gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  *gx *= (0.5f * *dt);		// pre-multiply common factors
  *gy *= (0.5f * *dt);
  *gz *= (0.5f * *dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * *gx - qc * *gy - q3 * *gz);
  q1 += (qa * *gx + qc * *gz - q3 * *gy);
  q2 += (qa * *gy - qb * *gz + q3 * *gx);
  q3 += (qa * *gz + qb * *gy - qc * *gx); 

  // Normalise quaternion
  recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void openIMU::GetEuler(void){
  //this function provied the Euler angles
  //Euler angles are easier to incorporte into PID control than the quaternion

  roll= ToDeg(FastAtan2(2 * (q0 * q1 + q2 * q3),1 - 2 * (q1 * q1 + q2 * q2))) - ROLL_OFFSET;
  pitch = ToDeg(asin(2 * (q0 * q2 - q3 * q1))) - PITCH_OFFSET;
  yaw = ToDeg(FastAtan2(2 * (q0 * q3 + q1 * q2) , 1 - 2* (q2 * q2 + q3 * q3)));
  yaw -= DECLINATION;
  if (yaw < 0){
    yaw +=360;
  }
  //these values are used for the rotation matrices for heading free and loiter
  //consider moving to the 50Hz loop since they will not be needed at 100Hz
  //cosYaw = cos(yaw);
  //sinYaw = sin(yaw);



}



