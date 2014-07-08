#ifndef AUXMATH_h
#define AUXMATH_h

#include <Arduino.h>
#include <Streaming.h>

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f
/*#define a_ 0.0489708f//fc = 6.5Hz
 #define b_ 0.0011991f
 #define c_ 0.0000147f*/
/*#define a_ 0.0636620f//fc = 5Hz
#define b_ 0.0020264f
#define c_ 0.0000323f*/
/*#define a_ 0.0707355f//fc = 4.5Hz
#define b_ 0.0025018f
#define c_ 0.0000442f*/
/*#define a_ 0.0795775f//fc = 4.0Hz
#define b_ 0.0031663f
#define c_ 0.0000630f*/
/*#define a_ 0.0848826f//fc = 3.75Hz
#define b_ 0.0036025f
#define c_ 0.0000764f*/
#define a_ 0.1061033f//fc = 3.0Hz
#define b_ 0.0056290f
#define c_ 0.0001493f
//#define a_ 0.1273240f//fc = 2.5Hz
//#define b_ 0.0081057f
//#define c_ 0.0002580f
/*#define a_ 0.1414711f//fc = 2.25Hz
#define b_ 0.0100070f
#define c_ 0.0003539f*/
/*#define a_ 0.1591549f//fc = 2.0Hz
#define b_ 0.0126651f
#define c_ 0.0005039f*/
/*#define a_ 0.2122066f//fc = 1.5Hz
#define b_ 0.0225158f
#define c 0.0011945f*/
#define _2bydt1 700.0f
#define _2bydt2 490000.0f
#define _2bydt3 343000000.0f
#define D  (float)(1.0 + a_ * _2bydt1 + b_ * _2bydt2 + c_ * _2bydt3) 
#define E  (float)(3.0 + a_ * _2bydt1 - b_ * _2bydt2 - 3.0 * c_ * _2bydt3)
#define F  (float)(3.0 - a_ * _2bydt1 - b_ * _2bydt2 + 3.0 * c_ * _2bydt3)
#define G  (float)(1.0 - a_ * _2bydt1 + b_ * _2bydt2 - c_ * _2bydt3)

static int8_t n1,n2,n3;
//static int16_t inBufferX[3],inBufferY[3],inBufferZ[3];
static float inBufferX[3],inBufferY[3],inBufferZ[3];
static float outBufferX[3],outBufferY[3],outBufferZ[3];
static int8_t filterIndex;

static char hex[17]="0123456789ABCDEF";

static float FastAtan2( float y, float x)
{
  static float atan;
  static float z;
  if ( x == 0.0f )
  {
    if ( y > 0.0f ) return PIBY2_FLOAT;
    if ( y == 0.0f ) return 0.0f;
    return -PIBY2_FLOAT;
  }
  //atan;
  z = y / x;
  if ( fabs( z ) < 1.0f )
  {
    atan = z/(1.0f + 0.28f*z*z);
    if ( x < 0.0f )
    {
      if ( y < 0.0f ) return atan - PI_FLOAT;
      return atan + PI_FLOAT;
    }
  }
  else
  {
    atan = PIBY2_FLOAT - z/(z*z + 0.28f);
    if ( y < 0.0f ) return atan - PI_FLOAT;
  }
  return atan;
}

static float InvSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5f;



  x = number * 0.5f;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}

/*static void SmoothingACC(  int16_t *raw,  float *smooth){
 *smooth = (*raw * (0.25)) + (*smooth * 0.75);
 //*smooth = (float)*raw;
 }*/
 static void SmoothingBaro(  float *raw,  float *smooth){
   *smooth = (*raw * (0.12)) + (*smooth * 0.88);
 
 }

//static void Filter( int16_t *rawX, int16_t *rawY, int16_t *rawZ, float *smoothX, float *smoothY, float *smoothZ){
static void Filter( float *rawX, float *rawY, float *rawZ, float *smoothX, float *smoothY, float *smoothZ){

  /*n1 = filterIndex;
   n2 = filterIndex-1;
   n3 = filterIndex-2;
   if (n2 < 0 ){
   n2 += 3;
   }
   if (n3 < 0){
   n3 += 3;
   }
   
   *smoothX = ( (float)*rawX + 3.0 * (float)inBufferX[n1] + 3.0 * (float)inBufferX[n2] + (float)inBufferX[n3] - E * outBufferX[n1] - F * outBufferX[n2] - G * outBufferX[n3] ) / D;
   *smoothY = ( (float)*rawY + 3.0 * (float)inBufferY[n1] + 3.0 * (float)inBufferY[n2] + (float)inBufferY[n3] - E * outBufferY[n1] - F * outBufferY[n2] - G * outBufferY[n3] ) / D;
   *smoothZ = ( (float)*rawZ + 3.0 * (float)inBufferZ[n1] + 3.0 * (float)inBufferZ[n2] + (float)inBufferZ[n3] - E * outBufferZ[n1] - F * outBufferZ[n2] - G * outBufferZ[n3] ) / D;*/

  *smoothX = ( *rawX + 3.0 * inBufferX[0] + 3.0 * inBufferX[1] + inBufferX[2] - E * outBufferX[0] - F * outBufferX[1] - G * outBufferX[2] ) / D;
  *smoothY = ( *rawY + 3.0 * inBufferY[0] + 3.0 * inBufferY[1] + inBufferY[2] - E * outBufferY[0] - F * outBufferY[1] - G * outBufferY[2] ) / D;
  *smoothZ = ( *rawZ + 3.0 * inBufferZ[0] + 3.0 * inBufferZ[1] + inBufferZ[2] - E * outBufferZ[0] - F * outBufferZ[1] - G * outBufferZ[2] ) / D;

  inBufferX[2] = inBufferX[1];
  inBufferX[1] = inBufferX[0];
  inBufferX[0] = *rawX;

  outBufferX[2] = outBufferX[1];
  outBufferX[1] = outBufferX[0];
  outBufferX[0] = *smoothX;

  inBufferY[2] = inBufferY[1];
  inBufferY[1] = inBufferY[0];
  inBufferY[0] = *rawY;

  outBufferY[2] = outBufferY[1];
  outBufferY[1] = outBufferY[0];
  outBufferY[0] = *smoothY;

  inBufferZ[2] = inBufferZ[1];
  inBufferZ[1] = inBufferZ[0];
  inBufferZ[0] = *rawZ;

  outBufferZ[2] = outBufferZ[1];  
  outBufferZ[1] = outBufferZ[0];  
  outBufferZ[0] = *smoothZ;  



  /*filterIndex++;
   if (filterIndex == 3){
   filterIndex = 0;
   }  
   
   inBufferX[filterIndex] = *rawX;
   inBufferY[filterIndex] = *rawY;
   inBufferZ[filterIndex] = *rawZ;
   
   outBufferX[filterIndex] = *smoothX;
   outBufferY[filterIndex] = *smoothY;
   outBufferZ[filterIndex] = *smoothZ;*/
}

static void MapVar (volatile uint16_t *x, volatile float *y, float in_min, float in_max, float out_min, float out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void ShowHex(byte convertByte){
  Serial << hex[(convertByte >>4) & 0x0F];
  Serial << hex[convertByte & 0x0F]<<"\r\n";
}


#endif 








