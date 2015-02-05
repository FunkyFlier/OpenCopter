#ifndef AUXMATH_h
#define AUXMATH_h

#include <Arduino.h>
//#include <Streaming.h>

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f

typedef union{
  float val;
  uint8_t buffer[4];
}
float_u;

typedef union{
  int32_t val;
  uint8_t buffer[4];
}
int32_u;

typedef union{
  uint32_t val;
  uint8_t buffer[4];
}
uint32_u;

typedef union{
  int16_t val;
  uint8_t buffer[2];
}
int16_u;

typedef union{
  uint16_t val;
  uint8_t buffer[2];
}
uint16_u;

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

static void MapVar (volatile int16_t *x, float *y, float in_min, float in_max, float out_min, float out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



#endif 








