#ifndef UBLOXL_h
#define UBLOXL_h

#include <Arduino.h>

#include "AUXMATH.h"

//#define PORTNUMBER 3
#define GPSPort Serial3


#define RADIUS_EARTH 6372795



typedef union{
  struct{
    uint32_t iTWO;//0
    int32_t lon;//4
    int32_t lat;//8
    int32_t height;//12
    int32_t hMSL;//16
    uint32_t hAcc;//20
    uint32_t vAcc;//24
    int32_t velN;//28
    int32_t velE;//32
    int32_t velD;//36
    uint32_t speed3D;//40
    uint32_t speed2D;//44
    int32_t heading;//48
    uint32_t sAcc;//52
    uint32_t cAcc;//56
    uint8_t gpsFix;
  }
  vars;
  byte buffer[61];
}
GPS_Union_t;

class UBLOX{
public:
  void init(void);
  void DistBearing(int32_t*, int32_t*, int32_t*, int32_t*,float*,float*,float*,float*);
  void Monitor(void);
  GPS_Union_t data;
  boolean newData;
private:
  uint8_t GPSState;
  int i;
  byte inByte;
  
  float deltaLon;
  float deltaLat;
  float x;
  float y;
  float lat1_f;
  float lat2_f;
  uint8_t lengthIndex,dataIndex;
  
};
#endif

