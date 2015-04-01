#ifndef UBLOXL_h
#define UBLOXL_h
#include <Arduino.h>

#include "AUXMATH.h"

//#define PORTNUMBER 3
//#define gpsPort Serial3
#define MSG_LEN 92//only using UBX-NAV-PVT 0x01 0x07 class and id

#define RADIUS_EARTH 6372795
#define gpsPort Serial3
#define Port0 Serial


typedef union{
  struct{
    uint32_t iTWO;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t gpsFix;
    uint8_t flags;
    uint8_t rsvd1;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDop;
    uint8_t rsvd2;
    uint8_t rsvd3;
    uint8_t rsvd4;
    uint8_t rsvd5;
    uint8_t rsvd6;
    uint8_t rsvd7;
    int32_t headVeh;
    uint8_t rsvd8;
    uint8_t rsvd9;
    uint8_t rsvd10;
    uint8_t rsvd11;
  }
  vars;
  byte buffer[92];
}
GPS_Union_t;

class UBLOX{
public:
  void init(void);
  void DistBearing(int32_t*, int32_t*, int32_t*, int32_t*,float*,float*,float*,float*);
  void Monitor(void);
  GPS_Union_t data;
  volatile bool newData;
  bool LLHFlag,VELFlag;
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
  uint16_t msgLength;
  uint8_t index,msgLengthLSB,msgLengthMSB,msgType,inBuffer[92],localSumA,localSumB;
  
  
};
#endif


