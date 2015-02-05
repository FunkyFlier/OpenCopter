#include "UBLOXL.h"
#include <Streaming.h>


void UBLOX::init(void){
  gpsPort.begin(38400);
  GPSState=0;
  newData = false;
}


//heading and distance
//starting coords are lat1 and lon1
//target coords are lat2 and lon2
/*void UBLOX::Heading(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2,float *bearing){
 dLon = ToRad((*lon2 - *lon1)/10000000.0);
 lat1_f = ToRad(*lat1/10000000.0);
 lat2_f= ToRad(*lat2/10000000.0);
 
 y = sin(dLon)*cos(lat2_f);
 x = cos(lat1_f)*sin(lat2_f)-sin(lat1_f)*cos(lat2_f)*cos(dLon);
 *bearing = ToDeg(fastAtan2(y,x));
 if(*bearing < 0){
 *bearing += 360;
 }
 
 }
 
 void UBLOX::Distance(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2,float *dist){
 dLon = ToRad((*lon2 - *lon1)/10000000.0);
 dLat = ToRad((*lat2 - *lat1)/10000000.0);
 lat2_f = ToRad((*lat2)/10000000.0);
 x = dLon*cos(lat2_f);
 *dist = sqrt(x*x + dLat*dLat)*6372795;
 }*/
void UBLOX::DistBearing(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2,float *distX,float *distY,float *distDirect,float *bearing){
  //using euqirectangular projection since the distances are << than the RADIUS of the earth
  deltaLat = ToRad( (*lat2 - * lat1) *0.0000001 );
  //the below line is as such to get the signs between the accelerometer and GPS position to the same sign convention
  //this will work for the north west heimsphere
  deltaLon = (ToRad( (*lon2 - * lon1) *0.0000001 ) ) * cos( ToRad((*lat2 * 0.0000001)) );
  *distX = deltaLat * RADIUS_EARTH;
  *distY = deltaLon * RADIUS_EARTH;
  *distDirect = sqrt(*distX * *distX + *distY * *distY);
  *bearing = FastAtan2(*distY,*distX);
}

void UBLOX::Monitor(){

  while (gpsPort.available() > 0){
   // Serial.println(index);
    switch (GPSState){
    case 0:
      inByte = gpsPort.read();

      if (inByte == 0xB5){
        GPSState = 1;
      }
      break;
    case 1:
      inByte = gpsPort.read();
      if (inByte == 0x62){
        GPSState = 2;
      }
      else{
        GPSState = 0;
      }
      break;
    case 2:
      inByte = gpsPort.read();
      localSumB = localSumA = inByte;
      if (inByte == 0x01){
        GPSState = 3;
      }
      else{
        GPSState = 0;
      }
      break;
    case 3://get message type
      inByte = gpsPort.read();
      localSumB += (localSumA += inByte);
      msgType = inByte;
      GPSState = 4;


      break;
    case 4://get number of bytes in message LSB
      inByte = gpsPort.read();
      localSumB += (localSumA += inByte);
      msgLengthLSB = inByte;
      index = 0;
      GPSState = 5;
      break;
    case 5://get number of bytes in message MSB
      inByte = gpsPort.read();
      localSumB += (localSumA += inByte);
      msgLengthMSB = inByte;
      msgLength = (msgLengthMSB << 8) | msgLengthLSB;
      if (msgLength > 40){
        GPSState = 0;
      }
      //Serial<<msgLength<<"*\r\n";
      index = 0;
      GPSState = 6;
      break;
    case 6://buffer in data
      inByte = gpsPort.read();
      localSumB += (localSumA += inByte);
      inBuffer[index++] = inByte;
      if (index >=49){
        GPSState = 0;
      }
      if(index == msgLength){
        GPSState = 7;
      }
      break;
    case 7://get first sum and check
      inByte = gpsPort.read();
      if (inByte == localSumA){
        GPSState = 8;
      }
      else{
        GPSState = 0;
      }
      break;
    case 8://get second sum and check
      inByte = gpsPort.read();
      if (inByte == localSumB){
        GPSState = 9;
      }
      else{
        GPSState = 0;
      }
      break;
    case 9://copy the bytes to the union
      switch(msgType){
      case 0x02:
        memcpy(&data.buffer[0],&inBuffer[0],msgLength);//LLH
        LLHFlag = true;
        break;
      case 0x12:
        memcpy(&data.buffer[28],&inBuffer[4],msgLength-4);//VELNED
        VELFlag = true;
        break;
      case 0x03:
        data.vars.gpsFix = inBuffer[4];//status
        break;
      default:
        GPSState = 0;
        break;
      }
      GPSState = 0;
      break;

    }
  }
  if (LLHFlag == true && VELFlag == true){
    LLHFlag = false;
    VELFlag = false;
    newData = true;
  }
}

















