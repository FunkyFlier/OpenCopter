#include "UBLOXL.h"
#include <Streaming.h>

void UBLOX::init(void){
  GPSPort.begin(115200);
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
    while (GPSPort.available() > 0){
      switch (GPSState){
      case 0:
        inByte = GPSPort.read();
        if (inByte == 0xB5){
          GPSState = 1;
        }
        break;
      case 1:
        inByte = GPSPort.read();
        if (inByte == 0x62){
          GPSState = 2;
        }
        else{
          GPSState = 0;
        }
        break;
      case 2:
        inByte = GPSPort.read();
        if (inByte == 0x01){
          GPSState = 3;
          lengthIndex = 0;
          dataIndex = 0;
        }
        else{
          GPSState = 0;
        }
        break;
      case 3:
        inByte = GPSPort.read();
        if (inByte == 0x02){
          GPSState = 4;
        }
        else{
          if (inByte == 0x12){
            GPSState = 5;
          }
          else{
            if(inByte == 0x03){
              GPSState = 6;
            }
            else{
              GPSState = 0;
            }
          }
        }
        break;
      case 4://POSLLH
        if (lengthIndex < 2 && dataIndex == 0){
          inByte = GPSPort.read();
          lengthIndex++;
          break;
        }
        if (lengthIndex == 2 && dataIndex < 28){
          inByte = GPSPort.read();
          data.buffer[dataIndex] = inByte;
          dataIndex++;
          break;
        }
        if (lengthIndex == 2 && dataIndex == 28){
          GPSState = 0;
          newData = true;
          break;
        }

        break;
      case 5://VELNED
        if (lengthIndex < 2 && dataIndex == 0){
          inByte = GPSPort.read();
          lengthIndex++;
          break;
        }
        if (lengthIndex == 2 && dataIndex < 4){
          inByte = GPSPort.read();
          data.buffer[dataIndex] = inByte;
          dataIndex++;
          if (dataIndex == 4){
            dataIndex = 28;
          }
          break;
        }
        if (lengthIndex == 2 && dataIndex < 60){
          inByte = GPSPort.read();
          data.buffer[dataIndex] = inByte;
          dataIndex++;
          break;
        }
        if (lengthIndex == 2 && dataIndex == 60){
          GPSState = 0;
          newData = true;
          break;
        }
        break;
      case 6://
        if (lengthIndex < 2 && dataIndex == 0){
          inByte = GPSPort.read();
          lengthIndex++;
          break;
        }
        if (lengthIndex == 2 && dataIndex < 4){
          inByte = GPSPort.read();
          data.buffer[dataIndex] = inByte;
          dataIndex++;
          break;
        }
        if (lengthIndex == 2 && dataIndex == 4){
          inByte = GPSPort.read();
          data.vars.gpsFix = inByte;
          dataIndex++;
          break;
        }
        if (lengthIndex == 2 && dataIndex == 5){
          GPSState = 0;
          break;
        }
        break;
      }
    }

}










