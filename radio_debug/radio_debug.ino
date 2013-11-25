#include "openIMUL.h"
#include "UBLOXL.h"
#include <Streaming.h>

//telemetery defines
#define RADIO_BUF_SIZE 256
#define radio Serial2
#define NUM_WAY_POINTS 0x14

#define ACTIVE 0
#define STABLE 1
#define HEAD_FREE 2
#define LOITER 3
#define WAYPOINT 4
#define RTB 5

#define WP_HOLD 0
#define WP_TRAVEL 1
#define WP_END 2
#define WP_RTB 3
#define WP_FAIL_RTB 4

uint8_t currentWayPointNumber = 0, inputWayPointNumber = 0,wayPointState = 0;
boolean startFlight = false;
int32_t latTarget,lonTarget;//check to make sure this is correct for loiter
boolean endOfWPCheck = false; 
boolean RTBFailSafe = false;
boolean RTBFlag = false;

float targetAltitude,targetVelAlt,actualAltitude,distToAlt;

float n;
openIMU imu(&n,&n,&n,&n,&n,&n,&n,&n,&n,&n,&n,&n,&n,&n,&n,&n);


//GPS related vars
typedef union{
  struct{
    int32_t lat;
    int32_t lon;
    int32_t alt;
  }
  coord;
  uint8_t buffer[12];
}
WayPoint_t;

WayPoint_t homeBase;
WayPoint_t wayPoints[20];
WayPoint_t loiterWP;//rename

UBLOX gps;

//protocol related vars 
typedef union{
  struct{
    float pitch;//0
    float roll;//4
    float yaw;//8
    int32_t lattitude;//12
    int32_t longitude;//16
    float baroAltitude;//20
    int32_t gpsAltitude;//24
    int32_t velN;//28
    int32_t velE;//32
    int32_t velD;//36
    int32_t gpsHeading;//40
    uint32_t _3DSpeed;//44
    uint32_t groundSpeed;//48

    float kp_pitch_rate;//52
    float ki_pitch_rate;//56
    float kd_pitch_rate;//60
    float fc_pitch_rate;//64

    float kp_roll_rate;//68
    float ki_roll_rate;//72
    float kd_roll_rate;//76
    float fc_roll_rate;//80

    float kp_yaw_rate;//84
    float ki_yaw_rate;//88
    float kd_yaw_rate;//92
    float fc_yaw_rate;//96

    float kp_pitch_attitude;//100
    float ki_pitch_attitude;//104
    float kd_pitch_attitude;//108
    float fc_pitch_attitude;//112

    float kp_roll_attitude;//116
    float ki_roll_attitude;//120
    float kd_roll_attitude;//124
    float fc_roll_attitude;//128

    float kp_yaw_attitude;//132
    float ki_yaw_attitude;//136
    float kd_yaw_attitude;//140
    float fc_yaw_attitude;//144

    float kp_altitude_position;//148
    float ki_altitude_position;//152
    float kd_altitude_position;//156
    float fc_altitude_position;//160

    float kp_altitude_rate;//164
    float ki_altitude_rate;//168
    float kd_altitude_rate;//172
    float fc_altitude_rate;///176

    float kp_loiter_pos_x;//180
    float ki_loiter_pos_x;//184
    float kd_loiter_pos_x;//188
    float fc_loiter_pos_x;//192

    float kp_loiter_rate_x;//196
    float ki_loiter_rate_x;//200
    float kd_loiter_rate_x;//204
    float fc_loiter_rate_x;//208

    float kp_loiter_pos_y;//212
    float ki_loiter_pos_y;//216
    float kd_loiter_pos_y;//220
    float fc_loiter_pos_y;//224

    float kp_loiter_rate_y;//228
    float ki_loiter_rate_y;//232
    float kd_loiter_rate_y;//236
    float fc_loiter_rate_y;//240

    float kp_waypoint_position;//244
    float ki_waypoint_position;//248
    float kd_waypoint_position;//252
    float fc_waypoint_position;//256

    float kp_waypoint_velocity;//260
    float ki_waypoint_velocity;//264
    float kd_waypoint_velocity;//268
    float fc_waypoint_velocity;//272

    uint8_t gpsFix;//276
    uint8_t flightMode;//277
  }
  v;
  uint8_t buffer[278];
}
FC_Data_type;//think of better name

FC_Data_type d;

uint8_t numBytesIn,numBytesOut,inByteCount,inByteRadio,inBuffer[RADIO_BUF_SIZE],bufferIndexRadio=0,parseIndex=0,cmdIndex=0,cmdState=0,varIndex=0,cmdBuffer[32];
boolean newRadio = false;
uint32_t radioTimer;
uint8_t inputSum=0,inputDoubleSum=0,outputSum=0,outputDoubleSum=0,packetTemp[2];
uint16_t packetNumberLocalOrdered,packetNumberRemoteOrdered,packetNumberLocalUn,packetNumberRemoteUn;
boolean ordered = false;
boolean handShake = false;
uint8_t temp;

float degreeGyroX,degreeGyroY,degreeGyroZ;

float pitchSetPoint;
float rollSetPoint;
float yawSetPoint;
float rateSetPointX;    
float rateSetPointY;
float rateSetPointZ;
float adjustmentX;
float adjustmentY;
float adjustmentZ; 
float altitudeSetPoint;
float throttleAdjustment;    
float velSetPointX,velSetPointY;
float yawInput;
float zero = 0.0;
float pitchSetPointTX,rollSetPointTX;
float distToWayPoint,targetVelWayPoint;
float speed2D_MPS;
float posXError,posYError;
float setPointX,setPointY;

//for debugging the protocol remove later
char hex[17]="0123456789ABCDEF";

//new vars---------------------------------------
boolean tuningTransmit = false;
uint8_t numOfItems;
uint8_t itemBuffer[30];
uint32_t refreshMillis;
uint32_t tuningTimer;

typedef union{
  float num;
  uint8_t buffer[4];
}
FLOAT_UNION_t;

FLOAT_UNION_t outFloat;

uint8_t lenOfTuningDataPacket;
uint8_t refreshRate;

//-----------------------------------------------


void setup(){
  Serial.begin(115200);
  Serial<<"Start\r\n";
  radio.begin(112500);
  InitVars();
  HandShake();
}

void loop(){

  Radio();
  TuningTransmitter();


  pitchSetPoint = 0.35;
  imu.pitch = 45.0;
  rollSetPoint = -.35;
  imu.roll = -45.1;
  yawSetPoint = 359.123;
  imu.yaw = 0.13;
  rateSetPointX = 10.10;
  degreeGyroX = 9.89;
  rateSetPointY = 6.10;
  degreeGyroY = 6.89;
  rateSetPointZ = 1.10;
  degreeGyroZ = 19.89;
  targetAltitude =123.56;
  actualAltitude = -1234.56;
  targetVelAlt = -1.0;
  imu.velZ = 9.34;
  distToWayPoint = -1.23;
  targetVelWayPoint = -3.45;
  speed2D_MPS = 1.0;
  posXError = 987.345;
  velSetPointX = 3425.345;
  imu.velX = 198.234;
  posYError = 567.90;
  velSetPointY = 382.67345;
  imu.velY = 100.3008;
}

void TuningTransmitter(){
  if (tuningTransmit == true){
    if (millis() - tuningTimer >= refreshMillis){
      outputSum = 0;
      outputDoubleSum = 0;
      radio.write(0xAA);
      lenOfTuningDataPacket = numOfItems * 5 + 2;
      radio.write(lenOfTuningDataPacket);
      radio.write(0x47);
      outputSum += 0x47;
      outputDoubleSum += outputSum;
      radio.write(numOfItems);
      outputSum += numOfItems;
      outputDoubleSum += outputSum;

      tuningTimer = millis();
      for (uint8_t i = 0; i < numOfItems; i++){
        switch (itemBuffer[i]){
        case 0x00:
          outFloat.num = pitchSetPoint;
          break;
        case 0x01:
          outFloat.num = imu.pitch;
          break;
        case 0x02:
          outFloat.num = rollSetPoint;
          break;
        case 0x03:
          outFloat.num = imu.roll;
          break;
        case 0x04:
          outFloat.num = yawSetPoint;
          break;
        case 0x05:
          outFloat.num = imu.yaw;
          break;
        case 0x06:
          outFloat.num = rateSetPointX;
          break;
        case 0x07:
          outFloat.num = degreeGyroX;
          break;
        case 0x08:
          outFloat.num = rateSetPointY;
          break;
        case 0x09:
          outFloat.num = degreeGyroY;
          break;
        case 0x0A:
          outFloat.num = rateSetPointZ;
          break;
        case 0x0B:
          outFloat.num = degreeGyroZ;
          break;
        case 0x0C:
          outFloat.num = targetAltitude;
          break;
        case 0x0D:
          outFloat.num = actualAltitude;
          break;
        case 0x0E:
          outFloat.num = targetVelAlt;
          break;
        case 0x0F:
          outFloat.num = imu.velZ;
          break;
        case 0x10:
          outFloat.num = distToWayPoint;
          break;
        case 0x11:
          outFloat.num = targetVelWayPoint;
          break;
        case 0x12:
          outFloat.num = speed2D_MPS;
          break;
        case 0x13:
          outFloat.num = posXError;
          break;
        case 0x14:
          outFloat.num = velSetPointX;
          break;
        case 0x15:
          outFloat.num = imu.velX;
          break;
        case 0x16:
          outFloat.num = posYError;
          break;
        case 0x17:
          outFloat.num = velSetPointY;
          break;
        case 0x18:
          outFloat.num = imu.velY;
          break;
        default:
          break;

        }
        radio.write(itemBuffer[i]);
        outputSum += itemBuffer[i];
        outputDoubleSum += outputSum;
        for(uint8_t j = 0; j < 4; j++){
          radio.write(outFloat.buffer[j]);
          outputSum += outFloat.buffer[j];
          outputDoubleSum += outputSum;
        }
      }
      radio.write(outputSum);
      radio.write(outputDoubleSum);
    }
  }

}





