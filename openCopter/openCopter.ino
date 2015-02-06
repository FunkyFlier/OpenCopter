//
//#include <MemoryFree.h>
#include <EEPROM.h>
#include <SPI.h>
#include "I2CL.h"
#include "openIMUL.h"
#include "PID.h"
#include <Streaming.h>
#include <AUXMATH.h>
#include "UBLOXL.h"

//#define ROT_45

#define RADIUS_EARTH 6372795

#define CEILING 6
#define FLOOR 2
#define TAKE_OFF_ALT 3

#define LAND_VEL -0.2
//LED defines
#define RED 38
#define YELLOW 40
#define GREEN 42
//ROM defines
enum CalibrationFlags {
  RC_FLAG,
  ACC_FLAG,
  MAG_FLAG,
  GAINS_FLAG
};


#define CLIMB 0
#define TRAVEL 1
#define DESCEND 2


//general SPI defines
#define READ 0x80
#define WRITE 0x00
#define MULTI 0x40
#define SINGLE 0x00

//gyro defines - ST L3G2
#define L3G_CTRL_REG1 0x20
#define L3G_CTRL_REG2 0x21
#define L3G_CTRL_REG3 0x22
#define L3G_CTRL_REG4 0x23
#define L3G_CTRL_REG5 0x24
#define L3G_OUT_X_L 0x28




//acc defines - Analog Devices ADXL345
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAX0 0x32



//mag defines ST LSM303DLHC - will work with the HMC5883L
#define MAG_ADDRESS 0x1E
#define LSM303_CRA_REG (uint8_t)0x00 
#define LSM303_CRB_REG 0x01
#define LSM303_MR_REG 0x02
#define LSM303_OUT_X_H 0x03



//barometer defines
#define BMP085_ADDRESS 0x77
#define POLL_RATE 20
/*#define OSS 0x00
 #define CONV_TIME 5*/
/*#define OSS 0x01
 #define CONV_TIME 8*/
/*#define OSS 0x02
 #define CONV_TIME 14*/
#define OSS 0x03
#define CONV_TIME 27
//however digitalWrite will work when using SPI 
#define GyroSSOutput() DDRL |= 1<<0 
#define GyroSSHigh() PORTL |= 1<<0 
#define GyroSSLow() PORTL &= ~(1<<0)

#define D22Output() DDRA |= 1<<0 
#define D22High() PORTA |= 1<<0 
#define D22Low() PORTA &= ~(1<<0)
#define D22Toggle() PORTA ^= (1<<0);

#define D23Output() DDRA |= 1<<1 
#define D23High() PORTA |= 1<<1 
#define D23Low() PORTA &= ~(1<<1)
#define D23Toggle() PORTA ^= (1<<1);

#define D24Output() DDRA |= 1<<2 
#define D24High() PORTA |= 1<<2 
#define D24Low() PORTA &= ~(1<<2)
#define D24Toggle() PORTA ^= (1<<2);

#define D25Output() DDRA |= 1<<3 
#define D25High() PORTA |= 1<<3 
#define D25Low() PORTA &= ~(1<<3)
#define D25Toggle() PORTA ^= (1<<3);

#define D26Output() DDRA |= 1<<4 
#define D26High() PORTA |= 1<<4 
#define D26Low() PORTA &= ~(1<<4)
#define D26Toggle() PORTA ^= (1<<4);

#define D27Output() DDRA |= 1<<5 
#define D27High() PORTA |= 1<<5 
#define D27Low() PORTA &= ~(1<<5)
#define D27Toggle() PORTA ^= (1<<5);

#define D28Output() DDRA |= 1<<6 
#define D28High() PORTA |= 1<<6 
#define D28Low() PORTA &= ~(1<<6)
#define D28Toggle() PORTA ^= (1<<6);

#define D29Output() DDRA |= 1<<7 
#define D29High() PORTA |= 1<<7 
#define D29Low() PORTA &= ~(1<<7)
#define D29Toggle() PORTA ^= (1<<7);

#define AccSSOutput() DDRL |= 1<<1 
#define AccSSHigh() PORTL |= 1<<1 
#define AccSSLow() PORTL &= ~(1<<1)

//control defines 
#define HH_ON 0
#define HH_OFF 1



#define MAX_Z_RATE 3.0f
#define MIN_Z_RATE -3.0f


//motor defines
#define FREQ 400
#define PRESCALE 8
#define PERIOD ((F_CPU/PRESCALE/FREQ) - 1)



#define Motor1WriteMicros(x) OCR3B = x * 2//motor 1 is attached to pin2
#define Motor2WriteMicros(x) OCR3C = x * 2//motor 2 is attached to pin3
#define Motor3WriteMicros(x) OCR3A = x * 2//motor 3 is attached to pin5
#define Motor4WriteMicros(x) OCR4A = x * 2//motor 4 is attached to pin6
#define Motor5WriteMicros(x) OCR4B = x * 2//motor 1 is attached to pin7
#define Motor6WriteMicros(x) OCR4C = x * 2//motor 2 is attached to pin8
#define Motor7WriteMicros(x) OCR1A = x * 2//motor 3 is attached to pin11
#define Motor8WriteMicros(x) OCR1B = x * 2//motor 4 is attached to pin12
//radio control defines
//RC defines
enum RC_Types {
  DSMX = 1, SBUS, RC};

enum motorControlStates{
  HOLD,
  TO,
  FLIGHT,
  LANDING,
};

enum RC_Chan {
  THRO, AILE, ELEV, RUDD, GEAR, AUX1, AUX2, AUX3};

enum loiterStates{
  LOITERING,
  RCINPUT,
  LAND,
  WAIT};

enum FlightStates {
  RATE,
  ATT,
  L0,
  L1,
  L2,
  FOLLOW,
  WP,
  RTB
};

enum Int16s {
  GYRO_X,
  GYRO_Y,
  GYRO_Z,
  ACC_X,
  ACC_Y,
  ACC_Z,
  MAG_X,
  MAG_Y,
  MAG_Z,
  NUM_SATS
};

enum BYTES {
  FLIGHT_MODE,
  RTB_STATE,
  Z_LOIT,
  XY_LOIT,
  GPS_FS,
  DR_FLAG,
  MOTOR_STATE


};

enum Int32s {
  PRESSURE_,
  HB_LAT,
  HB_LON,
  HB_ALT,
  H_DOP,
  LAT_,
  LON_

};

enum Floats {
  GYRO_X_DEG,
  GYRO_Y_DEG,
  GYRO_Z_DEG,
  ACC_X_FILT,
  ACC_Y_FILT,
  ACC_Z_FILT,
  MAG_X_CALIB,
  MAG_Y_CALIB,
  MAG_Z_CALIB,
  RAW_X,
  RAW_Y,
  RAW_Z,
  PITCH_,
  ROLL_,
  YAW_,
  QUAT_0,
  QUAT_1,
  QUAT_2,
  QUAT_3,
  X_EST,
  Y_EST,
  Z_EST,
  VEL_X,
  VEL_Y,
  VEL_Z,
  KP_PITCH_RATE_,
  KI_PITCH_RATE_,
  KD_PITCH_RATE_,
  FC_PITCH_RATE_,
  KP_ROLL_RATE_,
  KI_ROLL_RATE_,
  KD_ROLL_RATE_,
  FC_ROLL_RATE_,
  KP_YAW_RATE_,
  KI_YAW_RATE_,
  KD_YAW_RATE_,
  FC_YAW_RATE_,
  KP_PITCH_ATT_,
  KI_PITCH_ATT_,
  KD_PITCH_ATT_,
  FC_PITCH_ATT_,
  KP_ROLL_ATT_,
  KI_ROLL_ATT_,
  KD_ROLL_ATT_,
  FC_ROLL_ATT_,
  KP_YAW_ATT_,
  KI_YAW_ATT_,
  KD_YAW_ATT_,
  FC_YAW_ATT_,
  KP_ALT_POS_,
  KI_ALT_POS_,
  KD_ATL_POS_,
  FC_ALT_POS_,
  KP_ALT_VEL_,
  KI_ALT_VEL_,
  KD_ALT_VEL_,
  FC_ALT_VEL_,
  MUL_ALT_VEL_,
  KP_LOIT_X_POS_,
  KI_LOIT_X_POS_,
  KD_LOIT_X_POS_,
  FC_LOIT_X_POS_,
  KP_LOIT_X_VEL_,
  KI_LOIT_X_VEL_,
  KD_LOIT_X_VEL_,
  FC_LOIT_X_VEL_,
  KP_LOIT_Y_POS_,
  KI_LOIT_Y_POS_,
  KD_LOIT_Y_POS_,
  FC_LOIT_Y_POS_,
  KP_LOIT_Y_VEL_,
  KI_LOIT_Y_VEL_,
  KD_LOIT_Y_VEL_,
  FC_LOIT_Y_VEL_,
  KP_WP_POS_,
  KI_WP_POS_,
  KD_WP_POS_,
  FC_WP_POS_,
  KP_WP_VEL_,
  KI_WP_VEL_,
  KD_WP_VEL_,
  FC_WP_VEL_,
  KP_CT_,
  KI_CT_,
  KD_CT_,
  FC_CT_,
  MAG_DEC_,
  RATE_SP_X,
  RATE_SP_Y,
  RATE_SP_Z,
  ADJ_X,
  ADJ_Y,
  ADJ_Z,
  PITCH_SP,
  ROLL_SP,
  YAW_SP,
  X_TARG,
  Y_TARG,
  Z_TARG,
  VEL_SP_X,
  VEL_SP_Y,
  VEL_SP_Z,
  TILT_X,
  TILT_Y,
  THRO_ADJ,
  PITCH_SP_TX,
  ROLL_SP_TX,
  DIST_TO_WP,
  TARGET_VEL_WP,
  POS_ERR,
  ACC_CIR,
  DR_VEL_X,
  DR_VEL_Y,
  DR_POS_X,
  DR_POS_Y,
  MOTOR_CMD_1,
  MOTOR_CMD_2,
  MOTOR_CMD_3,
  MOTOR_CMD_4,
  PITCH_OFF,
  ROLL_OFF,
  INERTIAL_X,
  INERTIAL_Y,
  INERTIAL_Z

};





#define RADIO_BUF_SIZE 256
#define NUM_WAY_POINTS 0x14
#define Port0 Serial 
#define RCSigPort Serial1
#define Port2 Serial2
#define gpsPort Serial3

float_u *floatPointerArray[125];

int16_u *int16PointerArray[10];

int32_u *int32PointerArray[7];

uint8_t *bytePointerArray[7];

int16_u gyroX,gyroY,gyroZ,accX,accY,accZ,magX,magY,magZ;



//barometer variables
int32_u pressure;
short temperature;
uint32_t baroTimer;
int pressureState;
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
unsigned char msb;
unsigned char lsb;
unsigned char xlsb;
long x1;
long x2;
long x3;
long b3;
long b5;
long b6;
long p;
unsigned long b4;
unsigned long b7;
unsigned int ut;
unsigned long up;
uint32_t baroPollTimer;
boolean newBaro = false;
float pressureRatio;
int baroCount;
float baroSum;
long pressureInitial; 

//IMU related vars
int32_t gyroSumX,gyroSumY,gyroSumZ;
int16_t gyroOffsetX,gyroOffsetY,gyroOffsetZ;
float radianGyroX,radianGyroY,radianGyroZ;
float_u degreeGyroX,degreeGyroY,degreeGyroZ;
float_u filtAccX,filtAccY,filtAccZ;
float_u calibMagX,calibMagY,calibMagZ;//needs to be a float so the vector can be normalized
float shiftedMagX,shiftedMagY,shiftedMagZ;
float_u shiftedAccX,shiftedAccY,shiftedAccZ;
float_u scaledAccX,scaledAccY,scaledAccZ;
float accToFilterX,accToFilterY,accToFilterZ;


//GPS related vars
typedef struct{
  int32_u lat;
  int32_u lon;
  int32_u alt;
}
WayPoint_t;

WayPoint_t homeBase;
WayPoint_t wayPoints[20];
WayPoint_t loiterWP;//rename


UBLOX gps;
//TinyGPS gps;
volatile boolean GPSDetected;
float distToCraft;
float headingToCraft;


//RC related vars
uint8_t readState,inByte,byteCount,channelNumber;
volatile uint8_t rcType;
uint32_t frameTime;
boolean detected = false;
volatile boolean newRC = false;
boolean frameStart = true;
boolean frameValid = false;

uint8_t spekBuffer[14];

uint16_t bufferIndex=0;

volatile int16_t rawRCVal[8];
static uint16_t throttleCommand;


uint8_t currentPinState = 0;
uint8_t previousPinState = 0;
uint8_t changeMask = 0;
uint8_t lastPinState = 0;
uint16_t currentTime = 0;
uint16_t timeDifference = 0;
uint16_t changeTime[8];
uint8_t sBusData[25];

int16_t minRCVal[8],centerRCVal[8];
volatile int16_t RCValue[8];
int16_t RCOffset[8];
float RCScale[8];


//timers and DTs
uint32_t imuTimer,GPSTimer;
uint32_t generalPurposeTimer;
float imuDT;
float GPSDT;

//protocol related vars 

int32_u lattitude;//12
int32_u longitude;//16

float_u kp_pitch_rate;//52
float_u ki_pitch_rate;//56
float_u kd_pitch_rate;//60
float_u fc_pitch_rate;//64

float_u kp_roll_rate;//68
float_u ki_roll_rate;//72
float_u kd_roll_rate;//76
float_u fc_roll_rate;//80

float_u kp_yaw_rate;//84
float_u ki_yaw_rate;//88
float_u kd_yaw_rate;//92
float_u fc_yaw_rate;//96

float_u kp_pitch_attitude;//100
float_u ki_pitch_attitude;//104
float_u kd_pitch_attitude;//108
float_u fc_pitch_attitude;//112

float_u kp_roll_attitude;//116
float_u ki_roll_attitude;//120
float_u kd_roll_attitude;//124
float_u fc_roll_attitude;//128

float_u kp_yaw_attitude;//132
float_u ki_yaw_attitude;//136
float_u kd_yaw_attitude;//140
float_u fc_yaw_attitude;//144

float_u kp_altitude_position;//148
float_u ki_altitude_position;//152
float_u kd_altitude_position;//156
float_u fc_altitude_position;//160

float_u kp_altitude_velocity;//164
float_u ki_altitude_velocity;//168
float_u kd_altitude_velocity;//172
float_u fc_altitude_velocity;///176
float_u mul_altitude_velocity;

float_u kp_loiter_pos_x;//180
float_u ki_loiter_pos_x;//184
float_u kd_loiter_pos_x;//188
float_u fc_loiter_pos_x;//192

float_u kp_loiter_velocity_x;//196
float_u ki_loiter_velocity_x;//200
float_u kd_loiter_velocity_x;//204
float_u fc_loiter_velocity_x;//208

float_u kp_loiter_pos_y;//212
float_u ki_loiter_pos_y;//216
float_u kd_loiter_pos_y;//220
float_u fc_loiter_pos_y;//224

float_u kp_loiter_velocity_y;//228
float_u ki_loiter_velocity_y;//232
float_u kd_loiter_velocity_y;//236
float_u fc_loiter_velocity_y;//240

float_u kp_waypoint_position;//244
float_u ki_waypoint_position;//248
float_u kd_waypoint_position;//252
float_u fc_waypoint_position;//256

float_u kp_waypoint_velocity;//260
float_u ki_waypoint_velocity;//264
float_u kd_waypoint_velocity;//268
float_u fc_waypoint_velocity;//272

float_u kp_cross_track;//276
float_u ki_cross_track;//280
float_u kd_cross_track;//284
float_u fc_cross_track;//288

//float_u declination;//292

uint8_t gpsFix;//296
uint8_t flightMode;//297



float_u outFloat;


//control related vars
float_u pitchSetPoint;
float_u rollSetPoint;
float_u yawSetPoint;
float_u rateSetPointX;    
float_u rateSetPointY;
float_u rateSetPointZ;
float_u adjustmentX;
float_u adjustmentY;
float_u adjustmentZ; 
float_u throttleAdjustment;    
float_u velSetPointX,velSetPointY;
float_u zTarget,velSetPointZ,actualAltitude;
float yawInput;
float zero = 0.0;
float_u pitchSetPointTX,rollSetPointTX;
float_u distToWayPoint,targetVelWayPoint;
float speed2D_MPS;
float_u tiltAngleX,tiltAngleY;
uint8_t HHState = 1;
float_u motorCommand1,motorCommand2,motorCommand3,motorCommand4;

boolean integrate = false;
boolean enterState = true;

boolean calcYaw;


//failsafe related vars
volatile boolean failSafe = false;
boolean toggle;
volatile boolean watchDogStartCount;
volatile uint32_t watchDogFailSafeCounter,RCFailSafeCounter;

float_u xTarget,yTarget;
uint8_t calibrationFlags;

uint8_t outFloatIndex;

boolean allCalibrated = false;
boolean calibrationMode = false;

float magOffSetX;
float magOffSetY;
float magOffSetZ;
float magWInv00;
float magWInv01;
float magWInv02;
float magWInv10;
float magWInv11;
float magWInv12;
float magWInv20;
float magWInv21;
float magWInv22;
//float accXScalePos, accYScalePos, accZScalePos, accXScaleNeg, accYScaleNeg, accZScaleNeg;
float accXScale, accYScale, accZScale, accXOffset, accYOffset, accZOffset;


float gravSum;
float gravAvg;

float scaledMagX,scaledMagY,scaledMagZ,magNorm,magToFiltX,magToFiltY,magToFiltZ;


//x and y setpoints are rotated into pitch and roll set points


uint32_t _400HzTimer;
float controlBearing;
uint8_t XYLoiterState,ZLoiterState;
int16_t rcDifference;
uint32_t waitTimer;
uint8_t RTBState;



volatile boolean gpsUpdate = false;
uint32_t gpsFixAge;


float startingX,startingY,jumpDistX,jumpDistY,homeBaseXOffset=0,homeBaseYOffset=0;
#define POSITION_ERROR_LIMIT 2.5f
#define ACC_RATE 1.098f
#define DR_PERIOD 1.0 * 1000000 
#define DR_FS_PERIOD 5 * 1000000 
float drTimer;
float_u positionError,accCircle;
boolean drFlag = false;
float_u drVelX,drVelY,drPosX,drPosY;
float halfDTSq;
boolean GPSDenial = false;
uint32_t gpsUpdateTimer;
int16_u numSats;
int32_u hDop;

//radio protocol vars
uint16_t localPacketNumberOrdered,localPacketNumberUn,remotePacketNumberOrdered,remotePacketNumberUn,packetTemp[2];
uint32_t radioTimer;
boolean handShake;
uint8_t handShakeState,rxSum,rxDoubleSum,txSum,txDoubleSum,radioByte,
packetLength,numRXBytes,radioState,numRXbytes,typeNum,cmdNum,
itemBuffer[255],itemIndex,temp,hsNumItems,lsNumItems,hsList[40],lsList[40],liveDataBuffer[200],hsRequestNumber,lsRequestNumber,hsListIndex,lsListIndex;
uint32_t hsMillis,lsMillis,hsTXTimer,lsTXTimer;
boolean offsetFlag,sendCalibrationData,hsTX,lsTX,tuningTrasnmitOK;


boolean gpsFailSafe = true,txFailSafe,telemFailSafe,battFailSafe;

boolean trimMode,setTrim,trimComplete,autoMaticReady;
uint8_t throttleCheckFlag;
boolean modeSelect = false;
uint8_t switchPositions,clearTXRTB;
uint8_t previousFlightMode,motorState;
float initialYaw;
uint32_t tuningItemIndex;
uint32_t radioLimitTimer;

Print* radioPrint;
Stream* radioStream;

boolean USBFlag = false,saveGainsFlag = false;
uint16_t j_;

float_u xSlopeAcc,ySlopeAcc,zSlopeAcc;
float_u xSlopeMag,ySlopeMag,zSlopeMag;
float_u xSlopeGyro,ySlopeGyro,zSlopeGyro;

int16_u xAccOffset,yAccOffset;
int16_u calibTempAcc,calibTempMag,initialTemp;

uint32_t ledTimer;

volatile uint32_t start,width;
float pingDistCentimeters,pingDistMeters;
volatile boolean newPing = false;
float_u baroZ,ultraSonicRange;

float_u pingDistOutput,widthOutput;

//boolean pingFlag;
uint8_t pingFlag;

int16_u throOutput;

float_u gpsAlt;

byte GPSFlag;
byte baroFlag;
byte magFlag;

uint32_t loopCount;
float_u floatLat, floatLon;
float_u outAccX,outAccY,outAccZ,GPSCourse,GPSVel,velN,velE,velD;

uint32_t romWriteDelayTimer;
uint32_t _400Time;
float_u gpsX,gpsY;

float unFiltZ;
float_u zMeas;
volatile boolean baroCorrect;


float_u altX,altY,altZ;
float_u tempOutput;
uint32_t pingTimer;
float baroRate,baroDT,prevBaro,pingRate,pingDT,prevPing;
float_u baroVel,baroAlt,velZMeas,pingVel;
int16_t lagAmount,tempX,tempY;

uint8_t loopCount_;

float rotGyroX,rotGyroY,rotAccX,rotAccY;

uint32_t loopTime;
//constructors //fix the dts
openIMU imu(&radianGyroX,&radianGyroY,&radianGyroZ,&accToFilterX,&accToFilterY,&accToFilterZ,&filtAccX.val,&filtAccY.val,&filtAccZ.val,
&magToFiltX,&magToFiltY,&magToFiltZ,&gpsX.val,&gpsY.val,&zMeas.val,&velN.val,&velE.val,&velZMeas.val,&imuDT);
//openIMU imu(&radianGyroX,&radianGyroY,&radianGyroZ,&accToFilterX,&accToFilterY,&accToFilterZ,&filtAccX.val,&filtAccY.val,&filtAccZ.val,&magToFiltX,&magToFiltY,&magToFiltZ,&gpsX.val,&gpsY.val,&baroZ.val,&zero,&zero,&imuDT);

PID PitchRate(&rateSetPointY.val,&degreeGyroY.val,&adjustmentY.val,&integrate,&kp_pitch_rate.val,&ki_pitch_rate.val,&kd_pitch_rate.val,&fc_pitch_rate.val,&imuDT,400,400);
PID RollRate(&rateSetPointX.val,&degreeGyroX.val,&adjustmentX.val,&integrate,&kp_roll_rate.val,&ki_roll_rate.val,&kd_roll_rate.val,&fc_roll_rate.val,&imuDT,400,400);
PID YawRate(&rateSetPointZ.val,&degreeGyroZ.val,&adjustmentZ.val,&integrate,&kp_yaw_rate.val,&ki_yaw_rate.val,&kd_yaw_rate.val,&fc_yaw_rate.val,&imuDT,400,400);

PID PitchAngle(&pitchSetPoint.val,&imu.pitch.val,&rateSetPointY.val,&integrate,&kp_pitch_attitude.val,&ki_pitch_attitude.val,&kd_pitch_attitude.val,&fc_pitch_attitude.val,&imuDT,800,800);
PID RollAngle(&rollSetPoint.val,&imu.roll.val,&rateSetPointX.val,&integrate,&kp_roll_attitude.val,&ki_roll_attitude.val,&kd_roll_attitude.val,&fc_roll_attitude.val,&imuDT,800,800);
YAW YawAngle(&yawSetPoint.val,&imu.yaw.val,&rateSetPointZ.val,&integrate,&kp_yaw_attitude.val,&ki_yaw_attitude.val,&kd_yaw_attitude.val,&fc_yaw_attitude.val,&imuDT,800,800);

PID LoiterXPosition(&xTarget.val,&imu.XEst.val,&velSetPointX.val,&integrate,&kp_loiter_pos_x.val,&ki_loiter_pos_x.val,&kd_loiter_pos_x.val,&fc_loiter_pos_x.val,&imuDT,1,1);
PID LoiterXVelocity(&velSetPointX.val,&imu.velX.val,&tiltAngleX.val,&integrate,&kp_loiter_velocity_x.val,&ki_loiter_velocity_x.val,&kd_loiter_velocity_x.val,&fc_loiter_velocity_x.val,&imuDT,30,30);

PID LoiterYPosition(&yTarget.val,&imu.YEst.val,&velSetPointY.val,&integrate,&kp_loiter_pos_y.val,&ki_loiter_pos_y.val,&kd_loiter_pos_y.val,&fc_loiter_pos_y.val,&imuDT,1,1);
PID LoiterYVelocity(&velSetPointY.val,&imu.velY.val,&tiltAngleY.val,&integrate,&kp_loiter_velocity_y.val,&ki_loiter_velocity_y.val,&kd_loiter_velocity_y.val,&fc_loiter_velocity_y.val,&imuDT,30,30);

PID AltHoldPosition(&zTarget.val,&imu.ZEstUp.val,&velSetPointZ.val,&integrate,&kp_altitude_position.val,&ki_altitude_position.val,&kd_altitude_position.val,&fc_altitude_position.val,&imuDT,1.5,1.5);
ALT AltHoldVelocity(&velSetPointZ.val,&imu.velZUp.val,&throttleAdjustment.val,&integrate,&kp_altitude_velocity.val,&ki_altitude_velocity.val,&kd_altitude_velocity.val,&fc_altitude_velocity.val,&imuDT,800,800,&mul_altitude_velocity.val);

PID WayPointPosition(&zero,&distToWayPoint.val,&targetVelWayPoint.val,&integrate,&kp_waypoint_position.val,&ki_waypoint_position.val,&kd_waypoint_position.val,&fc_waypoint_position.val,&GPSDT,10,10);
PID WayPointRate(&targetVelWayPoint.val,&speed2D_MPS,&pitchSetPoint.val,&integrate,&kp_waypoint_velocity.val,&ki_waypoint_velocity.val,&kd_waypoint_velocity.val,&fc_waypoint_velocity.val,&imuDT,45,45);
//PID CrossTrack 
//2D speed from GPS best idea where or use the vel from the estimator?



/*void pause(){
 while(digitalRead(22)==0){
 }//wait for the toggle
 delay(500);
 }*/

/*char hex[17]="0123456789ABCDEF";
 
 void ShowHex(byte convertByte){
 Port0 << hex[(convertByte >>4) & 0x0F];
 Port0 << hex[convertByte & 0x0F]<<"\r\n";
 }*/

void setup(){
  pinMode(RED,OUTPUT);
  pinMode(YELLOW,OUTPUT);
  pinMode(GREEN,OUTPUT);
  //digitalWrite(RED,HIGH);
  pinMode(13,OUTPUT);
  Port0.begin(115200);
  Port2.begin(115200);
  Port2.write(0x0D);
  delay(250);
  Port2.print("B");
  AssignPointerArray();


  //----------------------------
  //ROMFlagsCheck();
  //DEBUG_DUMP();
  //while(1){}

  //----------------------------  

  GyroSSOutput();
  AccSSOutput();
  GyroSSHigh();
  AccSSHigh();


  D22Output();
  //pinMode(22,INPUT);
  D23Output();
  D24Output();
  D25Output();
  D26Output();
  D27Output();
  D28Output();
  D29Output();

  CheckESCFlag();

  DetectRC();

  _200HzISRConfig();


  ROMFlagsCheck();



  CalibrateESC();
  MotorInit();
  delay(3500);//this allows the telemetry radios to connect before trying the handshake
  if (handShake == false){
    radioStream = &Port2;
    radioPrint = &Port2;
    HandShake();

    if (handShake == false){
      USBFlag = true;
      radioStream = &Port0;
      radioPrint = &Port0;
      HandShake();
    }
  }
  I2c.begin();
  I2c.setSpeed(1);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);   


  if (calibrationMode == true){
    BaroInit();
    AccInit();
    MagInit();
    CalibrateSensors();  
    ROMFlagsCheck();
  }

  ModeSelect();
  Arm();//move the rudder to the right to begin calibration
  BaroInit();
  GyroInit();
  AccInit();
  MagInit();
  GetInitialQuat();
  GPSStart();
  CheckTXPositions();

  imu.DECLINATION = ToRad(3.3);
  imu.COS_DEC = cos(imu.DECLINATION);
  imu.SIN_DEC = sin(imu.DECLINATION);

  gpsFailSafe = false;
  imuDT = 0.01;
  imuTimer = micros();
  _400HzTimer = micros();
  generalPurposeTimer = millis();
  currentTime = micros();
  watchDogStartCount = true;
  digitalWrite(RED,1);
  digitalWrite(YELLOW,1);
  digitalWrite(GREEN,1);
  digitalWrite(13,1);
}

void loop(){

  _400HzTask();
  loopTime = micros();
  if (loopTime - imuTimer >= 10000){

    //imu.lagAmount = (uint8_t)fc_cross_track.val;
    loopCount_ = 0;
    GetGyro();
    _400HzTask();
    GetMag();
    //imu.magFlag =1;
    _400HzTask();

    imu.AHRSupdate();
    _400HzTask();
    imu.GenerateRotationMatrix();
    _400HzTask();
    imu.GetEuler();
    _400HzTask();
    imu.GetInertial();
    _400HzTask();
    imu.Predict();
    _400HzTask();
    imu.UpdateLagIndex();
    _400HzTask();  
    FlightSM();
    _400HzTask();

    if (GPSDetected == true){
      gps.Monitor();
    }
    _400HzTask();
    if (gps.newData == true){

      gps.newData = false;
      GPSFlag = true;

      floatLat.val = (gps.data.vars.lat) * 0.0000001;
      floatLon.val = (gps.data.vars.lon) * 0.0000001;
      gpsAlt.val = gps.data.vars.height * 0.001;
      velN.val = gps.data.vars.velN * 0.01;
      velE.val = gps.data.vars.velE * 0.01;
      velD.val = gps.data.vars.velD * 0.01;
      gps.DistBearing(&homeBase.lat.val,&homeBase.lon.val,&gps.data.vars.lat,&gps.data.vars.lon,&gpsX.val,&gpsY.val,&distToCraft,&headingToCraft);
      if (gps.data.vars.gpsFix != 3){
        gpsFailSafe = true;
      }
      imu.CorrectGPS();

    }

    PollPressure();
    _400HzTask();
    if (newBaro == true){
      newBaro = false;
      GetAltitude(&pressure.val,&pressureInitial,&baroAlt.val);
      baroDT = (millis() - baroTimer) * 0.001;
      baroTimer = millis();
      baroZ.val  =  baroZ.val * 0.85 + baroAlt.val * 0.15;
      if (baroDT <= 0.1){
        baroRate = (baroZ.val - prevBaro) / baroDT;

      }
      else{
        baroRate = 0;
      }

      baroVel.val = baroVel.val * 0.5 + baroRate * 0.5;
      prevBaro = baroZ.val;
      velZMeas.val = baroVel.val;
      zMeas.val = baroZ.val;
      imu.CorrectAlt();
    }
    _400HzTask();
    if (newRC == true){
      newRC = false;
      ProcessChannels();
      GetSwitchPositions();
      RCFailSafeCounter = 0;
    }  
    _400HzTask();
    if (RCFailSafeCounter >= 200 || failSafe == true){
      txFailSafe = true;
      TIMSK5 = (0<<OCIE5A);
      digitalWrite(13,LOW);
      digitalWrite(RED,LOW);
      digitalWrite(YELLOW,LOW);
      digitalWrite(GREEN,LOW);
      Motor1WriteMicros(1000);//set the output compare value
      Motor2WriteMicros(1000);
      Motor3WriteMicros(1000);
      Motor4WriteMicros(1000);
      Motor5WriteMicros(1000);
      Motor6WriteMicros(1000);
      //Motor7WriteMicros(1000);
      //Motor8WriteMicros(1000);
      if (failSafe == true){
        digitalWrite(RED,HIGH);
      }
      while(1){
        digitalWrite(YELLOW,HIGH);
        if (RCFailSafeCounter >= 200 ){
          digitalWrite(GREEN,LOW);
        }
        delay(500);
        digitalWrite(YELLOW,LOW);
        if (RCFailSafeCounter >= 200 ){
          digitalWrite(GREEN,HIGH);
        }
        delay(500);
      }
    }


    _400HzTask();
    if (flightMode > 0){
      PitchAngle.calculate();
      RollAngle.calculate();
      if (calcYaw == true){
        YawAngle.calculate();
      }
    }
    _400HzTask();
    PitchRate.calculate();
    RollRate.calculate();
    YawRate.calculate();
    _400HzTask();
    MotorHandler();
    tuningTrasnmitOK = true;
    _400HzTask();

  }


  _400HzTask();
  if (handShake == true){
    Radio();

    if (tuningTrasnmitOK == true){
      TuningTransmitter();

      tuningTrasnmitOK = false;
      
    }

  }  
  _400HzTask();
  watchDogFailSafeCounter = 0;
}

void _400HzTask(){
  _400Time = micros();
  if ( _400Time -_400HzTimer  >=2500 ){
    _400HzTimer = _400Time;
    GetAcc();
  }
}



void FlightSM(){

  switch(flightMode){
  case RATE:
    if (enterState == true){
      enterState = false;
      if (previousFlightMode != RATE && previousFlightMode != ATT){
        throttleCheckFlag = true;
      }
      digitalWrite(13,HIGH);
      digitalWrite(RED,HIGH);
      digitalWrite(YELLOW,HIGH);
      digitalWrite(GREEN,HIGH);
    }
    TrimCheck();

    break;
  case ATT:
    if (enterState == true){
      if (previousFlightMode != RATE && previousFlightMode != ATT){
        throttleCheckFlag = true;

      }
      yawSetPoint = imu.yaw;
      enterState = false;
      digitalWrite(13,LOW);
      digitalWrite(RED,LOW);
      digitalWrite(YELLOW,LOW);
      digitalWrite(GREEN,LOW);
    }
    HeadingHold();
    TrimCheck();
    break;
  case L0:
    if (enterState == true){
      enterState = false;
      InitLoiter();
      yawSetPoint = imu.yaw;
      digitalWrite(13,HIGH);
      digitalWrite(RED,HIGH);
      digitalWrite(YELLOW,LOW);
      digitalWrite(GREEN,LOW);
    }
    HeadingHold();
    controlBearing = imu.yaw.val;
    LoiterSM();
    break;
  case L1:
    if (enterState == true){
      enterState = false;
      InitLoiter();
      yawSetPoint = imu.yaw;
      digitalWrite(13,HIGH);
      digitalWrite(RED,LOW);
      digitalWrite(YELLOW,HIGH);
      digitalWrite(GREEN,LOW);
      controlBearing = initialYaw;

    }
    HeadingHold();
    LoiterSM();
    break;
  case L2:
    if (enterState == true){
      InitLoiter();
      yawSetPoint = imu.yaw;
      digitalWrite(13,HIGH);
      digitalWrite(RED,LOW);
      digitalWrite(YELLOW,LOW);
      digitalWrite(GREEN,HIGH);

      enterState = false;
    }
    HeadingHold();
    controlBearing = headingToCraft;
    LoiterSM();
    break;
  case FOLLOW:
    digitalWrite(13,LOW);
    digitalWrite(RED,HIGH);
    digitalWrite(YELLOW,LOW);
    digitalWrite(GREEN,LOW);
    if (enterState == true){
      enterState = false;
      yawSetPoint = imu.yaw;
    }
    break;
  case WP:
    digitalWrite(13,LOW);
    digitalWrite(RED,LOW);
    digitalWrite(YELLOW,HIGH);
    digitalWrite(GREEN,LOW);
    if (enterState == true){
      enterState = false;
      yawSetPoint = imu.yaw;
    }
    break;
  case RTB:
    digitalWrite(13,LOW);
    digitalWrite(RED,LOW);
    digitalWrite(YELLOW,LOW);
    digitalWrite(GREEN,HIGH);
    if (enterState == true){
      enterState = false;
      xTarget.val = imu.XEst.val;
      yTarget.val = imu.YEst.val;
      zTarget.val = imu.ZEstUp.val + 1; 
      if (zTarget.val > CEILING){
        zTarget.val = CEILING;
      }
      if (zTarget.val < FLOOR){
        zTarget.val = FLOOR;
      }
      RTBState = CLIMB;
      yawSetPoint = imu.yaw;
    }
    HeadingHold();
    RTBStateMachine();
    break;

  }

}

void TrimCheck(){
  uint8_t j;
  if (setTrim == true){
    if (trimComplete == false){
      trimComplete = true;
      imu.pitchOffset.val = imu.rawPitch.val;
      imu.rollOffset.val = imu.rawRoll.val;
      j = 0;
      for(uint8_t i = 73; i <=76; i++){
        EEPROM.write(i,imu.pitchOffset.buffer[j++]);
      }
      j = 0;
      for(uint8_t i = 77; i <=80; i++){
        EEPROM.write(i,imu.rollOffset.buffer[j++]);
      }
      EEPROM.write(382,0xAA);
    }
  }
}

void InitLoiter(){

  if (previousFlightMode != L1 && previousFlightMode != L2 && previousFlightMode != L0){
    if (motorState == LANDING){
      motorState = FLIGHT;
    }
    throttleCheckFlag = true;
    throttleAdjustment.val = 0;
    xTarget.val = imu.XEst.val;
    yTarget.val = imu.YEst.val;
    zTarget.val = imu.ZEstUp.val; 
    if (zTarget.val < FLOOR){
      zTarget.val = FLOOR;
    } 
    if (zTarget.val > CEILING){
      zTarget.val = FLOOR;
    }
    LoiterXPosition.reset();
    LoiterXVelocity.reset();
    LoiterYPosition.reset();
    LoiterYVelocity.reset();
    AltHoldPosition.reset();
    AltHoldVelocity.reset();
  }

}

void RTBStateMachine(){
  if (motorState == HOLD || motorState == TO){
    return;
  }
  switch(RTBState){
  case CLIMB:
    LoiterCalculations();
    RotatePitchRoll(&imu.yaw.val,&zero,&tiltAngleX.val,&tiltAngleY.val,&pitchSetPoint.val,&rollSetPoint.val);
    AltHoldPosition.calculate();
    AltHoldVelocity.calculate();
    if (imu.ZEstUp.val >= (zTarget.val - 0.1) ){

      RTBState = TRAVEL;
      xTarget.val = 0 + homeBaseXOffset;
      yTarget.val = 0 + homeBaseYOffset;
    }
    if (gpsFailSafe == true){
      velSetPointZ.val = LAND_VEL;
      RTBState = DESCEND;
    }

    break;
  case TRAVEL:
    //make special PID loops for the loiter positions
    /*if (fabs(xTarget.val - imu.XEst.val) < 1.5){
     velSetPointX.val = 0;
     }
     else{
     
     }
     if (fabs(yTarget.val - imu.YEst.val) < 1.5){
     velSetPointY.val = 0;
     }
     else{
     
     }*/
    LoiterXPosition.calculate();
    LoiterYPosition.calculate();
    if (velSetPointX.val > 0.25){
      velSetPointX.val = 0.25;
    }
    if (velSetPointX.val < -0.25){
      velSetPointX.val = -0.25;
    }
    if (velSetPointY.val > 0.25){
      velSetPointY.val = 0.25;
    }
    if (velSetPointY.val < -0.25){
      velSetPointY.val = -0.25;
    }
    LoiterXVelocity.calculate();
    tiltAngleX.val *= -1.0;
    LoiterYVelocity.calculate();
    RotatePitchRoll(&imu.yaw.val,&zero,&tiltAngleX.val,&tiltAngleY.val,&pitchSetPoint.val,&rollSetPoint.val);
    AltHoldPosition.calculate();
    AltHoldVelocity.calculate();
    //if (fabs(imu.XEst.val - homeBaseXOffset) < 1 && fabs(imu.YEst.val - homeBaseYOffset) < 1){
    if (fabs(imu.XEst.val + homeBaseXOffset) < 0.25 && fabs(imu.YEst.val + homeBaseYOffset) < 0.25){
      velSetPointZ.val = LAND_VEL;
      RTBState = DESCEND;
      motorState = LANDING;
    }
    if (gpsFailSafe == true){
      velSetPointZ.val = LAND_VEL;
      RTBState = DESCEND;
      motorState = LANDING;
    }
    break;
  case DESCEND:
    if (gpsFailSafe == true){
      pitchSetPoint.val = pitchSetPointTX.val;
      rollSetPoint.val = rollSetPointTX.val;

      if (txFailSafe == true){
        pitchSetPoint.val = 0;
        rollSetPoint.val = 0;
      }
    }
    else{
      LoiterCalculations();
      RotatePitchRoll(&imu.yaw.val,&zero,&tiltAngleX.val,&tiltAngleY.val,&pitchSetPoint.val,&rollSetPoint.val);
    }
    //velSetPointZ.val = LAND_VEL;
    AltHoldVelocity.calculate();
    //motorState = LANDING;
    break;
  }  
}

void LoiterCalculations(){
  LoiterXPosition.calculate();
  LoiterYPosition.calculate();
  LoiterXVelocity.calculate();
  tiltAngleX.val *= -1.0;
  LoiterYVelocity.calculate();
}


























































































