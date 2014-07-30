//
//#include <MemoryFree.h>
#include <EEPROM.h>
#include <SPI.h>
#include "I2CL.h"
#include "openIMUL.h"
#include "PID.h"
#include "SerialPort.h"
#include <Streaming.h>
#include <AUXMATH.h>
#include <TinyGPS.h>

#define FREQ_TRIG 20
#define PRESCALE_TRIG 64
#define PERIOD_TRIG ((F_CPU/PRESCALE_TRIG/FREQ_TRIG) - 1)

#define RADIUS_EARTH 6372795

#define CEILING 6
#define FLOOR 2
#define TAKE_OFF_ALT 2

#define LAND_VEL -0.75
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
#define POLL_RATE 0
/*#define OSS 0x00
 #define CONV_TIME 5*/
/*#define OSS 0x01
 #define CONV_TIME 8*/
/*#define OSS 0x02
 #define CONV_TIME 14*/
#define OSS 0x03
#define CONV_TIME 26

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


SerialPort<0,254,254> Port0;
SerialPort<1,254,254> RCSigPort;
SerialPort<2,254,254> Port2;
SerialPort<3,254,254> gpsPort;

#define RADIO_BUF_SIZE 256
#define NUM_WAY_POINTS 0x14

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
float scaledAccX,scaledAccY,scaledAccZ;
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

float_u rawX,rawY,rawZ;

//UBLOX gps;
TinyGPS gps;
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
float imuDT,GPSDT;

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
float_u yawInput;
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
float accXScalePos, accYScalePos, accZScalePos, accXScaleNeg, accYScaleNeg, accZScaleNeg;

float rateDT;

float gravSum;
float gravAvg;
float shiftedAccX,shiftedAccY,shiftedAccZ;



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


boolean gpsFailSafe,txFailSafe,telemFailSafe,battFailSafe;

boolean startMode,setTrim,trimComplete,autoMaticReady;
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
int16_u calibTempAcc,calibTempMag,initialTemp,deltaTemp;

uint32_t ledTimer;


//boolean pingFlag;
uint8_t pingFlag;

int16_u throOutput;


volatile uint32_t start,width;
volatile float pingDistCentimeters,pingDistMeters;
volatile boolean newPing = false;
float_u baroZ,ultraSonicRange;

//constructors //fix the dts
openIMU imu(&radianGyroX,&radianGyroY,&radianGyroZ,&accToFilterX,&accToFilterY,&accToFilterZ,&filtAccX.val,&filtAccY.val,&filtAccZ.val,&calibMagX.val,&calibMagY.val,&calibMagZ.val,&rawX.val,&rawY.val,&rawZ.val,&imuDT);

PID PitchRate(&rateSetPointY.val,&degreeGyroY.val,&adjustmentY.val,&integrate,&kp_pitch_rate.val,&ki_pitch_rate.val,&kd_pitch_rate.val,&fc_pitch_rate.val,&rateDT,400,400);
PID RollRate(&rateSetPointX.val,&degreeGyroX.val,&adjustmentX.val,&integrate,&kp_roll_rate.val,&ki_roll_rate.val,&kd_roll_rate.val,&fc_roll_rate.val,&rateDT,400,400);
PID YawRate(&rateSetPointZ.val,&degreeGyroZ.val,&adjustmentZ.val,&integrate,&kp_yaw_rate.val,&ki_yaw_rate.val,&kd_yaw_rate.val,&fc_yaw_rate.val,&rateDT,400,400);

PID PitchAngle(&pitchSetPoint.val,&imu.pitch.val,&rateSetPointY.val,&integrate,&kp_pitch_attitude.val,&ki_pitch_attitude.val,&kd_pitch_attitude.val,&fc_pitch_attitude.val,&imuDT,800,800);
PID RollAngle(&rollSetPoint.val,&imu.roll.val,&rateSetPointX.val,&integrate,&kp_roll_attitude.val,&ki_roll_attitude.val,&kd_roll_attitude.val,&fc_roll_attitude.val,&imuDT,800,800);
YAW YawAngle(&yawSetPoint.val,&imu.yaw.val,&rateSetPointZ.val,&integrate,&kp_yaw_attitude.val,&ki_yaw_attitude.val,&kd_yaw_attitude.val,&fc_yaw_attitude.val,&imuDT,800,800);

PID LoiterXPosition(&xTarget.val,&imu.XEst.val,&velSetPointX.val,&integrate,&kp_loiter_pos_x.val,&ki_loiter_pos_x.val,&kd_loiter_pos_x.val,&fc_loiter_pos_x.val,&imuDT,1,1);
PID LoiterXVelocity(&velSetPointX.val,&imu.velX.val,&tiltAngleX.val,&integrate,&kp_loiter_velocity_x.val,&ki_loiter_velocity_x.val,&kd_loiter_velocity_x.val,&fc_loiter_velocity_x.val,&imuDT,30,30);

PID LoiterYPosition(&yTarget.val,&imu.YEst.val,&velSetPointY.val,&integrate,&kp_loiter_pos_y.val,&ki_loiter_pos_y.val,&kd_loiter_pos_y.val,&fc_loiter_pos_y.val,&imuDT,1,1);
PID LoiterYVelocity(&velSetPointY.val,&imu.velY.val,&tiltAngleY.val,&integrate,&kp_loiter_velocity_y.val,&ki_loiter_velocity_y.val,&kd_loiter_velocity_y.val,&fc_loiter_velocity_y.val,&imuDT,30,30);

PID AltHoldPosition(&zTarget.val,&imu.ZEst.val,&velSetPointZ.val,&integrate,&kp_altitude_position.val,&ki_altitude_position.val,&kd_altitude_position.val,&fc_altitude_position.val,&imuDT,1.5,1.5);
ALT AltHoldVelocity(&velSetPointZ.val,&imu.velZ.val,&throttleAdjustment.val,&integrate,&kp_altitude_velocity.val,&ki_altitude_velocity.val,&kd_altitude_velocity.val,&fc_altitude_velocity.val,&imuDT,800,800,&mul_altitude_velocity.val);

PID WayPointPosition(&zero,&distToWayPoint.val,&targetVelWayPoint.val,&integrate,&kp_waypoint_position.val,&ki_waypoint_position.val,&kd_waypoint_position.val,&fc_waypoint_position.val,&GPSDT,10,10);
PID WayPointRate(&targetVelWayPoint.val,&speed2D_MPS,&pitchSetPoint.val,&integrate,&kp_waypoint_velocity.val,&ki_waypoint_velocity.val,&kd_waypoint_velocity.val,&fc_waypoint_velocity.val,&imuDT,45,45);


void setup(){
  pinMode(RED,OUTPUT);
  pinMode(YELLOW,OUTPUT);
  pinMode(GREEN,OUTPUT);
  //digitalWrite(RED,HIGH);
  pinMode(13,OUTPUT);
  Port0.begin(115200);
  Port2.begin(115200);
  AssignPointerArray();
  AccSSOutput();//this was moved from the init
  AccSSHigh();//if high isn't written to both devices befor config 
  GyroSSOutput();//the SPI bus will be addressing both devices 
  GyroSSHigh();
  D22Output();
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
  //throttle high will trigger this reset after calibration
  delay(3500);//this allows the telemetry radios to connect before trying the handshake
  if (handShake == false){
    Port2.begin(115200);
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
  WaitForTempStab();
  GyroInit();
  AccInit();
  MagInit();
  imu.InitialQuat();

  CalcGravityOffSet();


  GPSStart();

  CheckTXPositions();
  imuTimer = micros();
  _400HzTimer = micros();
  generalPurposeTimer = millis();
  watchDogStartCount = true;

  accCircle.val = POSITION_ERROR_LIMIT;

  digitalWrite(RED,LOW);
  digitalWrite(YELLOW,LOW);
  digitalWrite(GREEN,LOW);
  digitalWrite(13,LOW);

}



void loop(){
  //pingFlag = newPing;
  _400HzTask();
  if ( micros() - imuTimer >= 16666){  
    imuDT = (micros() - imuTimer) * 0.000001;
    imuTimer = micros();
    estimateAttitude();
    estimatePositionVelocity();
    getAngles();


    FlightSM();

    _400HzTask();

    if (flightMode > 0){
      PitchAngle.calculate();
      RollAngle.calculate();
      if (calcYaw == true){
        YawAngle.calculate();
      }
    }
    _400HzTask();



    tuningTrasnmitOK = true;

    _400HzTask();
    gps.get_position(&lattitude.val,&longitude.val,&gpsFixAge);
    if (gpsFixAge > 500){
      GPSDenial = true;
      gpsFailSafe = true;
    }
  }


  _400HzTask();
  if (handShake == true){
    Radio();
    if (tuningTrasnmitOK == true){
      TuningTransmitter();
      if (pingFlag == 1){
        pingFlag = 0;
      }
      tuningTrasnmitOK = false;
    }
  }


  _400HzTask();

  if (gpsUpdate == true){
    gpsUpdate = false;
    gps.get_position(&lattitude.val,&longitude.val,&gpsFixAge);

    DistBearing(&homeBase.lat.val,&homeBase.lon.val,&lattitude.val,&longitude.val,&rawX.val,&rawY.val,&distToCraft,&headingToCraft);
    numSats.val = gps.satellites();
    hDop.val = gps.hdop();
    if (numSats.val <= 6  || hDop.val >= 215 ){
      //digitalWrite(RED,HIGH);
      GPSDenial = true;
      gpsFailSafe = true;
    }

    positionError.val = sqrt( sq(rawX.val - drPosX.val)  + sq(rawY.val - drPosY.val)  );

    if (positionError.val > POSITION_ERROR_LIMIT && drFlag == false){
      drFlag = true;
      startingX = drPosX.val;
      startingY = drPosY.val;
      imu.velX.val = drVelX.val;
      imu.velY.val = drVelY.val;

      accCircle.val = POSITION_ERROR_LIMIT;
      drTimer = micros();
    }
    if (drFlag == false){
      imu.GPSKalUpdate();
    }
    else{
      accCircle.val *= ACC_RATE;
      if (positionError.val < POSITION_ERROR_LIMIT){
        drFlag = false;
      }
      else{
        if (positionError.val < accCircle.val){
          drFlag = false;
          jumpDistX = rawX.val - startingX;
          jumpDistY = rawY.val - startingY;
          xTarget.val += jumpDistX;
          yTarget.val += jumpDistY;
          drPosX.val = rawX.val;
          drPosY.val = rawY.val;
          homeBaseXOffset += jumpDistX;
          homeBaseYOffset += jumpDistY;
          imu.currentEstIndex = 0;
          imu.lagIndex = 1;
          imu.XEst.val = rawX.val;
          imu.YEst.val = rawY.val;
          imu.velX = drVelX;
          imu.velY = drVelY;
          for (uint8_t resetIndex = 0; resetIndex< LAG_SIZE; resetIndex++){
            imu.XEstHist[resetIndex] = rawX.val;
            imu.YEstHist[resetIndex] = rawY.val;
          }
          drTimer = micros();
        }
      }

    }
  }

  _400HzTask();
  PollPressure();
  _400HzTask();
  if (newBaro == true){
    newBaro = false;
    GetAltitude(&pressure.val,&pressureInitial,&baroZ.val);
    rawZ.val = baroZ.val;
    imu.BaroKalUpdate();
  }
  _400HzTask();
  if (newPing == true){
    pingFlag = 1;
    newPing = false;
    ultraSonicRange.val = cos(ToRad(imu.pitch.val)) * cos(ToRad(imu.roll.val)) * pingDistMeters;

  }


  /*if (millis() - generalPurposeTimer >= 50){
   generalPurposeTimer = millis();
   //Port0<<generalPurposeTimer<<","<<temperature<<","<<gyroX.val<<","<<gyroY.val
   //<<","<<gyroZ.val<<","<<magX.val<<","<<magY.val<<","<<magZ.val
   //<<","<<accX.val<<","<<accY.val<<","<<accZ.val
   //<<","<<imu.pitch.val<<","<<imu.roll.val<<","<<imu.yaw.val<<"\r\n";
   //Port0<<imu.ZEst.val<<","<<imu.velZ.val<<","<<baroZ.val<<","<<ultraSonicRange.val<<","<<pingFlag<<"\r\n";
   
   //Port0<<millis()<<","<<radianGyroX<<","<<radianGyroY<<","<<radianGyroZ<<"\r\n";
   //Port0<<"^"<<imu.pitch.val<<","<<imu.roll.val<<","<<imu.yaw.val<<"\r\n";
   //Port0<<generalPurposeTimer<<","<<scaledAccX<<","<<scaledAccY<<","<<scaledAccZ<<"\r\n";
   //Port0<<a<<","<<b<<","<<c<<","<<D<<","<<E<<","<<F<<","<<G<<"\r\n";
   ///Port0<<generalPurposeTimer<<","<<imu.pitch.val<<","<<imu.roll.val<<","<<imu.yaw.val<<","<<scaledAccX<<","<<scaledAccY<<","<<scaledAccZ<<","<<imu.feedBack<<","<<imu.inertialZ<<"\r\n";
   //Port0<<imu.inertialZ<<"\r\n";
   //Port0<<scaledAccX<<","<<scaledAccY<<","<<scaledAccZ<<"\r\n";
   //Port0<<RCValue[THRO]<<","<<RCValue[AILE]<<","<<RCValue[ELEV]
   //<<","<<RCValue[RUDD]<<","<<RCValue[GEAR]<<","<<RCValue[AUX1]
   //<<","<<RCValue[AUX2]<<","<<RCValue[AUX3]<<"\r\n";
   //Port0<<rateSetPointY.val<<","<<rateSetPointX.val<<","<<rateSetPointZ.val<<"\r\n";
   }*/


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
    Motor7WriteMicros(1000);
    Motor8WriteMicros(1000);
    if (failSafe == true){
      digitalWrite(RED,HIGH);
    }
    while(1){

      digitalWrite(YELLOW,HIGH);
      delay(500);
      digitalWrite(YELLOW,LOW);
      delay(500);
    }
  }
  _400HzTask();
  watchDogFailSafeCounter = 0;

}


void getAngles(){

  imu.GetPitch();

  _400HzTask();

  imu.GetRoll();

  _400HzTask();

  imu.GetYaw();

  _400HzTask();

}

void estimateAttitude(){
  _400HzTask();

  GetMag();

  _400HzTask();

  accToFilterX = -1.0 * filtAccX.val;//if the value from the smoothing filter is sent it will not work when the algorithm normalizes the vector
  accToFilterY = -1.0 * filtAccY.val;
  accToFilterZ = -1.0 * filtAccZ.val;
  imu.AHRSStart();

  _400HzTask();

  imu.AHRSEnd();

}

void estimatePositionVelocity(){
  if (drFlag == false){
    if (imuTimer - drTimer > DR_PERIOD ){
      drTimer = imuTimer;
      drVelX.val = imu.velX.val;
      drPosX.val = imu.XEst.val;

      drVelY.val = imu.velY.val;
      drPosY.val = imu.YEst.val;      

    }
    else{
      halfDTSq = 0.5 * imuDT * imuDT;
      drVelX.val += imu.inertialX.val * imuDT + imu.accelBiasX * imuDT;
      drPosX.val += drVelX.val * imuDT + imu.inertialX.val * halfDTSq + imu.accelBiasX * halfDTSq;

      drVelY.val += imu.inertialY.val * imuDT + imu.accelBiasY * imuDT;
      drPosY.val += drVelY.val * imuDT + imu.inertialY.val * halfDTSq + imu.accelBiasY * halfDTSq;
    }      
  }
  else{
    halfDTSq = 0.5 * imuDT * imuDT;
    drVelX.val += imu.inertialX.val * imuDT + imu.accelBiasX * imuDT;
    drPosX.val += drVelX.val * imuDT + imu.inertialX.val * halfDTSq + imu.accelBiasX * halfDTSq;

    drVelY.val += imu.inertialY.val * imuDT + imu.accelBiasY * imuDT;
    drPosY.val += drVelY.val * imuDT + imu.inertialY.val * halfDTSq + imu.accelBiasY * halfDTSq;

    imu.XEst.val = drPosX.val;
    imu.velX.val = drVelX.val;

    imu.YEst.val = drPosY.val;
    imu.velY.val = drVelY.val;
    if (imuTimer - drTimer > DR_FS_PERIOD ){
      GPSDenial = true;
      gpsFailSafe = true;
    }
  }

  _400HzTask();

  imu.GetInertial();

  _400HzTask();

  imu.pUpateX();

  _400HzTask();

  imu.pUpateY();

  _400HzTask();

  imu.pUpateZ();

  _400HzTask();

  imu.UpdateLagIndex();

  _400HzTask();
}

void DistBearing(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2,float *distX,float *distY,float *distDirect,float *bearing){
  //using euqirectangular projection since the distances are << than the RADIUS of the earth
  float deltaLat = ToRad( (*lat2 - * lat1) * 0.000001 );
  //the below line is as such to get the signs between the accelerometer and GPS position to the same sign convention
  //this will work for the north west heimsphere

  float deltaLon = (ToRad( (*lon2 - * lon1) * 0.000001 ) ) * cos( ToRad((*lat2 * 0.000001)) );
  *distX = deltaLat * RADIUS_EARTH;
  *distY = deltaLon * RADIUS_EARTH;
  *distDirect = sqrt(*distX * *distX + *distY * *distY);
  *bearing = FastAtan2(*distY,*distX);
}


void _400HzTask(){
  if (micros() - _400HzTimer >= 2500){
    rateDT = (micros() - _400HzTimer) * 0.000001;
    _400HzTimer = micros();
    GetGyro();
    PitchRate.calculate();
    RollRate.calculate();
    YawRate.calculate();
    MotorHandler();
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
    }
    break;
  case WP:
    digitalWrite(13,LOW);
    digitalWrite(RED,LOW);
    digitalWrite(YELLOW,HIGH);
    digitalWrite(GREEN,LOW);
    if (enterState == true){
      enterState = false;
    }
    break;
  case RTB:
    digitalWrite(13,LOW);
    digitalWrite(RED,LOW);
    digitalWrite(YELLOW,LOW);
    digitalWrite(GREEN,HIGH);
    if (enterState == true){
      enterState = false;
      enterState = false;
      xTarget.val = imu.XEst.val;
      yTarget.val = imu.YEst.val;
      zTarget.val = imu.ZEst.val + 1; 
      if (zTarget.val > CEILING){
        zTarget.val = CEILING;
      }
      RTBState = CLIMB;
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
    zTarget.val = imu.ZEst.val; 
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
    if (imu.ZEst.val >= zTarget.val){

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
    if (fabs(xTarget.val - imu.XEst.val) < 1.5){
      velSetPointX.val = 0;
    }
    else{

    }
    if (fabs(yTarget.val - imu.YEst.val) < 1.5){
      velSetPointY.val = 0;
    }
    else{

    }
    LoiterXPosition.calculate();
    LoiterYPosition.calculate();
    if (velSetPointX.val > 0.5){
      velSetPointX.val = 0.5;
    }
    if (velSetPointX.val < -0.5){
      velSetPointX.val = -0.5;
    }
    if (velSetPointY.val > 0.5){
      velSetPointY.val = 0.5;
    }
    if (velSetPointY.val < -0.5){
      velSetPointY.val = -0.5;
    }
    LoiterXVelocity.calculate();
    tiltAngleX.val *= -1.0;
    LoiterYVelocity.calculate();
    RotatePitchRoll(&imu.yaw.val,&zero,&tiltAngleX.val,&tiltAngleY.val,&pitchSetPoint.val,&rollSetPoint.val);
    AltHoldPosition.calculate();
    AltHoldVelocity.calculate();
    if (fabs(imu.XEst.val - homeBaseXOffset) < 1 && fabs(imu.YEst.val - homeBaseYOffset) < 1){
      velSetPointZ.val = LAND_VEL;
      RTBState = DESCEND;
    }
    if (gpsFailSafe == true){
      velSetPointZ.val = LAND_VEL;
      RTBState = DESCEND;
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
    velSetPointZ.val = LAND_VEL;
    AltHoldVelocity.calculate();
    motorState = LANDING;
    break;
  }  
}

void LoiterCalculations(){
  //make special PID loops for the loiter positions
  if (fabs(xTarget.val - imu.XEst.val) < 1.5){
    velSetPointX.val = 0;
  }
  else{
    LoiterXPosition.calculate();
  }
  if (fabs(yTarget.val - imu.YEst.val) < 1.5){
    velSetPointY.val = 0;
  }
  else{
    LoiterYPosition.calculate();
  }

  LoiterXVelocity.calculate();
  tiltAngleX.val *= -1.0;
  LoiterYVelocity.calculate();
}


















