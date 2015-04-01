//
//#include <MemoryFree.h>
#include <EEPROM.h>
#include <SPI.h>
#include "I2CL.h"
#include "openIMUL.h"
#include "PID.h"
#include <Streaming.h>
#include "AUXMATH.h"
#include "UBLOXL.h"

//#define ROT_45


#define V1
//#define V2

#ifdef V2
#ifdef V1
#undef V1
#endif
#endif


#define SACC_MAX 0.5
#define HACC_MAX 2
#define MIN_SATS 10

#define CAL_FLAGS 0
#define HS_FLAG 1

#define PKT_LOCAL_ORD_L 2
#define PKT_LOCAL_ORD_M 3

#define PKT_LOCAL_UN_L 4
#define PKT_LOCAL_UN_M 5

#define PR_FLAG 6

#define ESC_CAL_FLAG 7

#define ACC_CALIB_START 8
#define ACC_CALIB_END 31

#define MAG_CALIB_START 32
#define MAG_CALIB_END 79

#define PITCH_OFFSET_START 80
#define PITCH_OFFSET_END 83

#define ROLL_OFFSET_START 84
#define ROLL_OFFSET_END 87

#define GAINS_START 88
#define GAINS_END 327

#define DEC_START 328
#define DEC_END 331

#define RC_DATA_START 332
#define RC_DATA_END 427

#define MAX_INDEX 333
#define MIN_INDEX 335
#define MID_INDEX 337
#define CHAN_INDEX 338
#define SCALE_INDEX 342
#define REV_INDEX 343

#define ACC_S_X_INDEX 11
#define ACC_S_Y_INDEX 15
#define ACC_S_Z_INDEX 19
#define ACC_O_X_INDEX 23
#define ACC_O_Y_INDEX 27
#define ACC_O_Z_INDEX 31


#define MAG_OFF_X_INDEX 35
#define MAG_OFF_Y_INDEX 39
#define MAG_OFF_Z_INDEX 43
#define W_00_INDEX 47
#define W_01_INDEX 51
#define W_02_INDEX 55
#define W_10_INDEX 59
#define W_11_INDEX 63
#define W_12_INDEX 67
#define W_20_INDEX 71
#define W_21_INDEX 75
#define W_22_INDEX 79


#define VER_FLAG_1 428
#define VER_FLAG_2 429

#define VER_NUM_1 0x01
#define VER_NUM_2 0x01

#define PWM_LIM_HIGH_START 430
#define PWM_LIM_HIGH_END 431
#define PWM_LIM_LOW_START 432
#define PWM_LIM_LOW_END 433
#define PWM_FLAG 434
#define PROP_IDLE 435
#define PROP_IDLE_FLAG 436
#define HOVER_THRO 437
#define HOVER_THRO_FLAG 438

#define TX_FS_FLAG 439
#define TX_FS 440

#define FC 5
#define RC_CONST 1/(2.0 * 3.14 * FC)

#define FC_BARO 3.0
#define RC_CONST_BARO 1/(2.0 * 3.14 * FC_BARO)



#define RADIUS_EARTH 6372795

#define CEILING 6
#define FLOOR 2
#define TAKE_OFF_ALT 3

#define LAND_VEL -0.28

//ROM defines
enum CalibrationFlags {
  RC_FLAG,
  ACC_FLAG,
  MAG_FLAG,
  GAINS_FLAG
};

enum RTBStates {
  CLIMB,
  TRAVEL,
  DESCEND
};




//LED defines GREEN, YELLOW, BLUE, RED
#define GREEN 42
#define YELLOW 40
#define BLUE 13
#define RED 38

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
#define L3G_WHO_AM_I 0x0F

//mag defines ST HMC5983DLHC - will work with the HMC5883L
#define MAG_ADDRESS 0x1E
#define HMC5983_CRA_REG (uint8_t)0x00
#define HMC5983_CRB_REG 0x01
#define HMC5983_MR_REG 0x02
#define HMC5983_OUT_X_H 0x03

#define HMC5983_ID_A 0x0A
#define HMC5983_ID_B 0x0B
#define HMC5983_ID_C 0x0C


#define Port0 Serial
#define RCSigPort Serial1
#define Port2 Serial2
#define gpsPort Serial3


//V1 defines
#ifdef V1
//acc defines - Analog Devices ADXL345
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAX0 0x32

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

#define Motor1WriteMicros(x) OCR3B = x * 2
#define Motor2WriteMicros(x) OCR3C = x * 2
#define Motor3WriteMicros(x) OCR3A = x * 2
#define Motor4WriteMicros(x) OCR4A = x * 2
#define Motor5WriteMicros(x) OCR4B = x * 2
#define Motor6WriteMicros(x) OCR4C = x * 2
#define Motor7WriteMicros(x) OCR1A = x * 2
#define Motor8WriteMicros(x) OCR1B = x * 2
#endif//#ifdef V1
//end V1 defines

//V2 defines
#ifdef V2
//acc defines
#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define CTRL_REG6_A 0x25
#define OUT_X_L_A 0x28

//baro defines
#define MS5611_RESET 0x1E
#define MS5611_PROM_Setup 0xA0
#define MS5611_PROM_C1 0xA2
#define MS5611_PROM_C2 0xA4
#define MS5611_PROM_C3 0xA6
#define MS5611_PROM_C4 0xA8
#define MS5611_PROM_C5 0xAA
#define MS5611_PROM_C6 0xAC
#define MS5611_PROM_CRC 0xAE
#define CONVERT_D1_OSR4096 0x48   // Maximun resolution
#define CONVERT_D2_OSR4096 0x58   // Maximun resolution

#define ADC_READ 0x00

#define BARO_CONV_TIME 50


#define Motor1WriteMicros(x) OCR3A = x * 2//motor 1 is attached to pin2
#define Motor2WriteMicros(x) OCR3B = x * 2//motor 2 is attached to pin3
#define Motor3WriteMicros(x) OCR3C = x * 2//motor 3 is attached to pin5
#define Motor4WriteMicros(x) OCR4A = x * 2//motor 4 is attached to pin6
#define Motor5WriteMicros(x) OCR4B = x * 2//motor 1 is attached to pin7
#define Motor6WriteMicros(x) OCR4C = x * 2//motor 2 is attached to pin8
#define Motor7WriteMicros(x) OCR1A = x * 2//motor 3 is attached to pin11
#define Motor8WriteMicros(x) OCR1B = x * 2//motor 4 is attached to pin12
#endif//#ifdef V2
//end V2 defines



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

#define GyroSSOutput() DDRL |= 1<<0
#define GyroSSHigh() PORTL |= 1<<0
#define GyroSSLow() PORTL &= ~(1<<0)

#define AccSSOutput() DDRL |= 1<<1
#define AccSSHigh() PORTL |= 1<<1
#define AccSSLow() PORTL &= ~(1<<1)

#define BaroSSOutput() DDRL |= 1<<2
#define BaroSSHigh() PORTL |= 1<<2
#define BaroSSLow() PORTL &= ~(1<<2)

#define MagSSOutput() DDRL |= 1<<3
#define MagSSHigh() PORTL |= 1<<3
#define MagSSLow() PORTL &= ~(1<<3)

#define FlashSSOutput() DDRL |= 1<<4
#define FlashSSHigh() PORTL |= 1<<4
#define FlashSSLow() PORTL &= ~(1<<4)

#define RC_SS_Output() DDRH |= 1<<7
#define RC_SSHigh() PORTH |= 1<<7
#define RC_SSLow() PORTH &= ~(1<<7)

//control defines
#define HH_ON 0
#define HH_OFF 1



#define MAX_Z_RATE 3.0f
#define MIN_Z_RATE -3.0f


//motor defines
#define FREQ 100
#define PRESCALE 8
#define PERIOD ((F_CPU/PRESCALE/FREQ) - 1)




//radio control defines
//RC defines
enum RC_Types {
  DSMX = 1, SBUS, RC
};


enum ISR_States {
  STAND, PPM
};

enum motorControlStates {
  HOLD,
  TO,
  FLIGHT,
  LANDING
};

enum RC_Chan {
  THRO, AILE, ELEV, RUDD, GEAR, AUX1, AUX2, AUX3
};

enum loiterStates {
  LOITERING,
  RCINPUT,
  LAND,
  WAIT
};

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
  THRO_CMD,
  PWM_HIGH,
  PWM_LOW

};

enum BYTES {
  F_MODE_,
  GPS_FIX,
  XY_LOIT_STATE,
  Z_LOIT_STATE,
  RTB_STATE,
  MOTOR_STATE,
  TELEM_FS,
  GPS_FS,
  SWITCH_POS,
  NUM_SATS,
  IDLE_PERCENT,
  HOVER_PERCENT,
  TX_LOSS_RTB,
  MAG_DET,
  TX_FS_STATUS
};


enum Floats {
  GYRO_X_DEG,
  GYRO_Y_DEG,
  GYRO_Z_DEG,
  ACC_X_FILT,
  ACC_Y_FILT,
  ACC_Z_FILT,
  ACC_X_SC,
  ACC_Y_SC,
  ACC_Z_SC,
  MAG_X_CALIB,
  MAG_Y_CALIB,
  MAG_Z_CALIB,
  DIST_TO_CRAFT,
  HEAD_TO_CRAFT,
  RAW_X,
  RAW_Y,
  RAW_Z,
  VEL_N,
  VEL_E,
  VEL_D,
  VEL_BARO,
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
  ACC_BIAS_X,
  ACC_BIAS_Y,
  ACC_BIAS_Z,
  INERTIAL_X,
  INERTIAL_Y,
  INERTIAL_Z,
  INERTIAL_X_BIASED,
  INERTIAL_Y_BIASED,
  INERTIAL_Z_BIASED,
  RAW_PITCH,
  RAW_ROLL,
  PITCH_OFF,
  ROLL_OFF,
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
  MOTOR_CMD_1,
  MOTOR_CMD_2,
  MOTOR_CMD_3,
  MOTOR_CMD_4,
  PRESSURE_,
  CTRL_BEARING,
  YAW_INITIAL,
  GPS_ALT,
  LAT_,
  LON_,
  HB_LAT,
  HB_LON,
  H_ACC,
  S_ACC,
  P_DOP

};




#define RADIO_BUF_SIZE 256
#define NUM_WAY_POINTS 0x14
#define Port0 Serial
#define RCSigPort Serial1
#define Port2 Serial2
#define gpsPort Serial3

float_u *floatPointerArray[145];

int16_u *int16PointerArray[12];

uint8_t *bytePointerArray[16];

int16_u gyroX, gyroY, gyroZ, accX, accY, accZ, magX, magY, magZ;


int baroCount;
float baroSum;

#ifdef V1
//v1 vars
//barometer variables
//int32_t pres;
short temperature;
uint32_t baroDelayTimer;
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
//int baroCount;
//float baroSum;
long pressureInitial;
#endif//#ifdef V1
//end v1 vars


//v2 vars
#ifdef V2
//barometer
uint16_u C1, C2, C3, C4, C5, C6, promSetup, promCRC;
uint32_u D_rcvd;
float D1, D2;
float pres, temperature, dT, TEMP, OFF, SENS, P;
uint8_t baroState;
uint32_t baroPollTimer, baroDelayTimer;
boolean newBaro;


#endif//#ifdef V2
//end barometer
//IMU related vars
int32_t gyroSumX, gyroSumY, gyroSumZ;
int16_t gyroOffsetX, gyroOffsetY, gyroOffsetZ, gyroPrevX, gyroPrevY, gyroPrevZ;
float radianGyroX, radianGyroY, radianGyroZ;
float_u degreeGyroX, degreeGyroY, degreeGyroZ;
float_u filtAccX, filtAccY, filtAccZ;
float_u calibMagX, calibMagY, calibMagZ; //needs to be a float so the vector can be normalized
float shiftedMagX, shiftedMagY, shiftedMagZ;
float shiftedAccX, shiftedAccY, shiftedAccZ;
float_u scaledAccX, scaledAccY, scaledAccZ;
float accToFilterX, accToFilterY, accToFilterZ;

UBLOX gps;
//TinyGPS gps;
volatile boolean GPSDetected;
float_u distToCraft;
float_u headingToCraft;

//GPS related vars
typedef struct {
  int32_u lat;
  int32_u lon;
  //int32_u alt;
}
WayPoint_t;

WayPoint_t homeBase;
WayPoint_t wayPoints[20];
WayPoint_t loiterWP;//rename

float_u homeLat, homeLon;
//RC related vars
uint8_t readState, inByte, byteCount, channelNumber;
volatile uint8_t rcType;
uint32_t frameTime;
boolean detected = false;
volatile boolean newRC = false;
boolean frameStart = true;
boolean frameValid = false;

uint8_t spekBuffer[14];

uint16_t bufferIndex = 0;
boolean newGSRC = false;



uint8_t currentPinState = 0;
uint8_t previousPinState = 0;
uint8_t changeMask = 0;
uint8_t lastPinState = 0;
uint32_t currentTime = 0;
uint32_t previousTime = 0;
uint32_t timeDifference = 0;
uint32_t changeTime[8];
uint8_t sBusData[25];

uint8_t ISRState = STAND;
uint8_t channelCount = 0;


typedef struct {
  int16_t max;
  int16_t min;
  int16_t mid;
  volatile int16_t rcvd;
  uint8_t chan;
  float scale;
  uint8_t reverse;
}
RC_t;

RC_t rcData[8];
int16_t RCValue[8];
int16_t GSRCValue[8];

int16_u throttleCommand;

//timers and DTs
uint32_t imuTimer, GPSTimer;
uint32_t generalPurposeTimer;
uint16_t groundFSCount;
float imuDT;
float GPSDT;
float lpfDT, beta, alphaBaro, betaBaro;
float_u alpha;

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
//float_u mul_altitude_velocity;

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
float_u velSetPointX, velSetPointY;
float_u zTarget, velSetPointZ;
float yawInput;
float zero = 0.0;
float_u pitchSetPointTX, rollSetPointTX;
float_u distToWayPoint, targetVelWayPoint;
float speed2D_MPS;
float_u tiltAngleX, tiltAngleY;
uint8_t HHState = 1;
float_u motorCommand1, motorCommand2, motorCommand3, motorCommand4;
float motorPWM1, motorPWM2, motorPWM3, motorPWM4;
boolean integrate = false;
boolean enterState = true;

boolean calcYaw;


//failsafe related vars
volatile boolean failSafe = false;
boolean toggle;
volatile boolean watchDogStartCount;
volatile uint32_t watchDogFailSafeCounter, RCFailSafeCounter, GPSFailSafeCounter;

float_u xTarget, yTarget;
uint8_t calibrationFlags;

//uint8_t outFloatIndex;

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



float scaledMagX, scaledMagY, scaledMagZ, magNorm, magToFiltX, magToFiltY, magToFiltZ;


//x and y setpoints are rotated into pitch and roll set points


uint32_t _400HzTimer;
float_u controlBearing;
uint8_t XYLoiterState, ZLoiterState;
int16_t rcDifference;
uint32_t waitTimer;
uint8_t RTBState;

float homeBaseXOffset = 0, homeBaseYOffset = 0;


//radio protocol vars
uint16_t localPacketNumberOrdered, localPacketNumberUn, remotePacketNumberOrdered, remotePacketNumberUn, packetTemp[2];
uint32_t radioTimer;
boolean handShake;
uint8_t handShakeState, rxSum, rxDoubleSum, txSum, txDoubleSum, radioByte,
        packetLength, numRXBytes, radioState, numRXbytes, typeNum, cmdNum,
        itemBuffer[255], itemIndex, temp, hsNumItems, lsNumItems, hsList[40], lsList[40], liveDataBuffer[200], hsRequestNumber, lsRequestNumber, hsListIndex, lsListIndex;
uint32_t hsMillis, lsMillis, hsTXTimer, lsTXTimer;
boolean offsetFlag, sendCalibrationData, hsTX, lsTX, tuningTrasnmitOK;


uint8_t gpsFailSafe = false, txFailSafe = false, telemFailSafe = false, battFailSafe = false;

boolean trimMode, setTrim, trimComplete, autoMaticReady;
uint8_t throttleCheckFlag;
boolean modeSelect = false;
uint8_t switchPositions, clearTXRTB;
uint8_t previousFlightMode, motorState;
float_u initialYaw;
uint32_t tuningItemIndex;
uint32_t radioLimitTimer;

Print* radioPrint;
Stream* radioStream;

boolean USBFlag = false, saveGainsFlag = false;

uint32_t ledTimer;

float_u baroZ;

float_u gpsAlt;

float_u floatLat, floatLon;
float_u velN, velE, velD;

float initialPressure, alti;
float_u pressure;
uint32_t pollTimer;

uint32_t romWriteDelayTimer;
uint32_t _400Time;
float_u gpsX, gpsY;

uint32_t baroTimer;
float baroDT, prevBaro;
float_u baroRate;
float_u baroVel, baroAlt;
int16_t tempX, tempY;

float rotGyroX, rotGyroY, rotAccX, rotAccY;

uint32_t loopTime;
int16_u pwmHigh, pwmLow;
uint8_t propIdlePercent, hoverPercent;
uint16_t propIdleCommand, hoverCommand;

boolean gsCTRL = false;
int16_t loitThro;
float_u landingThroAdjustment;
float throAdjAlpha;

float_u hAcc, sAcc, pDop;

uint8_t numSats;
uint8_t txLossRTB;
//constructors //fix the dts
openIMU imu(&radianGyroX, &radianGyroY, &radianGyroZ, &accToFilterX, &accToFilterY, &accToFilterZ, &filtAccX.val, &filtAccY.val, &filtAccZ.val,
            &magToFiltX, &magToFiltY, &magToFiltZ, &gpsX.val, &gpsY.val, &baroZ.val, &velN.val, &velE.val, &baroVel.val, &imuDT);
//openIMU imu(&radianGyroX,&radianGyroY,&radianGyroZ,&accToFilterX,&accToFilterY,&accToFilterZ,&filtAccX.val,&filtAccY.val,&filtAccZ.val,&magToFiltX,&magToFiltY,&magToFiltZ,&gpsX.val,&gpsY.val,&baroZ.val,&zero,&zero,&imuDT);

PID PitchRate(&rateSetPointY.val, &degreeGyroY.val, &adjustmentY.val, &integrate, &kp_pitch_rate.val, &ki_pitch_rate.val, &kd_pitch_rate.val, &fc_pitch_rate.val, &imuDT, 400, 400);
PID RollRate(&rateSetPointX.val, &degreeGyroX.val, &adjustmentX.val, &integrate, &kp_roll_rate.val, &ki_roll_rate.val, &kd_roll_rate.val, &fc_roll_rate.val, &imuDT, 400, 400);
PID YawRate(&rateSetPointZ.val, &degreeGyroZ.val, &adjustmentZ.val, &integrate, &kp_yaw_rate.val, &ki_yaw_rate.val, &kd_yaw_rate.val, &fc_yaw_rate.val, &imuDT, 400, 400);

PID PitchAngle(&pitchSetPoint.val, &imu.pitch.val, &rateSetPointY.val, &integrate, &kp_pitch_attitude.val, &ki_pitch_attitude.val, &kd_pitch_attitude.val, &fc_pitch_attitude.val, &imuDT, 800, 800);
PID RollAngle(&rollSetPoint.val, &imu.roll.val, &rateSetPointX.val, &integrate, &kp_roll_attitude.val, &ki_roll_attitude.val, &kd_roll_attitude.val, &fc_roll_attitude.val, &imuDT, 800, 800);
YAW YawAngle(&yawSetPoint.val, &imu.yaw.val, &rateSetPointZ.val, &integrate, &kp_yaw_attitude.val, &ki_yaw_attitude.val, &kd_yaw_attitude.val, &fc_yaw_attitude.val, &imuDT, 800, 800);

PID LoiterXPosition(&xTarget.val, &imu.XEst.val, &velSetPointX.val, &integrate, &kp_loiter_pos_x.val, &ki_loiter_pos_x.val, &kd_loiter_pos_x.val, &fc_loiter_pos_x.val, &imuDT, 1, 1);
PID LoiterXVelocity(&velSetPointX.val, &imu.velX.val, &tiltAngleX.val, &integrate, &kp_loiter_velocity_x.val, &ki_loiter_velocity_x.val, &kd_loiter_velocity_x.val, &fc_loiter_velocity_x.val, &imuDT, 30, 30);

PID LoiterYPosition(&yTarget.val, &imu.YEst.val, &velSetPointY.val, &integrate, &kp_loiter_pos_y.val, &ki_loiter_pos_y.val, &kd_loiter_pos_y.val, &fc_loiter_pos_y.val, &imuDT, 1, 1);
PID LoiterYVelocity(&velSetPointY.val, &imu.velY.val, &tiltAngleY.val, &integrate, &kp_loiter_velocity_y.val, &ki_loiter_velocity_y.val, &kd_loiter_velocity_y.val, &fc_loiter_velocity_y.val, &imuDT, 30, 30);

PID AltHoldPosition(&zTarget.val, &imu.ZEstUp.val, &velSetPointZ.val, &integrate, &kp_altitude_position.val, &ki_altitude_position.val, &kd_altitude_position.val, &fc_altitude_position.val, &imuDT, 1.5, 1.5);
//ALT AltHoldVelocity(&velSetPointZ.val,&imu.velZUp.val,&throttleAdjustment.val,&integrate,&kp_altitude_velocity.val,&ki_altitude_velocity.val,&kd_altitude_velocity.val,&fc_altitude_velocity.val,&imuDT,800,800,&mul_altitude_velocity.val);
PID AltHoldVelocity(&velSetPointZ.val, &imu.velZUp.val, &throttleAdjustment.val, &integrate, &kp_altitude_velocity.val, &ki_altitude_velocity.val, &kd_altitude_velocity.val, &fc_altitude_velocity.val, &imuDT, 250, 250);

PID WayPointPosition(&zero, &distToWayPoint.val, &targetVelWayPoint.val, &integrate, &kp_waypoint_position.val, &ki_waypoint_position.val, &kd_waypoint_position.val, &fc_waypoint_position.val, &GPSDT, 10, 10);
PID WayPointRate(&targetVelWayPoint.val, &speed2D_MPS, &pitchSetPoint.val, &integrate, &kp_waypoint_velocity.val, &ki_waypoint_velocity.val, &kd_waypoint_velocity.val, &fc_waypoint_velocity.val, &imuDT, 45, 45);
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

void setup() {
  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(GREEN, OUTPUT);
  //digitalWrite(RED,HIGH);
  pinMode(13, OUTPUT);
  RC_SS_Output();
  Port0.begin(115200);
  Port2.begin(115200);
  Port2.write(0x0D);
  delay(250);
  Port2.print("B");
  AssignPointerArray();


  //----------------------------
  //Port0<<"1\r\n";
  //ROMFlagsCheck();
  //DEBUG_DUMP();
  //while(1){}

  //----------------------------

  GyroSSOutput();
  AccSSOutput();
  BaroSSOutput();
  MagSSOutput();
  FlashSSOutput();
  GyroSSHigh();
  AccSSHigh();
  BaroSSHigh();
  MagSSHigh();
  FlashSSHigh();

  D22Output();
  //pinMode(22,INPUT);
  D23Output();
  D24Output();
  D25Output();
  D26Output();
  D27Output();
  D28Output();
  D29Output();
  imuDT = 0.01;
  lpfDT = 0.0025;
  baroDT = 0.05;

  DetectRC();
  _200HzISRConfig();

  I2c.begin();
  I2c.setSpeed(1);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  ROMFlagsCheck();
  CheckESCFlag();

  CalibrateESC();
  MotorInit();
  delay(3500);//this allows the telemetry radios to connect before trying the handshake
  if (handShake == false) {
    radioStream = &Port2;
    radioPrint = &Port2;
    HandShake();

    if (handShake == false) {
      USBFlag = true;
      radioStream = &Port0;
      radioPrint = &Port0;
      HandShake();
    }
  }
  //Serial<<localPacketNumberOrdered<<","<<localPacketNumberUn<<"\r\n";

  Port0.begin(115200);
  if (calibrationMode == true) {
    //Port0<<"cal mode\r\n";
    BaroInit();
    AccInit();
    MagInit();
    CalibrateSensors();
    ROMFlagsCheck();
  }



  //ModeSelect();
  Arm();//move the rudder to the right to begin calibration
  GPSStart();
  BaroInit();
  GyroInit();
  AccInit();
  MagInit();
  GetInitialQuat();
  CheckTXPositions();


  imuTimer = micros();
  _400HzTimer = micros();
  generalPurposeTimer = millis();

  currentTime = micros();

  watchDogStartCount = true;
  digitalWrite(RED, 1);
  digitalWrite(YELLOW, 1);
  digitalWrite(GREEN, 1);
  digitalWrite(13, 1);
}

void loop() {//0

  _400HzTask();
  loopTime = micros();
  if (loopTime - imuTimer >= 10000) {//1-
    imuDT = (loopTime - imuTimer) * 0.000001;
    imuTimer = loopTime;
    GetGyro();
    _400HzTask();
    if (imu.magDetected == true) {//2
      GetMag();
    }//2
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

    if (GPSDetected == true) {//3
      gps.Monitor();
    }//3
    _400HzTask();
    if (gps.newData == true) {//4

      gps.newData = false;

      floatLat.val = (gps.data.vars.lat) * 0.0000001;
      floatLon.val = (gps.data.vars.lon) * 0.0000001;
      gpsAlt.val = gps.data.vars.height * 0.001;
      velN.val = gps.data.vars.velN * 0.01;
      velE.val = gps.data.vars.velE * 0.01;
      velD.val = gps.data.vars.velD * 0.01;
      gps.DistBearing(&homeBase.lat.val, &homeBase.lon.val, &gps.data.vars.lat, &gps.data.vars.lon, &gpsX.val, &gpsY.val, &distToCraft.val, &headingToCraft.val);
      hAcc.val = gps.data.vars.hAcc * 0.001;///raw pitch
      sAcc.val = gps.data.vars.sAcc * 0.001;//raw roll
      pDop.val = gps.data.vars.pDop * 0.01;//pitch offset
      numSats = gps.data.vars.numSV;
      if (gps.data.vars.gpsFix != 3 || numSats < MIN_SATS || hAcc.val > HACC_MAX || sAcc.val > SACC_MAX) {//5
        gpsFailSafe = true;
      }//5
      GPSFailSafeCounter = 0;

      imu.CorrectGPS();

    }//4

    if (GPSFailSafeCounter > 200) {//6
      gpsFailSafe = true;
    }//6
    PollPressure();
    _400HzTask();
    if (newBaro == true) {//7
      newBaro = false;
      GetAltitude(&pressure.val, &initialPressure, &baroAlt.val);
      baroDT = (millis() - baroTimer) * 0.001;
      baroTimer = millis();

      if (baroDT >= 0.1 || baroDT < 0) {//8
        baroDT = 0.1;
      }//8
      alphaBaro = baroDT / (baroDT + RC_CONST_BARO);
      betaBaro = 1 - alphaBaro;
      baroZ.val = baroZ.val * betaBaro + baroAlt.val * alphaBaro;
      baroRate.val = (baroZ.val - prevBaro) / baroDT;
      baroVel.val = baroVel.val * betaBaro + baroRate.val * alphaBaro;
      prevBaro = baroZ.val;
      imu.CorrectAlt();
    }//7
    _400HzTask();
    if (newRC == true) {//9
      newRC = false;
      ProcessChannels();
      GetSwitchPositions();
      RCFailSafeCounter = 0;
    }//9
    if (newGSRC == true) {//10
      groundFSCount = 0;
      newGSRC = false;
      telemFailSafe = false;
    }//10
    if (groundFSCount >= 200) {//11
      telemFailSafe = true;
    }//11
    _400HzTask();
    if (RCFailSafeCounter >= 200 || failSafe == true) {//12
      txFailSafe = true;
      if (txLossRTB == 0) {//13
        TIMSK5 = (0 << OCIE5A);
        digitalWrite(13, LOW);
        digitalWrite(RED, LOW);
        digitalWrite(YELLOW, LOW);
        digitalWrite(GREEN, LOW);
        Motor1WriteMicros(1000);//set the output compare value
        Motor2WriteMicros(1000);
        Motor3WriteMicros(1000);
        Motor4WriteMicros(1000);
        Motor5WriteMicros(1000);
        Motor6WriteMicros(1000);
        //Motor7WriteMicros(1000);
        //Motor8WriteMicros(1000);
        if (failSafe == true) {//14
          digitalWrite(RED, HIGH);
        }//14
        while (1) {

          digitalWrite(YELLOW, HIGH);
          if (RCFailSafeCounter >= 200 ) {//15
            digitalWrite(GREEN, LOW);
          }//15
          delay(500);
          digitalWrite(YELLOW, LOW);
          if (RCFailSafeCounter >= 200 ) {//16
            digitalWrite(GREEN, HIGH);
          }//16
          delay(500);
        }
      }//13

    }//12
    if (txFailSafe == true) {//17
      if (motorState >= FLIGHT) {//18
        if (flightMode != RTB) {//19
          enterState = true;
          flightMode = RTB;
        }//19
      }//18
    }//17


    _400HzTask();
    if (flightMode > 0) {//20
      PitchAngle.calculate();
      RollAngle.calculate();
      if (calcYaw == true) {//21
        YawAngle.calculate();
      }//21
    }//20
    _400HzTask();
    PitchRate.calculate();
    RollRate.calculate();
    YawRate.calculate();
    _400HzTask();
    MotorHandler();
    tuningTrasnmitOK = true;
    _400HzTask();
  }//1-

  _400HzTask();
  if (handShake == true) {//22
    Radio();
    if (tuningTrasnmitOK == true) {//23
      TuningTransmitter();

      tuningTrasnmitOK = false;

    }//23
  }//22
  _400HzTask();
  watchDogFailSafeCounter = 0;
}//0

void _400HzTask() {
  _400Time = micros();
  if ( _400Time - _400HzTimer  >= 2500 ) {
    lpfDT = (_400Time - _400HzTimer) * 0.000001;
    _400HzTimer = _400Time;
    GetAcc();
  }
}



void FlightSM() {

  switch (flightMode) {
    case RATE:
      if (enterState == true) {
        enterState = false;
        if (previousFlightMode != RATE && previousFlightMode != ATT) {
          throttleCheckFlag = true;
        }
        digitalWrite(13, HIGH);
        digitalWrite(RED, HIGH);
        digitalWrite(YELLOW, HIGH);
        digitalWrite(GREEN, HIGH);
      }
      TrimCheck();

      break;
    case ATT:
      if (enterState == true) {
        if (previousFlightMode != RATE && previousFlightMode != ATT) {
          throttleCheckFlag = true;

        }
        yawSetPoint = imu.yaw;
        enterState = false;
        digitalWrite(13, LOW);
        digitalWrite(RED, LOW);
        digitalWrite(YELLOW, LOW);
        digitalWrite(GREEN, LOW);
      }
      HeadingHold();
      TrimCheck();
      break;
    case L0:
      if (enterState == true) {
        enterState = false;
        InitLoiter();
        yawSetPoint = imu.yaw;
        digitalWrite(13, HIGH);
        digitalWrite(RED, HIGH);
        digitalWrite(YELLOW, LOW);
        digitalWrite(GREEN, LOW);
      }
      HeadingHold();
      controlBearing.val = imu.yaw.val;
      LoiterSM();
      break;
    case L1:
      if (enterState == true) {
        enterState = false;
        InitLoiter();
        yawSetPoint = imu.yaw;
        digitalWrite(13, HIGH);
        digitalWrite(RED, LOW);
        digitalWrite(YELLOW, HIGH);
        digitalWrite(GREEN, LOW);
        controlBearing.val = initialYaw.val;

      }
      HeadingHold();
      LoiterSM();
      break;
    case L2:
      if (enterState == true) {
        InitLoiter();
        yawSetPoint = imu.yaw;
        digitalWrite(13, HIGH);
        digitalWrite(RED, LOW);
        digitalWrite(YELLOW, LOW);
        digitalWrite(GREEN, HIGH);

        enterState = false;
      }
      HeadingHold();
      controlBearing.val = headingToCraft.val;
      LoiterSM();
      break;
    case FOLLOW:
      digitalWrite(13, LOW);
      digitalWrite(RED, HIGH);
      digitalWrite(YELLOW, LOW);
      digitalWrite(GREEN, LOW);
      if (enterState == true) {
        enterState = false;
        yawSetPoint = imu.yaw;
      }
      break;
    case WP:
      digitalWrite(13, LOW);
      digitalWrite(RED, LOW);
      digitalWrite(YELLOW, HIGH);
      digitalWrite(GREEN, LOW);
      if (enterState == true) {
        enterState = false;
        yawSetPoint = imu.yaw;
      }
      break;
    case RTB:
      digitalWrite(13, LOW);
      digitalWrite(RED, LOW);
      digitalWrite(YELLOW, LOW);
      digitalWrite(GREEN, HIGH);
      if (enterState == true) {
        enterState = false;
        xTarget.val = imu.XEst.val;
        yTarget.val = imu.YEst.val;
        zTarget.val = imu.ZEstUp.val + 1;
        if (zTarget.val > CEILING) {
          zTarget.val = CEILING;
        }
        if (zTarget.val < FLOOR) {
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

void TrimCheck() {
  uint8_t j;
  if (setTrim == true) {
    if (trimComplete == false) {
      trimComplete = true;
      imu.pitchOffset.val = imu.rawPitch.val;
      imu.rollOffset.val = imu.rawRoll.val;
      j = 0;
      for (uint16_t i = PITCH_OFFSET_START; i <= PITCH_OFFSET_END; i++) {
        EEPROM.write(i, imu.pitchOffset.buffer[j++]);
      }
      j = 0;
      for (uint16_t i = ROLL_OFFSET_START; i <= ROLL_OFFSET_END; i++) {
        EEPROM.write(i, imu.rollOffset.buffer[j++]);
      }
      EEPROM.write(PR_FLAG, 0xAA);
    }
  }
}

void InitLoiter() {

  if (previousFlightMode != L1 && previousFlightMode != L2 && previousFlightMode != L0) {
    if (motorState == LANDING) {
      motorState = FLIGHT;
    }
    throttleCheckFlag = true;
    throttleAdjustment.val = 0;
    xTarget.val = imu.XEst.val;
    yTarget.val = imu.YEst.val;
    zTarget.val = imu.ZEstUp.val;
    if (zTarget.val < FLOOR) {
      zTarget.val = FLOOR;
    }
    if (zTarget.val > CEILING) {
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

void RTBStateMachine() {
  if (motorState == HOLD || motorState == TO) {
    return;
  }
  switch (RTBState) {
    case CLIMB:
      LoiterCalculations();
      RotatePitchRoll(&imu.yaw.val, &zero, &tiltAngleX.val, &tiltAngleY.val, &pitchSetPoint.val, &rollSetPoint.val);
      AltHoldPosition.calculate();
      AltHoldVelocity.calculate();
      if (imu.ZEstUp.val >= (zTarget.val - 0.1) ) {

        RTBState = TRAVEL;
        xTarget.val = homeBaseXOffset;
        yTarget.val = homeBaseYOffset;
      }
      if (gpsFailSafe == true) {
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
      if (velSetPointX.val > 0.25) {
        velSetPointX.val = 0.25;
      }
      if (velSetPointX.val < -0.25) {
        velSetPointX.val = -0.25;
      }
      if (velSetPointY.val > 0.25) {
        velSetPointY.val = 0.25;
      }
      if (velSetPointY.val < -0.25) {
        velSetPointY.val = -0.25;
      }
      LoiterXVelocity.calculate();
      tiltAngleX.val *= -1.0;
      LoiterYVelocity.calculate();
      RotatePitchRoll(&imu.yaw.val, &zero, &tiltAngleX.val, &tiltAngleY.val, &pitchSetPoint.val, &rollSetPoint.val);
      AltHoldPosition.calculate();
      AltHoldVelocity.calculate();
      //if (fabs(imu.XEst.val - homeBaseXOffset) < 1 && fabs(imu.YEst.val - homeBaseYOffset) < 1){
      if (fabs(imu.XEst.val - xTarget.val) < 1.0 && fabs(imu.YEst.val - yTarget.val) < 1.0) {
        velSetPointZ.val = LAND_VEL;
        RTBState = DESCEND;
        motorState = LANDING;
      }
      if (gpsFailSafe == true) {
        velSetPointZ.val = LAND_VEL;
        RTBState = DESCEND;
        motorState = LANDING;
      }
      break;
    case DESCEND:
      if (gpsFailSafe == true) {
        pitchSetPoint.val = pitchSetPointTX.val;
        rollSetPoint.val = rollSetPointTX.val;

        if (txFailSafe == true) {
          pitchSetPoint.val = 0;
          rollSetPoint.val = 0;
        }
      }
      else {
        LoiterCalculations();
        RotatePitchRoll(&imu.yaw.val, &zero, &tiltAngleX.val, &tiltAngleY.val, &pitchSetPoint.val, &rollSetPoint.val);
      }
      //velSetPointZ.val = LAND_VEL;
      AltHoldVelocity.calculate();
      //motorState = LANDING;
      break;
  }
}

void LoiterCalculations() {
  LoiterXPosition.calculate();
  LoiterYPosition.calculate();
  LoiterXVelocity.calculate();
  tiltAngleX.val *= -1.0;
  LoiterYVelocity.calculate();
}






































































































