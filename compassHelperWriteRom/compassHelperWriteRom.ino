#include <EEPROM.h>

//ROM defines
#define ALL_DEF 7
#define GAIN_DEF 6
#define COMP_DEF 5
#define ACC_DEF 4 
#define ALL_CAL 3
#define GAIN_CAL 2
#define COMP_CAL 1
#define ACC_CAL 0

//magnetometer calibration values
float MAG_OFFSET_X;
float MAG_OFFSET_Y;
float MAG_OFFSET_Z;
float W_INV_00;
float W_INV_01;
float W_INV_02;
float W_INV_10;
float W_INV_11;
float W_INV_12;
float W_INV_20;
float W_INV_21;
float W_INV_22;

typedef union{
  float num;
  uint8_t buffer[4];
}
Float_Union_t;

Float_Union_t outFloat;

uint8_t calibrationFlags;

uint8_t outFloatIndex;

void setup(){
  SetDefaultCompassCal();
}
void loop(){
}


void SetDefaultCompassCal(){

  MAG_OFFSET_X = 7.218511;
  MAG_OFFSET_Y = 48.940489;
  MAG_OFFSET_Z = -7.922210;
  W_INV_00 = 0.994517;
  W_INV_01 = 0.051270;
  W_INV_02 = -0.030916;
  W_INV_10 = 0.051270;
  W_INV_11 = 0.922016;
  W_INV_12 = 0.044415;
  W_INV_20 = -0.030916;
  W_INV_21 = 0.044415;
  W_INV_22 = 1.124646;
  for (uint16_t i = 0x19; i <= 0x48; i++){
    switch (i){
    case 0x19:
      outFloat.num = MAG_OFFSET_X;
      outFloatIndex = 0;
      break;
    case 0x1D:
      outFloat.num = MAG_OFFSET_Y;
      outFloatIndex = 0;
      break;
    case 0x21:
      outFloat.num = MAG_OFFSET_Z;
      outFloatIndex = 0;
      break;  
    case 0x25:
      outFloat.num = W_INV_00;
      outFloatIndex = 0;
      break;  
    case 0x29:
      outFloat.num = W_INV_01;
      outFloatIndex = 0;
      break;  
    case 0x2D:
      outFloat.num = W_INV_02;
      outFloatIndex = 0;
      break;  
    case 0x31:
      outFloat.num = W_INV_10;
      outFloatIndex = 0;
      break;  
    case 0x35:
      outFloat.num = W_INV_11;
      outFloatIndex = 0;
      break;  
    case 0x39:
      outFloat.num = W_INV_12;
      outFloatIndex = 0;
      break;  
    case 0x3D:
      outFloat.num = W_INV_20;
      outFloatIndex = 0;
      break;  
    case 0x41:
      outFloat.num = W_INV_21;
      outFloatIndex = 0;
      break;  
    case 0x45:
      outFloat.num = W_INV_22;
      outFloatIndex = 0;
      break;  
    default:
      break;
    }
    EEPROM.write(i,outFloat.buffer[outFloatIndex]);
    outFloatIndex++;
  }

  calibrationFlags = EEPROM.read(0x00);
  calibrationFlags |= (1<<COMP_CAL);
  calibrationFlags &= ~(1<<COMP_DEF);
  calibrationFlags &= ~(1<<ALL_DEF);
  EEPROM.write(0x00,calibrationFlags);



}
