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

  MAG_OFFSET_X = 2.469721;
  MAG_OFFSET_Y = 93.204543;
  MAG_OFFSET_Z = -3.020533;
  W_INV_00 = 0.995896;
  W_INV_01 = -0.002183;
  W_INV_02 = 0.003579;
  W_INV_10 = -0.002183;
  W_INV_11 = 1.013013;
  W_INV_12 = -0.003179;
  W_INV_20 = 0.003579;
  W_INV_21 = -0.003179;
  W_INV_22 = 1.056729;
  for (uint16_t i = 25; i <= 72; i++){
    switch (i){
    case 25:
      outFloat.num = MAG_OFFSET_X;
      outFloatIndex = 0;
      break;
    case 29:
      outFloat.num = MAG_OFFSET_Y;
      outFloatIndex = 0;
      break;
    case 33:
      outFloat.num = MAG_OFFSET_Z;
      outFloatIndex = 0;
      break;  
    case 37:
      outFloat.num = W_INV_00;
      outFloatIndex = 0;
      break;  
    case 41:
      outFloat.num = W_INV_01;
      outFloatIndex = 0;
      break;  
    case 45:
      outFloat.num = W_INV_02;
      outFloatIndex = 0;
      break;  
    case 49:
      outFloat.num = W_INV_10;
      outFloatIndex = 0;
      break;  
    case 53:
      outFloat.num = W_INV_11;
      outFloatIndex = 0;
      break;  
    case 57:
      outFloat.num = W_INV_12;
      outFloatIndex = 0;
      break;  
    case 61:
      outFloat.num = W_INV_20;
      outFloatIndex = 0;
      break;  
    case 65:
      outFloat.num = W_INV_21;
      outFloatIndex = 0;
      break;  
    case 69:
      outFloat.num = W_INV_22;
      outFloatIndex = 0;
      break;  
    default:
      break;
    }
    EEPROM.write(i,outFloat.buffer[outFloatIndex]);
    outFloatIndex++;
  }




}
