#include <EEPROM.h>
#include <Streaming.h>
//ROM defines
#define ALL_DEF 7
#define GAIN_DEF 6
#define COMP_DEF 5
#define ACC_DEF 4 
#define ALL_CAL 3
#define GAIN_CAL 2
#define COMP_CAL 1
#define ACC_CAL 0

static char hex[17]="0123456789ABCDEF";

//magnetometer calibration values
float accXScalePos, accYScalePos, accZScalePos, accXScaleNeg, accYScaleNeg, accZScaleNeg;

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

  accXScalePos = 0.0392;
  accYScalePos = 0.03726235741444866920152091254753;
  accZScalePos = 0.03798449612403100775193798449612;
  accXScaleNeg = 0.03873517786561264822134387351779;
  accYScaleNeg = 0.040329218106995884773662551440333;
  accZScaleNeg = 0.0392;

  for (uint16_t i = 1; i <= 24; i++){
    switch (i){
    case 1:
      outFloat.num = accXScalePos;
      outFloatIndex = 0;
      break;
    case 5:
      outFloat.num = accYScalePos;
      outFloatIndex = 0;
      break;
    case 9:
      outFloat.num = accZScalePos;
      outFloatIndex = 0;
      break;  
    case 13:
      outFloat.num = accXScaleNeg;
      outFloatIndex = 0;
      break;  
    case 17:
      outFloat.num = accYScaleNeg;
      outFloatIndex = 0;
      break;  
    case 21:
      outFloat.num = accZScaleNeg;
      outFloatIndex = 0;
      break;  
 
    default:
      break;
    }
    EEPROM.write(i,outFloat.buffer[outFloatIndex]);
    outFloatIndex++;
  }




}

static void ShowHex(byte convertByte){
 Serial << hex[(convertByte >>4) & 0x0F];
 Serial << hex[convertByte & 0x0F]<<"\r\n";
}
