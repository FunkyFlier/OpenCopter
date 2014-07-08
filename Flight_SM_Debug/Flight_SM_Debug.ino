#include <Streaming.h>
#include <EEPROM.h>

/*
Spektrum DSMx serial should be logic level translated from 3.3v to 5v. 
 sBus data must run through an inverter
 For standard PWM RC signals here are the channel assignments:
 
 A8  - throttle
 A9  - aileron
 A10 - elevator
 A11 - rudder
 A12 - gear
 A13 - aux1
 A14 - aux2
 A15 - aux3
 
 */

typedef union{
  float val;
  uint8_t buffer[4];
}
float_u;

typedef union{
  int32_t val;
  uint8_t buffer[4];
}
int32_u;

typedef union{
  int16_t val;
  uint8_t buffer[2];
}
int16_u;

//LED defines
#define RED 38
#define YELLOW 40
#define GREEN 42

enum RC_Types {
  DSMX = 1, SBUS, RC};

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

enum RC_Chan {
  THRO, AILE, ELEV, RUDD, GEAR, AUX1, AUX2, AUX3};

volatile int16_t rawRCVal[8];


uint8_t currentPinState = 0;
uint8_t previousPinState = 0;
uint8_t changeMask = 0;
uint8_t lastPinState = 0;
uint16_t currentTime = 0;
uint16_t timeDifference = 0;
uint16_t changeTime[8];
uint8_t sBusData[25];



int16_t maxRCVal[8],minRCVal[8],centerRCVal[8];
int16_t RCValue[8];
int16_t RCOffset[8];
float RCScale[8];

uint32_t printTimer,generalPurposeTimer;
volatile boolean failSafe;
boolean gpsFailSafe,txFailSafe,telemFailSafe,battFailSafe;


void pause(){
  while(digitalRead(22)==0){
  }//wait for the toggle
  delay(500);
}

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

uint8_t itemBuffer[64];
uint32_t itemIndex;
boolean trimMode,setTrim,enterState,trimComplete;
boolean modeSelect = false;
uint8_t switchPositions,flightMode,clearTXRTB;
uint32_t timer;

void setup(){

  pinMode(RED,OUTPUT);
  pinMode(GREEN,OUTPUT);
  pinMode(YELLOW,OUTPUT);
  pinMode(13,OUTPUT);
  Serial.begin(115200);
  Serial<<"Start\r\n";
  DetectRC();
  _200HzISRConfig();
  printTimer = millis();
  generalPurposeTimer = millis();
  //GetMinMaxMid();
  //Serial<<"min max mid done\r\n";
  //pause();
  //Serial<<"write to rom\r\n";
  //WriteToROM();
  //pause();
  //Serial<<"read from rom\r\n";
  ReadFromROM();
  //Serial<<"Center vals: "<<centerRCVal[AILE]<<","<<centerRCVal[ELEV]<<","<<centerRCVal[RUDD]<<","<<centerRCVal[THRO]<<"\r\n";  
  //Serial<<"Min values: "<<minRCVal[AILE]<<","<<minRCVal[ELEV]<<","<<minRCVal[RUDD]<<","<<minRCVal[THRO]<<","<<minRCVal[GEAR]<<","<<minRCVal[AUX1]<<","<<minRCVal[AUX2]<<","<<minRCVal[AUX3]<<"\r\n";
  //Serial<<"Scale values: "<<RCScale[AILE]<<","<<RCScale[ELEV]<<","<<RCScale[RUDD]<<","<<RCScale[THRO]<<","<<RCScale[GEAR]<<","<<RCScale[AUX1]<<","<<RCScale[AUX2]<<","<<RCScale[AUX3]<<"\r\n";
  //pause();
  Serial<<"mode select\r\n";
  ModeSelect();
  Serial<<"Trim mode: "<<trimMode<<"\r\n";
  //pause();

  CheckTXPositions();

  digitalWrite(RED,LOW);
  digitalWrite(YELLOW,LOW);
  digitalWrite(GREEN,HIGH);
  digitalWrite(13,LOW);
  //delay(2000);

}
void CheckTXPositions(){
  boolean positionOK = false;
  while (positionOK == false){
    digitalWrite(RED,LOW);
    digitalWrite(YELLOW,LOW);
    digitalWrite(GREEN,LOW);
    digitalWrite(13,HIGH);
    if (newRC == true){
      newRC = false;
      ProcessChannels();
      Serial<<RCValue[THRO]<<","<<RCValue[AILE]<<","<<RCValue[ELEV]<<","<<RCValue[RUDD]<<","<<RCValue[GEAR]<<","<<RCValue[AUX1]<<","<<RCValue[AUX2]<<","<<RCValue[AUX3]<<"\r\n";

    } 
    positionOK = true;
    if (RCValue[THRO] > 1010){
      positionOK = false;
    }
    if (RCValue[GEAR] > 1010){
      positionOK = false;
    }
    if (RCValue[AUX1] > 1010){
      positionOK = false;
    }
    if (RCValue[AUX2] > 1010){
      positionOK = false;
    }
    if (RCValue[AUX3] > 1010){
      positionOK = false;
    }
  }
}

void ProcessChannels(){

  RCValue[THRO] = (rawRCVal[THRO] - minRCVal[THRO]) * RCScale[THRO] + 1000;
  RCValue[GEAR] = (rawRCVal[GEAR] - minRCVal[GEAR]) * RCScale[GEAR] + 1000;
  RCValue[AUX1] = (rawRCVal[AUX1] - minRCVal[AUX1]) * RCScale[AUX1] + 1000;
  RCValue[AUX2] = (rawRCVal[AUX2] - minRCVal[AUX2]) * RCScale[AUX2] + 1000;
  RCValue[AUX3] = (rawRCVal[AUX3] - minRCVal[AUX3]) * RCScale[AUX3] + 1000;

  RCValue[AILE] = (rawRCVal[AILE] - centerRCVal[AILE]) * RCScale[AILE] + 1500;
  RCValue[ELEV] = (rawRCVal[ELEV] - centerRCVal[ELEV]) * RCScale[ELEV] + 1500 ;
  RCValue[RUDD] = (rawRCVal[RUDD] - centerRCVal[RUDD]) * RCScale[RUDD] + 1500 ;


  if (txFailSafe == true){
    
    flightMode = RTB;
    switch(clearTXRTB){
      
    case 0:
      if (RCValue[GEAR] > 1850){
        clearTXRTB = 1;
      }
      return;
      break;
      
    case 1:
      if (RCValue[GEAR] < 1150){
        txFailSafe = false;
        break;
      }
      return;
      break;
      
    }

  }
  else{
    clearTXRTB = 0;
  }

  if (trimMode == false){


    switch (switchPositions){
    case 0:
      flightMode = L0;
      break;
    case 1:
      flightMode = L1;
      break;
    case 2:
      flightMode = L2;
      if (gpsFailSafe == true){
        flightMode = L1;

      }
      break;

    case 4:
      flightMode = FOLLOW;
      if (gpsFailSafe == true){
        flightMode = L0;
      }
      if (telemFailSafe == true){
        flightMode = L0;
      }
      break;
    case 5:
      flightMode = WP;
      if (gpsFailSafe == true){
        flightMode = L0;
      }
      if (telemFailSafe == true){
        flightMode = RTB;
      }
      break;
    case 6:
      flightMode = L0;
      break;

    case 8:
      //break;
    case 9:
      //break;
    case 10:
      flightMode = RTB;
      break;
    }

  }
  else{
    switch (switchPositions){
    case 0:
    case 1:
    case 2:
      flightMode = L0;
      break;

    case 4:
    case 5:
      flightMode = ATT;
      setTrim = false;
      trimComplete = false;
      break;
    case 6:
      flightMode = ATT;
      setTrim = true;
      break;

    case 8:
    case 9:
      flightMode = RATE;
      setTrim = false;
      trimComplete = false;
      break;
    case 10:
      setTrim = true;
      flightMode = RATE;
      break;
    }

  }

}

void FlightSM(){
  Serial<<"FS: "<<gpsFailSafe<<","<<txFailSafe<<","<<telemFailSafe<<"\r\n";
  switch(flightMode){
  case RATE:
    if (enterState == true){
      enterState = false;
    }
    if (setTrim == true){
      if (trimComplete == false){
        trimComplete = true;
        Serial<<"trims set\r\n";
        Serial<<"***************\r\n";
        delay(100);
      }
    }
    Serial<<"rate\r\n";
    break;
  case ATT:
    if (enterState == true){
      enterState = false;
    }
    if (setTrim == true){
      if (trimComplete == false){
        trimComplete = true;
        Serial<<"trims set\r\n";
        Serial<<"***************\r\n";
        delay(100);
      }
    }
    Serial<<"att\r\n";
    break;
  case L0:
    if (enterState == true){
      enterState = false;
    }
    Serial<<"Loiter 0\r\n";
    break;
  case L1:
    if (enterState == true){
      enterState = false;
    }
    Serial<<"Loiter 1\r\n";
    break;
  case L2:
    if (enterState == true){
      enterState = false;
    }
    Serial<<"Loiter 2\r\n";
    break;
  case FOLLOW:
    if (enterState == true){
      enterState = false;
    }
    Serial<<"Follow\r\n";
    break;
  case WP:
    if (enterState == true){
      enterState = false;
    }
    Serial<<"wp\r\n";
    break;
  case RTB:
    if (enterState == true){
      enterState = false;
    }
    Serial<<"RTB\r\n";
    break;

  }

}

void SetFailSafeFlags(){
  if (RCValue[AUX2] > 1850){
    return;
  }
  if (RCValue[AUX2] < 1100){
    gpsFailSafe = false;
    txFailSafe = false;
    telemFailSafe = false;
  }
  else{
    if (RCValue[AUX3] < 1150){
      gpsFailSafe = true;
    }
    if (RCValue[AUX3] > 1350 && RCValue[AUX3] < 1650){
      txFailSafe = true;
    }
    if (RCValue[AUX3] > 1750){
      telemFailSafe = true;
    }
  }

}

void loop(){
  if (newRC == true){
    newRC = false;
    ProcessChannels();
    GetSwitchPositions();
    SetFailSafeFlags();

  }    
  if (millis() - printTimer > 100){
    printTimer = millis();
    FlightSM();
    //Serial<<switchPositions<<","<<trimMode<<"\r\n";
    /*if (trimMode == false){
     
     
     switch (switchPositions){
     case 0:
     
     Serial<<"Flight state: 2\r\n";
     break;
     case 1:
     Serial<<"Flight state: 3\r\n";
     break;
     case 2:
     Serial<<"Flight state: 4\r\n";
     break;
     
     case 4:
     Serial<<"Flight state: 6\r\n";
     break;
     case 5:
     Serial<<"Flight state: 5\r\n";
     break;
     case 6:
     Serial<<"Flight state: 2\r\n";
     break;
     
     case 8:
     //break;
     case 9:
     //break;
     case 10:
     Serial<<"Flight state: 7\r\n";
     break;
     }
     
     }
     else{
     switch (switchPositions){
     case 0:
     Serial<<"Flight state: 2\r\n";
     break;
     case 1:
     Serial<<"Flight state: 2\r\n";
     break;
     case 2:
     Serial<<"Flight state: 2\r\n";
     break;
     
     case 4:
     case 5:
     Serial<<"Don't set trim\r\n";
     Serial<<"Flight state: 1\r\n";
     break;
     case 6:
     Serial<<"Set trim\r\n";
     Serial<<"Flight state: 1\r\n";
     break;
     
     case 8:
     case 9:
     Serial<<"Don't set trim\r\n";
     Serial<<"Flight state: 0\r\n";
     break;
     case 10:
     Serial<<"Set trim\r\n";
     Serial<<"Flight state: 0\r\n";
     break;
     }
     
     }*/

    //Serial<<rawRCVal[0]<<","<<rawRCVal[1]<<","<<rawRCVal[2]<<","<<rawRCVal[3]<<","<<rawRCVal[4]<<","<<rawRCVal[5]<<","<<rawRCVal[6]<<","<<rawRCVal[7]<<"\r\n";
    //Serial<<RCValue[0]<<","<<RCValue[1]<<","<<RCValue[2]<<","<<RCValue[3]<<","<<RCValue[4]<<","<<RCValue[5]<<","<<RCValue[6]<<","<<RCValue[7]<<"\r\n";
  }

}
void WriteToROM(){
  int16_u temp16;
  float_u tempFloat;

  //fill item buffer - in full prog this will be set through the radio protocol
  Serial<<"*Min values: "<<minRCVal[AILE]<<","<<minRCVal[ELEV]<<","<<minRCVal[RUDD]<<","<<minRCVal[THRO]<<","<<minRCVal[GEAR]<<","<<minRCVal[AUX1]<<","<<minRCVal[AUX2]<<","<<minRCVal[AUX3]<<"\r\n";

  itemIndex = 0;
  temp16.val = minRCVal[THRO];
  itemBuffer[itemIndex++] = temp16.buffer[0];
  itemBuffer[itemIndex++] = temp16.buffer[1];

  temp16.val = minRCVal[GEAR];
  itemBuffer[itemIndex++] = temp16.buffer[0];
  itemBuffer[itemIndex++] = temp16.buffer[1];

  temp16.val = minRCVal[AUX1];
  itemBuffer[itemIndex++] = temp16.buffer[0];
  itemBuffer[itemIndex++] = temp16.buffer[1];

  temp16.val = minRCVal[AUX2];
  itemBuffer[itemIndex++] = temp16.buffer[0];
  itemBuffer[itemIndex++] = temp16.buffer[1];

  temp16.val = minRCVal[AUX3];
  itemBuffer[itemIndex++] = temp16.buffer[0];
  itemBuffer[itemIndex++] = temp16.buffer[1];

  Serial<<"*Center vals: "<<centerRCVal[AILE]<<","<<centerRCVal[ELEV]<<","<<centerRCVal[RUDD]<<","<<centerRCVal[THRO]<<"\r\n";  

  temp16.val = centerRCVal[AILE];
  itemBuffer[itemIndex++] = temp16.buffer[0];
  itemBuffer[itemIndex++] = temp16.buffer[1];

  temp16.val = centerRCVal[ELEV];
  itemBuffer[itemIndex++] = temp16.buffer[0];
  itemBuffer[itemIndex++] = temp16.buffer[1];

  temp16.val = centerRCVal[RUDD];
  itemBuffer[itemIndex++] = temp16.buffer[0];
  itemBuffer[itemIndex++] = temp16.buffer[1];


  Serial<<"*Scale values: "<<RCScale[AILE]<<","<<RCScale[ELEV]<<","<<RCScale[RUDD]<<","<<RCScale[THRO]<<","<<RCScale[GEAR]<<","<<RCScale[AUX1]<<","<<RCScale[AUX2]<<","<<RCScale[AUX3]<<"\r\n";

  tempFloat.val = RCScale[THRO];
  itemBuffer[itemIndex++] = tempFloat.buffer[0];
  itemBuffer[itemIndex++] = tempFloat.buffer[1];
  itemBuffer[itemIndex++] = tempFloat.buffer[2];
  itemBuffer[itemIndex++] = tempFloat.buffer[3];

  tempFloat.val = RCScale[AILE];
  itemBuffer[itemIndex++] = tempFloat.buffer[0];
  itemBuffer[itemIndex++] = tempFloat.buffer[1];
  itemBuffer[itemIndex++] = tempFloat.buffer[2];
  itemBuffer[itemIndex++] = tempFloat.buffer[3];

  tempFloat.val = RCScale[ELEV];
  itemBuffer[itemIndex++] = tempFloat.buffer[0];
  itemBuffer[itemIndex++] = tempFloat.buffer[1];
  itemBuffer[itemIndex++] = tempFloat.buffer[2];
  itemBuffer[itemIndex++] = tempFloat.buffer[3];

  tempFloat.val = RCScale[RUDD];
  itemBuffer[itemIndex++] = tempFloat.buffer[0];
  itemBuffer[itemIndex++] = tempFloat.buffer[1];
  itemBuffer[itemIndex++] = tempFloat.buffer[2];
  itemBuffer[itemIndex++] = tempFloat.buffer[3];

  tempFloat.val = RCScale[GEAR];
  itemBuffer[itemIndex++] = tempFloat.buffer[0];
  itemBuffer[itemIndex++] = tempFloat.buffer[1];
  itemBuffer[itemIndex++] = tempFloat.buffer[2];
  itemBuffer[itemIndex++] = tempFloat.buffer[3];

  tempFloat.val = RCScale[AUX1];
  itemBuffer[itemIndex++] = tempFloat.buffer[0];
  itemBuffer[itemIndex++] = tempFloat.buffer[1];
  itemBuffer[itemIndex++] = tempFloat.buffer[2];
  itemBuffer[itemIndex++] = tempFloat.buffer[3];

  tempFloat.val = RCScale[AUX2];
  itemBuffer[itemIndex++] = tempFloat.buffer[0];
  itemBuffer[itemIndex++] = tempFloat.buffer[1];
  itemBuffer[itemIndex++] = tempFloat.buffer[2];
  itemBuffer[itemIndex++] = tempFloat.buffer[3];

  tempFloat.val = RCScale[AUX3];
  itemBuffer[itemIndex++] = tempFloat.buffer[0];
  itemBuffer[itemIndex++] = tempFloat.buffer[1];
  itemBuffer[itemIndex++] = tempFloat.buffer[2];
  itemBuffer[itemIndex++] = tempFloat.buffer[3];



  //write the buffer to rom for the min max etc
  itemIndex = 0;
  for(uint16_t i = 329; i <= 376; i++){
    EEPROM.write(i,itemBuffer[itemIndex++]);
  }


}
void ReadFromROM(){
  int16_u outShort;
  float_u outFloat;
  uint8_t j = 0;
  uint32_t   outFloatIndex ;
  for(uint16_t i = 329; i <= 344; i++){
    outShort.buffer[j] = EEPROM.read(i);
    j++;
    switch(i){
    case 330:
      minRCVal[THRO] = outShort.val;
      j = 0;
      break;
    case 332:
      minRCVal[GEAR] = outShort.val;
      j = 0;
      break;
    case 334:
      minRCVal[AUX1] = outShort.val;
      j = 0;
      break;
    case 336:
      minRCVal[AUX2] = outShort.val;
      j = 0;
      break;
    case 338:
      minRCVal[AUX3] = outShort.val;
      j = 0;
      break;
    case 340:
      centerRCVal[AILE] = outShort.val;
      j = 0;
      break;
    case 342:
      centerRCVal[ELEV] = outShort.val;
      j = 0;
      break;
    case 344:
      centerRCVal[RUDD] = outShort.val;
      j = 0;
      break;
    }
  }  

  outFloatIndex = 0;
  for(uint16_t i = 345; i <= 376; i++){
    outFloat.buffer[outFloatIndex] = EEPROM.read(i);
    outFloatIndex++;
    switch (i){
    case 348:
      RCScale[THRO] = outFloat.val;
      outFloatIndex = 0;
      break;
    case 352:
      RCScale[AILE] = outFloat.val;
      outFloatIndex = 0;
      break;
    case 356:
      RCScale[ELEV] = outFloat.val;
      outFloatIndex = 0;
      break;
    case 360:
      RCScale[RUDD] = outFloat.val;
      outFloatIndex = 0;
      break;
    case 364:
      RCScale[GEAR] = outFloat.val;
      outFloatIndex = 0;
      break;
    case 368:
      RCScale[AUX1] = outFloat.val;
      outFloatIndex = 0;
      break;
    case 372:
      RCScale[AUX2] = outFloat.val;
      outFloatIndex = 0;
      break;
    case 376:
      RCScale[AUX3] = outFloat.val;
      outFloatIndex = 0;
      break;
    }
  }



}

/*void loop(){
 if (newRC == true){
 newRC = false;
 GetSwitchPositions();
 Serial<<switchPositions<<"\r\n";
 }
 
 }*/

void GetSwitchPositions(){
  //value from gear switch
  if (RCValue[GEAR] < 1250){
    switchPositions = 0;
  }
  if (RCValue[GEAR] < 1650 && RCValue[GEAR] > 1350){
    switchPositions = 4;
  }
  if (RCValue[GEAR] > 1750){
    switchPositions = 8;
  }

  //value from aux 1
  if (RCValue[AUX1] < 1250){
    switchPositions += 0;
  }
  if (RCValue[AUX1] < 1650 && RCValue[AUX1] > 1350){
    switchPositions += 1;
  }
  if (RCValue[AUX1] > 1750){
    switchPositions += 2;
  }

}

void ModeSelect(){
  uint8_t selectState = 0;
  uint32_t timeDiff;
  //wait for mode input
  timer = millis();
  while(modeSelect == false){//---

    if (newRC == true){//+++
      newRC = false;
      ProcessChannels();
      Serial<<RCValue[AILE]<<","<<RCValue[ELEV]<<"\r\n";
      //to do add lights
      switch (selectState){

      case 0://wait for input
        digitalWrite(RED,HIGH);
        digitalWrite(YELLOW,LOW);
        digitalWrite(GREEN,LOW);
        timeDiff = millis() - timer; 
        if (timeDiff > 5000){

          modeSelect = true;
          trimMode = false;
          break;
        }
        if (RCValue[AILE] > 1800 && RCValue[ELEV] > 1800){
          selectState = 1;
          timer = millis();
        }
        break;

      case 1:
        digitalWrite(RED,LOW);
        digitalWrite(YELLOW,HIGH);
        digitalWrite(GREEN,LOW);

        if (RCValue[AILE] < 1800 || RCValue[ELEV] < 1800){
          selectState = 0;
        }
        if (((int32_t)millis() - (int32_t)timer) - (int32_t)timeDiff > 2500){
          modeSelect = true;
          trimMode = true;
          break;
        }
        break;
      }
    }//+++

  }//---
}

void GetMinMaxMid(){
  while(digitalRead(22)==0){//center
    if (newRC == true){
      newRC = false;
      Serial<<"Mid: "<<rawRCVal[AILE]<<","<<rawRCVal[ELEV]<<","<<rawRCVal[RUDD]<<","<<rawRCVal[THRO]<<","<<rawRCVal[GEAR]<<","<<rawRCVal[AUX1]<<","<<rawRCVal[AUX2]<<","<<rawRCVal[AUX3]<<"\r\n";
    }     
  }
  for (uint8_t i = THRO; i <= RUDD; i++){
    centerRCVal[i] = rawRCVal[i] ;
  }
  for (uint8_t i = THRO; i <= AUX3; i++){
    maxRCVal[i] = 1500;
    minRCVal[i] = 1500;
  }
  Serial<<"Center vals: "<<centerRCVal[AILE]<<","<<centerRCVal[ELEV]<<","<<centerRCVal[RUDD]<<","<<centerRCVal[THRO]<<"\r\n";
  delay(500);
  pause();
  while(digitalRead(22)==0){
    if (newRC == true){
      newRC = false;
      for (uint8_t i = THRO; i <= AUX3; i++){
        if (rawRCVal[i] > maxRCVal[i]){
          maxRCVal[i] = rawRCVal[i];
        }
        if (rawRCVal[i] < minRCVal[i]){
          minRCVal[i] = rawRCVal[i];
        }
      }
      Serial<<rawRCVal[AILE]<<","<<rawRCVal[ELEV]<<","<<rawRCVal[RUDD]<<","<<rawRCVal[THRO]<<","<<rawRCVal[GEAR]<<","<<rawRCVal[AUX1]<<","<<rawRCVal[AUX2]<<","<<rawRCVal[AUX3]<<"\r\n";

    }  
  }
  Serial<<"Max values: "<<maxRCVal[AILE]<<","<<maxRCVal[ELEV]<<","<<maxRCVal[RUDD]<<","<<maxRCVal[THRO]<<","<<maxRCVal[GEAR]<<","<<maxRCVal[AUX1]<<","<<maxRCVal[AUX2]<<","<<maxRCVal[AUX3]<<"\r\n";
  Serial<<"Min values: "<<minRCVal[AILE]<<","<<minRCVal[ELEV]<<","<<minRCVal[RUDD]<<","<<minRCVal[THRO]<<","<<minRCVal[GEAR]<<","<<minRCVal[AUX1]<<","<<minRCVal[AUX2]<<","<<minRCVal[AUX3]<<"\r\n";
  delay(500);
  pause();
  for (uint8_t i = THRO; i <= AUX3; i++){   
    if (i > THRO){
      RCOffset[i] = 1500 - (minRCVal[i] + (maxRCVal[i] - minRCVal[i]) * 0.5);
    }
    else{
      RCOffset[i] = 1500 - centerRCVal[i];
    }
    if (i == THRO){
      RCOffset[i] = 1500 - (minRCVal[i] + (maxRCVal[i] - minRCVal[i]) * 0.5);
    }
    RCScale[i] = 1000.0/( (float)maxRCVal[i] - (float)minRCVal[i] );
  }

  Serial<<"Offset values: "<<RCOffset[AILE]<<","<<RCOffset[ELEV]<<","<<RCOffset[RUDD]<<","<<RCOffset[THRO]<<","<<RCOffset[GEAR]<<","<<RCOffset[AUX1]<<","<<RCOffset[AUX2]<<","<<RCOffset[AUX3]<<"\r\n";
  Serial<<"Scale values: "<<RCScale[AILE]<<","<<RCScale[ELEV]<<","<<RCScale[RUDD]<<","<<RCScale[THRO]<<","<<RCScale[GEAR]<<","<<RCScale[AUX1]<<","<<RCScale[AUX2]<<","<<RCScale[AUX3]<<"\r\n";
  delay(500);
  pause();

}


void _200HzISRConfig(){
  TCCR5A = (1<<COM5A1);
  TCCR5B = (1<<CS51)|(1<<WGM52);
  TIMSK5 = (1<<OCIE5A);
  OCR5A = 10000;
}

ISR(TIMER5_COMPA_vect, ISR_NOBLOCK){
  if (rcType != RC){
    FeedLine();
  }
}




ISR(PCINT2_vect){
  currentPinState = PINK;
  changeMask = currentPinState ^ lastPinState;
  lastPinState = currentPinState;
  currentTime = micros();
  for(uint8_t i=0;i<8;i++){
    if(changeMask & 1<<i){//has there been a change
      if(!(currentPinState & 1<<i)){//is the pin in question logic low?
        timeDifference = currentTime - changeTime[i];//if so then calculate the pulse width
        if (900 < timeDifference && timeDifference < 2200){//check to see if it is a valid length
          //rcCommands.standardRCBuffer[i] = timeDifference;
          rawRCVal[i] = timeDifference;
          //rcCommands.standardRCBuffer[i] = (constrain((timeDifference ),1128,1924) - 1128) * 1.25628 + 1000;
          if (i == THRO && ((timeDifference ) < 1025)){
            failSafe = true;
          }
          else{
            newRC = true;
          }

        }
      }
      else{//the pin is logic high implying that this is the start of the pulse
        changeTime[i] = currentTime;
      }
    }
  }
}

void FeedLine(){

  switch(rcType){
  case 1:
    DSMXParser();
    break;
  case 2:
    SBusParser();
    break;
  }

}
void SBusParser(){
  while(Serial1.available() > 0){
    if (millis() - frameTime > 8){
      readState = 0;
    }
    inByte = Serial1.read();
    frameTime = millis();
    switch (readState){
    case 0:
      if (inByte == 0x0F){
        bufferIndex = 0;
        sBusData[bufferIndex] = inByte;
        sBusData[24] = 0xff;
        readState = 1;
      }

      break;
    case 1:
      bufferIndex ++;
      sBusData[bufferIndex] = inByte;

      if (bufferIndex == 24){
        readState = 0;
        if (sBusData[0]==0x0f && sBusData[24] == 0x00){
          newRC = true;
          rawRCVal[AILE] = (sBusData[1]|sBusData[2]<< 8) & 0x07FF ;
          rawRCVal[ELEV] = (sBusData[2]>>3|sBusData[3]<<5) & 0x07FF;
          rawRCVal[THRO] = (sBusData[3]>>6|sBusData[4]<<2|sBusData[5]<<10) & 0x07FF;
          rawRCVal[RUDD] = (sBusData[5]>>1|sBusData[6]<<7) & 0x07FF;
          rawRCVal[GEAR] = (sBusData[6]>>4|sBusData[7]<<4) & 0x07FF;
          rawRCVal[AUX1] = (sBusData[7]>>7|sBusData[8]<<1|sBusData[9]<<9) & 0x07FF;
          rawRCVal[AUX2] = (sBusData[9]>>2|sBusData[10]<<6) & 0x07FF;
          rawRCVal[AUX3] = (sBusData[10]>>5|sBusData[11]<<3) & 0x07FF;
          if (sBusData[23] & (1<<2)) {
            failSafe = true;
          }
          if (sBusData[23] & (1<<3)) {
            failSafe = true;
          }
        }
      }
      break;
    }
  }


}

void DSMXParser(){

  while (Serial1.available() > 0){

    if (millis() - frameTime > 8){
      byteCount = 0;
      bufferIndex = 0;
    }
    inByte = Serial1.read();
    frameTime = millis();
    byteCount++;
    if (byteCount > 2){
      spekBuffer[bufferIndex] = inByte;
      bufferIndex++;
    }
    if (byteCount == 16 && bufferIndex == 14){
      newRC = true;
      byteCount = 0;
      bufferIndex = 0;
      for (int i = 0; i < 14; i=i+2){
        channelNumber = (spekBuffer[i] >> 3) & 0x0F;
        rawRCVal[channelNumber] = ((spekBuffer[i] << 8) | (spekBuffer[i+1])) & 0x07FF;

      }
    }
  }
}

void DetectRC(){
  readState = 0;
  SBus();
  readState = 0;
  if (detected == true){
    FrameCheck();
    readState = 0;
    return;
  }
  readState = 0;
  Spektrum();
  readState = 0;
  if (detected == true){
    FrameCheck();
    readState = 0;
    return;
  }
  else{
    rcType = RC;
  }
  readState = 0;
  if (rcType == RC){
    DDRK = 0;//PORTK as input
    PORTK |= 0xFF;//turn on pull ups
    PCMSK2 |= 0xFF;//set interrupt mask for all of PORTK
    PCICR = 1<<2;//enable the pin change interrupt for K
    delay(100);//wait for a few frames
  } 


}


void FrameCheck(){//checks if serial RC was incorrectly detected
  newRC = false;
  generalPurposeTimer = millis();
  while (newRC == false){
    if (rcType == RC){
      delay(100);
    }
    if (rcType != RC){
      FeedLine();
    }
    if (millis() - generalPurposeTimer > 1000){//in case it has incorrectly detected serial RC
      rcType = RC;
      DDRK = 0;//PORTK as input
      PORTK |= 0xFF;//turn on pull ups
      PCMSK2 |= 0xFF;//set interrupt mask for all of PORTK
      PCICR = 1<<2;
      delay(100);//wait for a few frames
      generalPurposeTimer = millis();
    }
  } 
  newRC = false;

}

void SBus(){

  Serial1.begin(100000);
  generalPurposeTimer = millis();

  while (Serial1.available() == 0){
    if (millis() - generalPurposeTimer > 1000){
      return;
    }
  }

  delay(20);
  while(Serial1.available() > 0){
    inByte = Serial1.read();
    switch (readState){
    case 0:
      if (inByte == 0x0f){
        bufferIndex = 0;
        sBusData[bufferIndex] = inByte;
        sBusData[24] = 0xff;
        readState = 1;
      }
      break;
    case 1:
      bufferIndex ++;
      sBusData[bufferIndex] = inByte;
      if (bufferIndex == 24){
        readState = 0;
        if (sBusData[0]==0x0f && sBusData[24] == 0x00){
          rcType = SBUS;
          detected = true;
        }
      }
      break;
    }
  }
  frameTime = millis();
}
void Spektrum(){
  Serial1.begin(115200);
  generalPurposeTimer = millis();
  while (Serial1.available() == 0){
    if (millis() - generalPurposeTimer > 1000){
      return;
    }
  }  
  delay(5);
  while(Serial1.available() > 0){
    Serial1.read();
  }
  frameTime = millis();
  rcType = DSMX;
  detected = true;
}






































