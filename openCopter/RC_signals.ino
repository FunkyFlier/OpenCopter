#include "AUXMATH.h"


void SetFailSafeFlags(){//remove - for debugging purposes only
  if (RCValue[AUX2] > 1800){
    gpsFailSafe = false;
    drFlag = false;
    imu.XEst.val = rawX.val;
    imu.YEst.val = rawY.val;
    drPosX.val = rawX.val;
    drPosY.val = rawY.val;
    imu.velX.val = 0;
    imu.velY.val = 0;
    drVelX.val = 0;
    drVelY.val = 0;
  }


}
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
  generalPurposeTimer = millis();
  while(modeSelect == false){//---

    if (newRC == true){//+++
      newRC = false;
      ProcessChannels();
      switch (selectState){

      case 0://wait for input
        digitalWrite(RED,HIGH);
        digitalWrite(YELLOW,LOW);
        digitalWrite(GREEN,LOW);
        digitalWrite(13,LOW);
        timeDiff = millis() - generalPurposeTimer; 
        if (timeDiff > 5000){

          modeSelect = true;
          trimMode = false;
          break;
        }
        if (RCValue[AILE] > 1800){
          digitalWrite(RED,HIGH);
          digitalWrite(YELLOW,LOW);
          digitalWrite(GREEN,HIGH);
          digitalWrite(13,LOW);
        }
        if (RCValue[ELEV] > 1800){
          digitalWrite(RED,HIGH);
          digitalWrite(YELLOW,LOW);
          digitalWrite(GREEN,LOW);
          digitalWrite(13,HIGH);
        }
        if (RCValue[AILE] > 1800 && RCValue[ELEV] > 1800){
          selectState = 1;
          generalPurposeTimer = millis();
        }
        break;

      case 1:
        digitalWrite(RED,LOW);
        digitalWrite(YELLOW,HIGH);
        digitalWrite(GREEN,LOW);
        digitalWrite(13,LOW);

        if (RCValue[AILE] < 1800 || RCValue[ELEV] < 1800){
          selectState = 0;
        }
        if (((int32_t)millis() - (int32_t)generalPurposeTimer) - (int32_t)timeDiff > 2500){
          modeSelect = true;
          trimMode = true;
          break;
        }
        break;
      }
    }//+++

  }//---
}

void CheckTXPositions(){
  boolean positionOK = false;
  while (positionOK == false){
    StartUpAHRSRun();
    digitalWrite(RED,LOW);
    digitalWrite(YELLOW,LOW);
    digitalWrite(GREEN,HIGH);
    digitalWrite(13,HIGH);
    if (newRC == true){
      newRC = false;
      ProcessChannels();
    } 
    positionOK = true;
    if (RCValue[THRO] > 1050){
      positionOK = false;
    }
    if (RCValue[GEAR] > 1050){
      positionOK = false;
    }
    if (RCValue[AUX1] > 1050){
      positionOK = false;
    }
    if (RCValue[AUX2] > 1050){
      positionOK = false;
    }
    if (RCValue[AUX3] > 1050){
      positionOK = false;
    }
  }
}
void ProcessChannels(){

  previousFlightMode = flightMode;


  RCValue[THRO] = (rawRCVal[THRO] - minRCVal[THRO]) * RCScale[THRO] + 1000;
  RCValue[GEAR] = (rawRCVal[GEAR] - minRCVal[GEAR]) * RCScale[GEAR] + 1000;
  RCValue[AUX1] = (rawRCVal[AUX1] - minRCVal[AUX1]) * RCScale[AUX1] + 1000;
  RCValue[AUX2] = (rawRCVal[AUX2] - minRCVal[AUX2]) * RCScale[AUX2] + 1000;
  RCValue[AUX3] = (rawRCVal[AUX3] - minRCVal[AUX3]) * RCScale[AUX3] + 1000;

  RCValue[AILE] = (rawRCVal[AILE] - centerRCVal[AILE]) * RCScale[AILE] + 1500;
  RCValue[ELEV] = (rawRCVal[ELEV] - centerRCVal[ELEV]) * RCScale[ELEV] + 1500 ;
  RCValue[RUDD] = (rawRCVal[RUDD] - centerRCVal[RUDD]) * RCScale[RUDD] + 1500 ;



  if (txFailSafe == true){
    if (motorState >= FLIGHT){
      if (flightMode != RTB){
        enterState = true;
        flightMode = RTB;
      }
    }
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
        clearTXRTB = 0;
        break;
      }
      return;
      break;

    }


  }
  else{
    clearTXRTB = 0;
  }

  if (RCValue[AUX3] > 1750){
    flightMode = ATT;
    setTrim = true;
    trimComplete = true;
    MapVar(&RCValue[ELEV],&pitchSetPoint.val,1000,2000,-60,60);
    MapVar(&RCValue[AILE],&rollSetPoint.val,1000,2000,-60,60);
    MapVar(&RCValue[RUDD],&yawInput,1000,2000,-300,300);
    if (rollSetPoint.val < 1 && rollSetPoint.val > -1){
      rollSetPoint.val = 0;
    }
    if (pitchSetPoint.val < 1 && pitchSetPoint.val > -1){
      pitchSetPoint.val = 0;
    }
    if (yawInput < 5 && yawInput > -5){
      yawInput = 0;
    }
    if (flightMode != previousFlightMode){
      enterState = true;
    }

    return;

  }


  if (trimMode == false){


    switch (switchPositions){
    case 0:
      flightMode = L0;
      MapVar(&RCValue[AILE],&rollSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[ELEV],&pitchSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[RUDD],&yawInput,1000,2000,-300,300);
      if (rollSetPointTX.val < 1 && rollSetPointTX.val > -1){
        rollSetPointTX.val = 0;
      }
      if (pitchSetPointTX.val < 1 && pitchSetPointTX.val > -1){
        pitchSetPointTX.val = 0;
      }
      if (yawInput < 5 && yawInput > -5){
        yawInput = 0;
      }
      break;
    case 1:
      flightMode = L1;
      MapVar(&RCValue[AILE],&rollSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[ELEV],&pitchSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[RUDD],&yawInput,1000,2000,-300,300);
      if (rollSetPointTX.val < 1 && rollSetPointTX.val > -1){
        rollSetPointTX.val = 0;
      }
      if (pitchSetPointTX.val < 1 && pitchSetPointTX.val > -1){
        pitchSetPointTX.val = 0;
      }
      if (yawInput < 5 && yawInput > -5){
        yawInput = 0;
      }
      break;
    case 2:
      flightMode = L2;
      MapVar(&RCValue[AILE],&rollSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[ELEV],&pitchSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[RUDD],&yawInput,1000,2000,-300,300);
      if (rollSetPointTX.val < 1 && rollSetPointTX.val > -1){
        rollSetPointTX.val = 0;
      }
      if (pitchSetPointTX.val < 1 && pitchSetPointTX.val > -1){
        pitchSetPointTX.val = 0;
      }
      if (yawInput < 5 && yawInput > -5){
        yawInput = 0;
      }
      if (gpsFailSafe == true){
        flightMode = L1;

      }
      break;

    case 4:
      flightMode = FOLLOW;
      MapVar(&RCValue[AILE],&rollSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[ELEV],&pitchSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[RUDD],&yawInput,1000,2000,-300,300);
      if (rollSetPointTX.val < 1 && rollSetPointTX.val > -1){
        rollSetPointTX.val = 0;
      }
      if (pitchSetPointTX.val < 1 && pitchSetPointTX.val > -1){
        pitchSetPointTX.val = 0;
      }
      if (yawInput < 5 && yawInput > -5){
        yawInput = 0;
      }
      if (gpsFailSafe == true){
        flightMode = L0;
      }
      if (telemFailSafe == true){
        flightMode = L0;
      }
      break;
    case 5:
      flightMode = WP;
      MapVar(&RCValue[AILE],&rollSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[ELEV],&pitchSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[RUDD],&yawInput,1000,2000,-300,300);
      if (rollSetPointTX.val < 1 && rollSetPointTX.val > -1){
        rollSetPointTX.val = 0;
      }
      if (pitchSetPointTX.val < 1 && pitchSetPointTX.val > -1){
        pitchSetPointTX.val = 0;
      }
      if (yawInput < 5 && yawInput > -5){
        yawInput = 0;
      }
      if (gpsFailSafe == true){
        flightMode = L0;
      }
      if (telemFailSafe == true){
        flightMode = RTB;
      }
      break;
    case 6:
      flightMode = L0;
      MapVar(&RCValue[AILE],&rollSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[ELEV],&pitchSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[RUDD],&yawInput,1000,2000,-300,300);
      if (rollSetPointTX.val < 1 && rollSetPointTX.val > -1){
        rollSetPointTX.val = 0;
      }
      if (pitchSetPointTX.val < 1 && pitchSetPointTX.val > -1){
        pitchSetPointTX.val = 0;
      }
      if (yawInput < 5 && yawInput > -5){
        yawInput = 0;
      }
      break;

    case 8:
      //break;
    case 9:
      //break;
    case 10:
      flightMode = RTB;
      MapVar(&RCValue[AILE],&rollSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[ELEV],&pitchSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[RUDD],&yawInput,1000,2000,-300,300);
      if (rollSetPointTX.val < 1 && rollSetPointTX.val > -1){
        rollSetPointTX.val = 0;
      }
      if (pitchSetPointTX.val < 1 && pitchSetPointTX.val > -1){
        pitchSetPointTX.val = 0;
      }
      if (yawInput < 5 && yawInput > -5){
        yawInput = 0;
      }
      break;
    }

  }
  else{
    switch (switchPositions){
    case 0:
    case 1:
    case 2:
      flightMode = L0;
      MapVar(&RCValue[AILE],&rollSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[ELEV],&pitchSetPointTX.val,1000,2000,-60,60);
      MapVar(&RCValue[RUDD],&yawInput,1000,2000,-300,300);
      if (rollSetPointTX.val < 1 && rollSetPointTX.val > -1){
        rollSetPointTX.val = 0;
      }
      if (pitchSetPointTX.val < 1 && pitchSetPointTX.val > -1){
        pitchSetPointTX.val = 0;
      }
      if (yawInput < 5 && yawInput > -5){
        yawInput = 0;
      }
      break;

    case 4:
    case 5:
      flightMode = ATT;
      setTrim = false;
      trimComplete = false;
      MapVar(&RCValue[ELEV],&pitchSetPoint.val,1000,2000,-60,60);
      MapVar(&RCValue[AILE],&rollSetPoint.val,1000,2000,-60,60);
      MapVar(&RCValue[RUDD],&yawInput,1000,2000,-300,300);
      if (rollSetPoint.val < 1 && rollSetPoint.val > -1){
        rollSetPoint.val = 0;
      }
      if (pitchSetPoint.val < 1 && pitchSetPoint.val > -1){
        pitchSetPoint.val = 0;
      }
      if (yawInput < 5 && yawInput > -5){
        yawInput = 0;
      }
      break;
    case 6:
      flightMode = ATT;
      setTrim = true;
      MapVar(&RCValue[ELEV],&pitchSetPoint.val,1000,2000,-60,60);
      MapVar(&RCValue[AILE],&rollSetPoint.val,1000,2000,-60,60);
      MapVar(&RCValue[RUDD],&yawInput,1000,2000,-300,300);
      if (rollSetPoint.val < 1 && rollSetPoint.val > -1){
        rollSetPoint.val = 0;
      }
      if (pitchSetPoint.val < 1 && pitchSetPoint.val > -1){
        pitchSetPoint.val = 0;
      }
      if (yawInput < 5 && yawInput > -5){
        yawInput = 0;
      }
      break;

    case 8:
    case 9:
      flightMode = RATE;
      setTrim = false;
      trimComplete = false;
      MapVar(&RCValue[ELEV],&rateSetPointY.val,1000,2000,-400,400);
      MapVar(&RCValue[AILE],&rateSetPointX.val,1000,2000,-400,400);
      MapVar(&RCValue[RUDD],&rateSetPointZ.val,1000,2000,-400,400);
      if (rateSetPointY.val < 5 && rateSetPointY.val > -5){
        rateSetPointY.val = 0;
      }
      if (rateSetPointX.val < 5 && rateSetPointX.val > -5){
        rateSetPointX.val = 0;
      }
      if (rateSetPointZ.val < 5 && rateSetPointZ.val > -5){
        rateSetPointZ.val = 0;
      }
      break;
    case 10:
      setTrim = true;
      flightMode = RATE;
      MapVar(&RCValue[ELEV],&rateSetPointY.val,1000,2000,-400,400);
      MapVar(&RCValue[AILE],&rateSetPointX.val,1000,2000,-400,400);
      MapVar(&RCValue[RUDD],&rateSetPointZ.val,1000,2000,-400,400);
      if (rateSetPointY.val < 5 && rateSetPointY.val > -5){
        rateSetPointY.val = 0;
      }
      if (rateSetPointX.val < 5 && rateSetPointX.val > -5){
        rateSetPointX.val = 0;
      }
      if (rateSetPointZ.val < 5 && rateSetPointZ.val > -5){
        rateSetPointZ.val = 0;
      }
      break;
    }

  }

  if (flightMode != previousFlightMode){
    enterState = true;
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
  while(RCSigPort.available() > 0){
    if (millis() - frameTime > 8){
      readState = 0;
    }
    inByte = RCSigPort.read();
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

  while (RCSigPort.available() > 0){

    if (millis() - frameTime > 8){
      byteCount = 0;
      bufferIndex = 0;
    }
    inByte = RCSigPort.read();
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
  //Spektrum();
  
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
    PCICR |= 1<<2;//enable the pin change interrupt for K
    delay(100);//wait for a few frames
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
          rawRCVal[i] = timeDifference;
          if (i == THRO && ((timeDifference ) < 1000)){
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
      PCICR |= 1<<2;
      delay(100);//wait for a few frames
      generalPurposeTimer = millis();
    }
  } 
  newRC = false;

}




void SBus(){

  RCSigPort.begin(100000);
  generalPurposeTimer = millis();

  while (RCSigPort.available() == 0){
    if (millis() - generalPurposeTimer > 1000){
      return;
    }
  }

  delay(20);
  while(RCSigPort.available() > 0){
    inByte = RCSigPort.read();
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
  RCSigPort.begin(115200);
  generalPurposeTimer = millis();
  while (RCSigPort.available() == 0){
    if (millis() - generalPurposeTimer > 1000){
      return;
    }
  }  
  delay(5);
  while(RCSigPort.available() > 0){
    RCSigPort.read();
  }
  frameTime = millis();
  rcType = DSMX;
  detected = true;
}













