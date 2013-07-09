void ProcessChannels(){
  //this function processes the signals from the transmitter
  if (rcCommands.values.gear < 1500){
    //normal mode
    MapVar(&rcCommands.values.aileron,&rollSetPoint,1000,2000,-60,60);
    MapVar(&rcCommands.values.elevator,&pitchSetPoint,1000,2000,-60,60);
    MapVar(&rcCommands.values.rudder,&rateSetPointZ,1000,2000,-300,300);
    //dead zone
    if (rollSetPoint < 2 && rollSetPoint > -2){
      rollSetPoint = 0;
    }
    if (pitchSetPoint < 2 && pitchSetPoint > -2){
      pitchSetPoint = 0;
    }
  }
  else{
    //stunt mode
    //the aircraft will flip very quickly at the maximum extents of the right stick
    MapVar(&rcCommands.values.aileron,&rateSetPointX,1000,2000,-300,300);
    MapVar(&rcCommands.values.elevator,&rateSetPointY,1000,2000,-300,300);
    MapVar(&rcCommands.values.rudder,&rateSetPointZ,1000,2000,-300,300);
    //increased rate for flips
    if (rcCommands.values.aileron > 1950){
      rateSetPointX = 600.0;
    }
    if (rcCommands.values.aileron < 1050){
      rateSetPointX = -600.0;
    }
    if (rcCommands.values.elevator > 1950){
      rateSetPointY = 600.0;
    }
    if (rcCommands.values.elevator < 1050){
      rateSetPointY = -600.0;
    }
    //dead zone
    if (rateSetPointY < 2 && rateSetPointY > -2){
      rateSetPointY= 0; 
    }  
    if (rateSetPointX < 2 && rateSetPointX > -2){
      rateSetPointX = 0; 
    }      
  }
  //dead zone
  if (rateSetPointZ < 2 && rateSetPointZ > -2){
    rateSetPointZ = 0;
  }
  //throttle check
  //do not integrate unless throttle is near take off
  //this prevents integral windup and possibly flipping the aircraft on take off
  if (rcCommands.values.throttle > LIFTOFF){
    integrate = true;
  }    
  //this limits the maximum throttle command
  //the purpose of this is so that at maximum throttle the craft will still be controllable
  if (rcCommands.values.throttle > 1900){
    rcCommands.values.throttle = 1900;
  }

}

void Center(){

  while (newRC == false){
    delay(1);
  }

  offset = rcCommands.values.aileron - 1500;
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
          rcCommands.standardRCBuffer[i] = (constrain((timeDifference - offset),1080,1920) - 1080) * 1.19 + 1000;
          if (i != 2){ //fail safe - in loss of signal all channels stop except for the throttle
            newRC = true;
          }
          if (i == 2 && ((timeDifference ) < 1025)){//fail safe for futaba / DSMx
            failSafe = true;
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
    inByte = Serial1.read();
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
        }
      }
      break;
    }
  }

  if (newRC == true){
    //credit to the folks at multiWii for this sBus parsing algorithm
    rcCommands.values.aileron  = constrain(((sBusData[1]|sBusData[2]<< 8) & 0x07FF),352,1695) ;
    rcCommands.values.aileron  = (rcCommands.values.aileron  - 352) * 0.7446 + 1000;
    rcCommands.values.elevator  = constrain(((sBusData[2]>>3|sBusData[3]<<5) & 0x07FF),352,1695);
    rcCommands.values.elevator  = (rcCommands.values.elevator  - 352) * 0.7446 + 1000;
    rcCommands.values.throttle  = constrain(((sBusData[3]>>6|sBusData[4]<<2|sBusData[5]<<10) & 0x07FF),352,1695);
    rcCommands.values.throttle  = (rcCommands.values.throttle  - 352) * 0.7446 + 1000;
    rcCommands.values.rudder  = constrain(((sBusData[5]>>1|sBusData[6]<<7) & 0x07FF),352,1695);
    rcCommands.values.rudder  = (rcCommands.values.rudder  - 352) * 0.7446 + 1000;
    rcCommands.values.gear = constrain(((sBusData[6]>>4|sBusData[7]<<4) & 0x07FF),352,1695);
    rcCommands.values.gear  = (rcCommands.values.gear  - 352) * 0.7446 + 1000;
    rcCommands.values.aux1 = constrain(((sBusData[7]>>7|sBusData[8]<<1|sBusData[9]<<9) & 0x07FF),352,1695);
    rcCommands.values.aux1  = (rcCommands.values.aux1  - 352) * 0.7446 + 1000;
    rcCommands.values.aux2  = constrain(((sBusData[9]>>2|sBusData[10]<<6) & 0x07FF),352,1695);
    rcCommands.values.aux2  = (rcCommands.values.aux2  - 352) * 0.7446 + 1000;
    rcCommands.values.aux3  = constrain(((sBusData[10]>>5|sBusData[11]<<3) & 0x07FF),352,1695);
    rcCommands.values.aux3  = (rcCommands.values.aux3  - 352) * 0.7446 + 1000;
    if (sBusData[23] & (1<<2)) {
      failSafe = true;
    }
    if (sBusData[23] & (1<<3)) {
      failSafe = true;
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
        switch(channelNumber){
        case 0://throttle
          rcCommands.values.throttle = constrain(((spekBuffer[i] << 8) | (spekBuffer[i+1])) & 0x07FF,342,1706);
          rcCommands.values.throttle = (rcCommands.values.throttle - 342) * 0.7331378 + 1000;
          break;
        case 1://aileron
          rcCommands.values.aileron = constrain(((spekBuffer[i] << 8) | (spekBuffer[i+1])) & 0x07FF,342,1706);
          rcCommands.values.aileron = (rcCommands.values.aileron - 342) * 0.7331378 + 1000;
          break;
        case 2://elevator
          rcCommands.values.elevator = constrain(((spekBuffer[i] << 8) | (spekBuffer[i+1])) & 0x07FF,342,1706);
          rcCommands.values.elevator = (rcCommands.values.elevator - 342) * 0.7331378 + 1000;
          break;
        case 3://rudder
          rcCommands.values.rudder = constrain(((spekBuffer[i] << 8) | (spekBuffer[i+1])) & 0x07FF,342,1706);
          rcCommands.values.rudder = (rcCommands.values.rudder - 342) * 0.7331378 + 1000;
          break;
        case 4://gear
          rcCommands.values.gear = constrain(((spekBuffer[i] << 8) | (spekBuffer[i+1])) & 0x07FF,342,1706);
          rcCommands.values.gear = (rcCommands.values.gear - 342) * 0.7331378 + 1000;
          break;
        case 5://aux1
          rcCommands.values.aux1 = constrain(((spekBuffer[i] << 8) | (spekBuffer[i+1])) & 0x07FF,342,1706);
          rcCommands.values.aux1 = (rcCommands.values.aux1 - 342) * 0.7331378 + 1000;
          break;
        case 6://aux2
          rcCommands.values.aux2 = constrain(((spekBuffer[i] << 8) | (spekBuffer[i+1])) & 0x07FF,342,1706);
          rcCommands.values.aux2 = (rcCommands.values.aux2 - 342) * 0.7331378 + 1000;
          break;
        case 7://aux3
          rcCommands.values.aux3 = constrain(((spekBuffer[i] << 8) | (spekBuffer[i+1])) & 0x07FF,342,1706);
          rcCommands.values.aux3 = (rcCommands.values.aux3 - 342) * 0.7331378 + 1000;
          break;
        default:
          break;
        }

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
    Center();
  } 


}
void FrameCheck(){//checks if serial RC was incorrectly detected
  newRC = false;
  timer = millis();
  while (newRC == false){
    if (rcType == RC){
      delay(100);
    }
    if (rcType != RC){
      FeedLine();
    }
    if (millis() - timer > 1000){//in case it has incorrectly detected serial RC
      rcType = RC;
      DDRK = 0;//PORTK as input
      PORTK |= 0xFF;//turn on pull ups
      PCMSK2 |= 0xFF;//set interrupt mask for all of PORTK
      PCICR = 1<<2;//enable the pin change interrupt for K
      delay(100);//wait for a few frames
      Center();
      timer = millis();
    }
  } 
}
void SBus(){

  Serial1.begin(100000);
  timer = millis();
  while(Serial1.available() > 0){
    Serial1.read();
  }
  while (Serial1.available() == 0){
    if (millis() - timer > 1000){
      return;
    }
  }

  delay(10);
  if (Serial1.available() > 24){
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
  }  
}

void Spektrum(){
  Serial1.begin(115200);
  timer = millis();
  while (Serial1.available() == 0){
    if (millis() - timer > 1000){
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













