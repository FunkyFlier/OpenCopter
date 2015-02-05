void CheckESCFlag(){

  if (EEPROM.read(428) == 0xAA){
    DDRE |= B00111000;
    DDRH |= B00111000;


    TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);               
    ICR3 = PERIOD;   

    TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    ICR4 = PERIOD;  
    Motor1WriteMicros(2000);//set the output compare value
    Motor2WriteMicros(2000);
    Motor3WriteMicros(2000);
    Motor4WriteMicros(2000);
    Motor5WriteMicros(2000);
    Motor6WriteMicros(2000);
    delay(4000);
    Motor1WriteMicros(1000);//set the output compare value
    Motor2WriteMicros(1000);
    Motor3WriteMicros(1000);
    Motor4WriteMicros(1000);
    Motor5WriteMicros(1000);
    Motor6WriteMicros(1000);
    EEPROM.write(428,0xFF);
    while(1){
      digitalWrite(13,LOW);
      digitalWrite(RED,HIGH);
      digitalWrite(YELLOW,HIGH);
      digitalWrite(GREEN,HIGH);
      delay(250);
      digitalWrite(13,HIGH);
      digitalWrite(RED,LOW);
      digitalWrite(YELLOW,HIGH);
      digitalWrite(GREEN,HIGH);
      delay(250);
      digitalWrite(13,HIGH);
      digitalWrite(RED,HIGH);
      digitalWrite(YELLOW,LOW);
      digitalWrite(GREEN,HIGH);
      delay(250);
      digitalWrite(13,HIGH);
      digitalWrite(RED,HIGH);
      digitalWrite(YELLOW,HIGH);
      digitalWrite(GREEN,LOW);
      delay(250);
    }
  }
}

void CalibrateESC(){
  delay(100);//wait for new frame

  while(newRC == false){

  }
  ProcessChannels();
  if (RCValue[THRO] > 1900){
    EEPROM.write(0x3E8,0xFF);//clear the handshake flag
    EEPROM.write(428,0xAA);
    while(1){
      digitalWrite(13,HIGH);
      digitalWrite(RED,LOW);
      digitalWrite(YELLOW,LOW);
      digitalWrite(GREEN,LOW);
      delay(250);
      digitalWrite(13,LOW);
      digitalWrite(RED,HIGH);
      digitalWrite(YELLOW,LOW);
      digitalWrite(GREEN,LOW);
      delay(250);
      digitalWrite(13,LOW);
      digitalWrite(RED,LOW);
      digitalWrite(YELLOW,HIGH);
      digitalWrite(GREEN,LOW);
      delay(250);
      digitalWrite(13,LOW);
      digitalWrite(RED,LOW);
      digitalWrite(YELLOW,LOW);
      digitalWrite(GREEN,HIGH);
      delay(250);
    }

  }

}


void MotorInit(){
  DDRE |= B00111000;
  DDRH |= B00111000;
  //DDRB |= B01100000;


  TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);               
  ICR3 = PERIOD;   

  TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
  TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
  ICR4 = PERIOD;  

  //TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
  //TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
  //ICR1 = PERIOD;

  Motor1WriteMicros(1000);//set the output compare value
  Motor2WriteMicros(1000);
  Motor3WriteMicros(1000);
  Motor4WriteMicros(1000);
  Motor5WriteMicros(1000);
  Motor6WriteMicros(1000);

}


void MotorHandler(){
  /*    if (saveGainsFlag == true && (millis() - romWriteDelayTimer) > 2000){
   j_ = 81;
   for(uint16_t i = KP_PITCH_RATE_; i <= MAG_DEC_; i++){
   EEPROM.write(j_++,(*floatPointerArray[i]).buffer[0]); 
   EEPROM.write(j_++,(*floatPointerArray[i]).buffer[1]); 
   EEPROM.write(j_++,(*floatPointerArray[i]).buffer[2]); 
   EEPROM.write(j_++,(*floatPointerArray[i]).buffer[3]); 
   watchDogFailSafeCounter = 0;
   }
   calibrationFlags = EEPROM.read(0x00);
   calibrationFlags &= ~(1<<GAINS_FLAG);
   EEPROM.write(0x00,calibrationFlags);
   saveGainsFlag = false;
   UpdateOffset();
   imuTimer = micros();
   baroTimer = millis();
   _400HzTimer = imuTimer;
   
   }*/
  switch(motorState){
  case HOLD:

    if (saveGainsFlag == true && (millis() - romWriteDelayTimer) > 2000){
      j_ = 81;
      for(uint16_t i = KP_PITCH_RATE_; i <= MAG_DEC_; i++){
        EEPROM.write(j_++,(*floatPointerArray[i]).buffer[0]); 
        EEPROM.write(j_++,(*floatPointerArray[i]).buffer[1]); 
        EEPROM.write(j_++,(*floatPointerArray[i]).buffer[2]); 
        EEPROM.write(j_++,(*floatPointerArray[i]).buffer[3]); 
        watchDogFailSafeCounter = 0;
      }
      calibrationFlags = EEPROM.read(0x00);
      calibrationFlags &= ~(1<<GAINS_FLAG);
      EEPROM.write(0x00,calibrationFlags);
      saveGainsFlag = false;
      imuTimer = micros();
      baroTimer = millis();
      _400HzTimer = imuTimer;
    }
    //pressureInitial = pressure;
    initialYaw = imu.yaw.val;
    integrate = false;
    HHState = 0;

    ZLoiterState = LOITERING;
    XYLoiterState = LOITERING;
    if (RCValue[THRO] > 1100){
      motorCommand1.val = 1000;
      motorCommand2.val = 1000;
      motorCommand3.val = 1000;
      motorCommand4.val = 1000;
      break;
    }
    if (flightMode == RTB){
      motorState = HOLD;
      motorCommand1.val = 1000;
      motorCommand2.val = 1000;
      motorCommand3.val = 1000;
      motorCommand4.val = 1000;
      break;
    }

    if (RCValue[RUDD] < 1300){
      motorState = TO;


      PitchAngle.reset();
      RollAngle.reset();
      YawAngle.reset();

      PitchRate.reset();
      RollRate.reset();
      YawRate.reset();

      AltHoldPosition.reset();
      AltHoldVelocity.reset();

      WayPointPosition.reset();
      WayPointRate.reset();

      LoiterXPosition.reset();
      LoiterXVelocity.reset();

      LoiterYPosition.reset();
      LoiterYVelocity.reset();
      homeBaseXOffset = imu.XEst.val;
      homeBaseYOffset = imu.YEst.val;
      UpdateOffset();
    }

    motorCommand1.val = 1000;
    motorCommand2.val = 1000;
    motorCommand3.val = 1000;
    motorCommand4.val = 1000;
    throttleCheckFlag = false;
    break;
  case TO:
    motorCommand1.val = 1125;
    motorCommand2.val = 1125;
    motorCommand3.val = 1125;
    motorCommand4.val = 1125;
    throttleCheckFlag = false;
    pressureInitial = pressure.val;
    imu.ZEst.val = 0;
	imu.ZEstUp.val = 0;
    imu.velZ.val = 0;
	imu.velZUp.val = 0;
    prevBaro = 0;
    baroZ.val = 0;
    //baroTimer = millis();
    initialYaw = imu.yaw.val;

    if (RCValue[RUDD] > 1700){
      motorState = HOLD;
    }
    if (flightMode == RTB){
      motorState = HOLD;
    }

    if (flightMode == RATE || flightMode == ATT){
      if (RCValue[THRO] > 1150 && RCValue[THRO] < 1350){
        motorState = FLIGHT;
        integrate = true;
      }
    }
    if (flightMode <= L2 && flightMode >= L0){
      if (RCValue[THRO] <= 1600 && RCValue[THRO] >= 1500){
        motorState = FLIGHT;
        zTarget.val = TAKE_OFF_ALT;
        enterState = true;
        throttleAdjustment.val = 0;
        xTarget.val = imu.XEst.val;
        yTarget.val = imu.YEst.val;
        LoiterXPosition.reset();
        LoiterXVelocity.reset();
        LoiterYPosition.reset();
        LoiterYVelocity.reset();
        AltHoldPosition.reset();
        AltHoldVelocity.reset();
        integrate = true;
      }
    }
    if (flightMode == WP || flightMode == FOLLOW){
      if (RCValue[THRO] <= 1600 && RCValue[THRO] >= 1500){
        autoMaticReady = true;
      }
    }

    break;
  case FLIGHT:
    if (flightMode == RATE || flightMode == ATT){
      throttleAdjustment.val = 0;
      throttleCommand = RCValue[THRO];
      if (throttleCommand > 1900){
        throttleCommand = 1900;
      }
      if (throttleCommand < 1050){
        motorState = HOLD;
      }
    }
    if (flightMode >= L0){
      throttleCommand = 1550;
    }
    if (throttleCheckFlag == true){
      if (RCValue[THRO] <= 1600 && RCValue[THRO] >= 1500){
        throttleCheckFlag = false;
        throttleCommand = 1550;
      }
    }

    motorCommand1.val = constrain((throttleCommand + throttleAdjustment.val + adjustmentX.val + adjustmentY.val - adjustmentZ.val),1000,2000);
    motorCommand2.val = constrain((throttleCommand + throttleAdjustment.val - adjustmentX.val + adjustmentY.val + adjustmentZ.val),1000,2000);
    motorCommand3.val = constrain((throttleCommand + throttleAdjustment.val - adjustmentX.val - adjustmentY.val - adjustmentZ.val),1000,2000);
    motorCommand4.val = constrain((throttleCommand + throttleAdjustment.val + adjustmentX.val - adjustmentY.val + adjustmentZ.val),1000,2000);
    /*motorCommand1.val = constrain((throttleCommand + throttleAdjustment.val + adjustmentY.val - adjustmentZ.val),1000,2000);
     motorCommand2.val = constrain((throttleCommand + throttleAdjustment.val - adjustmentX.val + adjustmentZ.val),1000,2000);
     motorCommand3.val = constrain((throttleCommand + throttleAdjustment.val - adjustmentY.val - adjustmentZ.val),1000,2000);
     motorCommand4.val = constrain((throttleCommand + throttleAdjustment.val + adjustmentX.val + adjustmentZ.val),1000,2000);*/

    break;
  case LANDING:
    if (flightMode == RATE || flightMode == ATT){
      motorState = FLIGHT;
    }
    if (throttleCheckFlag == true){
      if (RCValue[THRO] <= 1600 && RCValue[THRO] >= 1500){
        throttleCheckFlag = false;
      }
    }
    throttleCommand = 1550;
    if ( (1550 + throttleAdjustment.val) < 1250){
      motorCommand1.val = 1000;
      motorCommand2.val = 1000;
      motorCommand3.val = 1000;
      motorCommand4.val = 1000;
      motorState = HOLD;
      break;
    }
    if (RCValue[RUDD] > 1950){
      motorCommand1.val = 1000;
      motorCommand2.val = 1000;
      motorCommand3.val = 1000;
      motorCommand4.val = 1000;
      motorState = HOLD;
      break;
    }
    if (fabs(imu.inertialZ.val) > 5.0){
      motorCommand1.val = 1000;
      motorCommand2.val = 1000;
      motorCommand3.val = 1000;
      motorCommand4.val = 1000;
      motorState = HOLD;
      break;
    }

    motorCommand1.val = constrain((throttleCommand + throttleAdjustment.val + adjustmentX.val + adjustmentY.val - adjustmentZ.val),1000,2000);
    motorCommand2.val = constrain((throttleCommand + throttleAdjustment.val - adjustmentX.val + adjustmentY.val + adjustmentZ.val),1000,2000);
    motorCommand3.val = constrain((throttleCommand + throttleAdjustment.val - adjustmentX.val - adjustmentY.val - adjustmentZ.val),1000,2000);
    motorCommand4.val = constrain((throttleCommand + throttleAdjustment.val + adjustmentX.val - adjustmentY.val + adjustmentZ.val),1000,2000);
    /*    motorCommand1.val = constrain((throttleCommand + throttleAdjustment.val + adjustmentY.val - adjustmentZ.val),1000,2000);
     motorCommand2.val = constrain((throttleCommand + throttleAdjustment.val - adjustmentX.val + adjustmentZ.val),1000,2000);
     motorCommand3.val = constrain((throttleCommand + throttleAdjustment.val - adjustmentY.val - adjustmentZ.val),1000,2000);
     motorCommand4.val = constrain((throttleCommand + throttleAdjustment.val + adjustmentX.val + adjustmentZ.val),1000,2000);*/
    break;
  }

  Motor1WriteMicros(motorCommand1.val);
  Motor2WriteMicros(motorCommand2.val);
  Motor3WriteMicros(motorCommand3.val);
  Motor4WriteMicros(motorCommand4.val);


}


