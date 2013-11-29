//this checks to make sure the user is not commanding any throttle after calibration is complete
void SafetyCheck(){
  generalPurposeTimer = millis();
  while (rcCommands.values.throttle > 1020){
    /*if (rcType != RC){//removed since this is handled in the ISR
     FeedLine();
     }*/
    if (millis() - generalPurposeTimer > 500){
      digitalWrite(GREEN,toggle);
      toggle = ~toggle;
      generalPurposeTimer = millis();
    }
    if (newRC == true){
      newRC = false;
    }
  }
  digitalWrite(RED,HIGH);
  digitalWrite(YELLOW,HIGH);
  digitalWrite(GREEN,LOW);
  digitalWrite(13,LOW);
  throttleHold = true;
}
//move the rudder to the right to start calibration
void Arm(){
  //arming procedure
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
      DDRB &= 0xE0;
      PORTB |= 0x1F;//turn on pull ups
      PCMSK0 |= 0x1F;//set interrupt mask for all of PORTK
      PCICR |= 1<<0;//enable the pin change interrupt for K
      delay(100);//wait for a few frames
      Center();
      generalPurposeTimer = millis();
    }
  }
  newRC = false;
  while (rcCommands.values.rudder < 1750){
    if (rcType == RC){
      delay(100);//wait for a few frames
    }
    if (rcType != RC){
      FeedLine();
    }
    if (newRC == true){
      newRC = false;
    }
  } 
  digitalWrite(RED,LOW);
}



void Rotate2D(float *currentBearing, float *initialBearing, float *pitchIn, float *rollIn, float *pitchOut, float *rollOut){//change to take arguments
  static float headingFreeDifference = *currentBearing - *initialBearing;
  static float sinHeadingFreeDiff = sin(ToRad(headingFreeDifference));
  static float cosHeadingFreeDiff = cos(ToRad(headingFreeDifference));
  *rollOut = *rollIn * cosHeadingFreeDiff + *pitchIn * sinHeadingFreeDiff;
  *pitchOut = -1.0 * *rollIn * sinHeadingFreeDiff + *pitchIn * cosHeadingFreeDiff;

}

void SetAltHold(){
  targetAltitude = imu.ZEst;
  AltHoldPosition.reset();
  AltHoldRate.reset();  
}
void AltHold(){
  actualAltitude = imu.ZEst;//switch PID loop to use imu.Zest?
  AltHoldPosition.calculate();
  AltHoldRate.calculate();  
}

void SetXYLoiterPosition(){
  LoiterXPosition.reset();
  LoiterXRate.reset();
  LoiterYPosition.reset();
  LoiterYRate.reset();
  xTarget = imu.XEst;
  yTarget = imu.YEst;
}

void Loiter(){
  LoiterXPosition.calculate();
  LoiterYPosition.calculate();
  LoiterXRate.calculate();
  setPointX *= -1.0;
  LoiterYRate.calculate();
  Rotate2D(&imu.yaw,&zero,&setPointX,&setPointY,&pitchSetPoint,&rollSetPoint);
}

void GPSStable(){
  if (pitchSetPointTX == 0 && rollSetPointTX == 0){
    Loiter();
  }
  else{
    Rotate2D(&imu.yaw,&headingFreeInitial,&pitchSetPointTX,&rollSetPointTX,&pitchSetPoint,&rollSetPoint);
    SetXYLoiterPosition();
  }

  if (throttleCommand > (startingThrottle + 100)){
    targetVelAlt = 1.25;
    AltHoldRate.calculate();
    targetAltitude = imu.ZEst;
  }
  else{
    if(throttleCommand < (startingThrottle - 100)){
      targetVelAlt = - 1.25;
      AltHoldRate.calculate();
      targetAltitude = imu.ZEst;
    }
    else{
      AltHold();
    }
  }

}
void WayPointControl(){
  switch (wayPointState){
  case WP_HOLD:
    GPSStable();
    if (startFlight == true && inputWayPointNumber > 0){
      startFlight = false;
      wayPointState = WP_TRAVEL;
      CrossTrack.reset();
      currentWayPointNumber = 0;
      targetAltitude = (wayPoints[0].coord.alt * 0.001) - altitudeDifference;
      gps.DistBearing(&d.v.lattitude,&d.v.longitude,&wayPoints[0].coord.lat,&wayPoints[0].coord.lon,&wpXDist,&wpYDist,&distToWayPoint,&yawSetPoint);
      distToWayPoint *= -1;
      rollSetPoint = 0;
      SetXYLoiterPosition();
      GPSTimer = millis();
    }
    break;
  case WP_TRAVEL:
    if (fabs(targetAltitude - imu.ZEst) > 2){
      AltHold();
      Loiter();
    }
    else{
      AltHold();
      if (GPSPID == true){
        GPSPID = false;
        gps.DistBearing(&d.v.lattitude,&d.v.longitude,&wayPoints[0].coord.lat,&wayPoints[0].coord.lon,&wpXDist,&wpYDist,&distToWayPoint,&yawSetPoint);
        distToWayPoint *= -1;
        GPSDT = (millis() - GPSTimer) * 0.001;
        GPSTimer = millis();
        WayPointPosition.calculate();
        if (distToWayPoint >= -1 ){
          currentWayPointNumber++;
          targetAltitude = (wayPoints[currentWayPointNumber].coord.alt * 0.001) - altitudeDifference;
          SetXYLoiterPosition();
        }
      }
      Rotate2D(&imu.yaw,&zero,&imu.velX,&imu.velY,&velXBody,&velYBody);
      WayPointRate.calculate();
      pitchSetPoint *= -1;
      CrossTrack.calculate();
    }
    if (currentWayPointNumber == inputWayPointNumber){
      wayPointState = WP_END;
      SetXYLoiterPosition();
      SendEndOfWPCheck();
      endOfWPTimer = millis();
    }
    break;
  case WP_END:
    GPSStable();
    if (endOfWPCheck == true){
      wayPointState = WP_HOLD;
      break;
    }
    if (RTBFlag == true){
      wayPointState = WP_RTB;
      //targetAltitude = (homeBase.coord.alt * 0.001) - altitudeDifference + 4;
      break;
    }
    if (millis() - endOfWPTimer > 5000){
      wayPointState = WP_FAIL_RTB;
      RTBFailSafe = true;
      //targetAltitude = (homeBase.coord.alt * 0.001) - altitudeDifference + 4;
    }
    break;
  case WP_RTB:
    AltHold();
    if (GPSPID == true){
      GPSPID = false;
      gps.DistBearing(&d.v.lattitude,&d.v.longitude,&homeBase.coord.lat,&homeBase.coord.lon,&wpXDist,&wpYDist,&distToWayPoint,&yawSetPoint);
      distToWayPoint *= -1;
      GPSDT = (millis() - GPSTimer) * 0.001;
      GPSTimer = millis();
      WayPointPosition.calculate();
      if (distToWayPoint >= -1){
        targetAltitude = (homeBase.coord.alt * 0.001) - altitudeDifference + 4;
        SetXYLoiterPosition();
        wayPointState = WP_HOLD;
      }
    }
    Rotate2D(&imu.yaw,&zero,&imu.velX,&imu.velY,&velXBody,&velYBody);
    WayPointRate.calculate();
    pitchSetPoint *= -1;
    CrossTrack.calculate();



    break;
  case WP_FAIL_RTB:
    AltHold();
    if (GPSPID == true){
      GPSPID = false;
      gps.DistBearing(&d.v.lattitude,&d.v.longitude,&homeBase.coord.lat,&homeBase.coord.lon,&wpXDist,&wpYDist,&distToWayPoint,&yawSetPoint);
      distToWayPoint *= -1;
      GPSDT = (millis() - GPSTimer) * 0.001;
      GPSTimer = millis();
      WayPointPosition.calculate();
      if (distToWayPoint >= -1){
        SetXYLoiterPosition();
        targetAltitude = (homeBase.coord.alt * 0.001) - altitudeDifference + 4;
        d.v.flightMode = CARE_FREE;
        enterState = true;
      }
    }
    Rotate2D(&imu.yaw,&zero,&imu.velX,&imu.velY,&velXBody,&velYBody);
    WayPointRate.calculate();
    pitchSetPoint *= -1;
    CrossTrack.calculate();

    break;
  }

}


void HeadingHold(){
  switch (HHState){
  case TAKEOFF:
    calcYaw = false;
    rateSetPointZ = yawInput;
    if (rcCommands.values.throttle > LIFTOFF && abs(yawInput) < 1){
      yawSetPoint = imu.yaw;
      HHState = HH_ON;
    }
    if (rcCommands.values.throttle > LIFTOFF && abs(yawInput) > 1){
      HHState = HH_OFF;
    }
    break;
  case HH_ON:
    calcYaw = true;
    if (abs(yawInput) > 1){
      HHState = HH_OFF;
    }
    break;
  case HH_OFF:
    calcYaw = false;
    rateSetPointZ = yawInput;
    if (abs(yawInput) < 1){
      yawSetPoint = imu.yaw;
      HHState = HH_ON;
    }
    break;
  case LAND:
    calcYaw = false;
    yawSetPoint = imu.yaw;
    if (rcCommands.values.throttle > 1100){
      HHState = TAKEOFF;
    }
    break;
  }  
}




















