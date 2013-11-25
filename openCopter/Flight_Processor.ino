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



void RotatePitchRoll(float *currentBearing, float *initialBearing, float *pitchIn, float *rollIn, float *pitchOut, float *rollOut){//change to take arguments
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
  RotatePitchRoll(&imu.yaw,&zero,&setPointX,&setPointY,&pitchSetPoint,&rollSetPoint);
}

void GPSStable(){
  if (pitchSetPointTX == 0 && rollSetPointTX == 0){
    Loiter();
  }
  else{
    RotatePitchRoll(&imu.yaw,&headingFreeInitial,&pitchSetPointTX,&rollSetPointTX,&pitchSetPoint,&rollSetPoint);
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
    //Serial<<"wp hold\r\n";
    //AltitudeHold();
    GPSStable();
    if (startFlight == true && inputWayPointNumber > 0){
      startFlight = false;
      wayPointState = WP_TRAVEL;
      currentWayPointNumber = 0;
      targetAltitude = (wayPoints[0].coord.alt * 0.001) - altitudeDifference;
      //gps.Heading(&d.v.lattitude,&d.v.longitude,&wayPoints[0].coord.lat,&wayPoints[0].coord.lon,&currentBearing);
      //change to the correct dist heading function call
      //previousBearing = currentBearing;
      rollSetPoint = 0;
    }
    break;
  case WP_TRAVEL:
    //Serial<<"wp travel\r\n";
    //AltitudeHold();
    if (GPSPID == true){
      GPSPID = false;
      //gps.Distance(&d.v.lattitude,&d.v.longitude,&wayPoints[currentWayPointNumber].v.lat,&wayPoints[currentWayPointNumber].v.lon,&distToWayPoint);
      distToWayPoint *= -1;
      speed2D_MPS = d.v.groundSpeed * 0.001;
      //previousBearing = currentBearing;
      //gps.Heading(&d.v.lattitude,&d.v.longitude,&wayPoints[currentWayPointNumber].v.lat,&wayPoints[currentWayPointNumber].v.lon,&currentBearing);
      //yawSetPoint = currentBearing;
      //GPSdt = (millis() - GPSTimer) * 0.001;
      //GPSTimer = millis();
      WayPointPosition.calculate();
      WayPointRate.calculate();
      pitchSetPoint *= -1;
      if (distToWayPoint >= -1 /*|| fabs(previousBearing - currentBearing) > 90*/){
        currentWayPointNumber++;
        targetAltitude = (wayPoints[currentWayPointNumber].coord.alt * 0.001) - altitudeDifference;
      }
      /*if (rcCommands.values.aileron > 1650){
       currentWayPointNumber++;
       }*/

    }
    if (currentWayPointNumber == inputWayPointNumber){
      wayPointState = WP_END;
      targetAltitude = imu.ZEst;
      latTarget = gps.data.vars.lat;
      lonTarget = gps.data.vars.lon;
      SendEndOfWPCheck();
      endOfWPTimer = millis();
    }
    break;
  case WP_END:
    //Serial<<"wp end\r\n";s
    GPSStable();
    //AltitudeHold();
    if (endOfWPCheck == true){
      wayPointState = WP_HOLD;
      break;
    }
    if (RTBFlag == true){
      wayPointState = WP_RTB;
      //remove extra altitude for autoland
      targetAltitude = (homeBase.coord.alt * 0.001) - altitudeDifference;
      break;
    }
    if (millis() - endOfWPTimer > 5000){
      wayPointState = WP_FAIL_RTB;
      RTBFailSafe = true;
      //remove more for autoland
      targetAltitude = (homeBase.coord.alt * 0.001) - altitudeDifference;
    }
    break;
  case WP_RTB:
    ///Serial<<"wp rtb\r\n";
    //AltitudeHold();
    if (GPSPID == true){
      GPSPID = false;
      //gps.Distance(&d.v.lattitude,&d.v.longitude,&homeBase.v.lat,&homeBase.v.lon,&distToWayPoint);
      distToWayPoint *= -1;
      speed2D_MPS = d.v.groundSpeed * 0.001;
      //gps.Heading(&d.v.lattitude,&d.v.longitude,&homeBase.v.lat,&homeBase.v.lon,&currentBearing);
      //yawSetPoint = currentBearing;
      //GPSdt = (millis() - GPSTimer) * 0.001;
      //GPSTimer = millis();
      WayPointPosition.calculate();
      WayPointRate.calculate();
      pitchSetPoint *= -1;
      if (distToWayPoint >= -1){
        targetAltitude = imu.ZEst;
        latTarget = gps.data.vars.lat;
        lonTarget = gps.data.vars.lon;
        wayPointState = WP_HOLD;
      }
    }

    break;
  case WP_FAIL_RTB:
    //Serial<<"wp rtb fs\r\n";
    //AltitudeHold();
    if (GPSPID == true){
      GPSPID = false;
      //gps.Distance(&d.v.lattitude,&d.v.longitude,&homeBase.v.lat,&homeBase.v.lon,&distToWayPoint);
      distToWayPoint *= -1;
      speed2D_MPS = d.v.groundSpeed * 0.001;
      //gps.Heading(&d.v.lattitude,&d.v.longitude,&homeBase.v.lat,&homeBase.v.lon,&currentBearing);
      //yawSetPoint = currentBearing;
      //GPSdt = (millis() - GPSTimer) * 0.001;
      //GPSTimer = millis();
      WayPointPosition.calculate();
      WayPointRate.calculate();
      pitchSetPoint *= -1;
      if (distToWayPoint >= -1){
        d.v.flightMode = STABLE;
        enterState = true;

      }
    }
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














