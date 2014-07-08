
//move the rudder to the right to start calibration
void Arm(){
  //arming procedure
  digitalWrite(RED,LOW);
  digitalWrite(YELLOW,LOW);
  digitalWrite(GREEN,LOW);
  digitalWrite(13,LOW);
  newRC = false;
  generalPurposeTimer = millis();
  while (newRC == false){
  }
  newRC = false;
  digitalWrite(RED,LOW);
  digitalWrite(YELLOW,LOW);
  digitalWrite(GREEN,LOW);
  digitalWrite(13,HIGH);
  while (RCValue[RUDD] < 1750){
    if (newRC == true){
      ProcessChannels();
      newRC = false;
    }
  } 
  digitalWrite(RED,HIGH);
  digitalWrite(YELLOW,LOW);
  digitalWrite(GREEN,HIGH);
  digitalWrite(13,LOW);

}

void LoiterSM(){

  switch(ZLoiterState){
  case LOITERING:
    AltHoldPosition.calculate();
    AltHoldVelocity.calculate();
    if (abs(RCValue[THRO] - 1550) > 150 && throttleCheckFlag == false){
      ZLoiterState = RCINPUT;
    }
    if (RCValue[THRO] < 1100 && motorState == FLIGHT){
      ZLoiterState = LAND;
      motorState = LANDING;
      velSetPointZ.val = LAND_VEL;
    }
    break;
  case RCINPUT:
    if (throttleCheckFlag == true){
      ZLoiterState = LOITERING;
      break;
    }
    rcDifference = RCValue[THRO] - 1550;
    if (abs(rcDifference) < 150){
      ZLoiterState = LOITERING;
      zTarget = imu.ZEst;
      if (zTarget.val < 1.0){
        zTarget.val = 1.0;
      } 
      if (zTarget.val > 30.0){
        zTarget.val = 30.0;
      }
      AltHoldPosition.calculate();
      AltHoldVelocity.calculate();
      break;
    }
    velSetPointZ.val = rcDifference * 0.0034;
    if (velSetPointZ.val > MAX_Z_RATE){
      velSetPointZ.val = MAX_Z_RATE;
    }
    if (velSetPointZ.val < MIN_Z_RATE){
      velSetPointZ.val = MIN_Z_RATE;
    }

    AltHoldVelocity.calculate();
    if (RCValue[THRO] < 1100 && motorState == FLIGHT){
      ZLoiterState = LAND;
      motorState = LANDING;
      velSetPointZ.val = LAND_VEL;
    }
    break;

  case LAND:
    AltHoldVelocity.calculate();
    if (RCValue[THRO] > 1200 && motorState == LANDING){
      ZLoiterState = LOITERING;
      motorState = FLIGHT;
      throttleCheckFlag = true;

    }
    break;
  }


  if (gpsFailSafe == false && drFlag == false){
    switch(XYLoiterState){
    case LOITERING:
      LoiterCalculations();
      RotatePitchRoll(&imu.yaw.val,&zero,&tiltAngleX.val,&tiltAngleY.val,&pitchSetPoint.val,&rollSetPoint.val);
      if (fabs(rollSetPointTX.val) > 0.5 || fabs(pitchSetPointTX.val) > 0.5){
        XYLoiterState = RCINPUT;
      }
      break;
    case RCINPUT:
      RotatePitchRoll(&imu.yaw.val,&controlBearing,&pitchSetPointTX.val,&rollSetPointTX.val,&pitchSetPoint.val,&rollSetPoint.val);
      if (fabs(rollSetPointTX.val) < 0.5 && fabs(pitchSetPointTX.val) < 0.5){
        XYLoiterState = WAIT;
        waitTimer = millis();
      }
      break;
    case WAIT:
      if (fabs(rollSetPointTX.val) > 0.5 || fabs(pitchSetPointTX.val) > 0.5){
        XYLoiterState = RCINPUT;
        break;
      }
      if (millis() - waitTimer > 1000){
        XYLoiterState = LOITERING;
        xTarget.val = imu.XEst.val;
        yTarget.val = imu.YEst.val;
      }
      break;

    }  
  }
  else{
    if (flightMode = L2){
      controlBearing = initialYaw;
    }
    RotatePitchRoll(&imu.yaw.val,&controlBearing,&pitchSetPointTX.val,&rollSetPointTX.val,&pitchSetPoint.val,&rollSetPoint.val);
  }
}



void RotatePitchRoll(float *currentBearing, float *initialBearing, float *pitchIn, float *rollIn, float *pitchOut, float *rollOut){//change to take arguments
  float headingFreeDifference;
  float sinHeadingFreeDiff;
  float cosHeadingFreeDiff;
  headingFreeDifference = *currentBearing - *initialBearing;
  sinHeadingFreeDiff = sin(ToRad(headingFreeDifference));
  cosHeadingFreeDiff = cos(ToRad(headingFreeDifference));
  *rollOut = *rollIn * cosHeadingFreeDiff + *pitchIn * sinHeadingFreeDiff;
  *pitchOut = -1.0 * *rollIn * sinHeadingFreeDiff + *pitchIn * cosHeadingFreeDiff;

}

void WayPointControl(){


}

void HeadingHold(){
  switch (HHState){
  case HH_ON:
    calcYaw = true;
    if (abs(yawInput) > 1){
      HHState = HH_OFF;
    }
    break;
  case HH_OFF:
    calcYaw = false;
    rateSetPointZ.val = yawInput;
    if (abs(yawInput) < 1){
      yawSetPoint = imu.yaw;
      HHState = HH_ON;
    }
    break;
  default:
    HHState = HH_OFF;

    break;
  }  
}




















