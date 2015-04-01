
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


  if (gsCTRL == false){
    loitThro = RCValue[THRO];
  }
  else{
    loitThro = GSRCValue[THRO];
  }
  switch(ZLoiterState){
  case LOITERING:
    AltHoldPosition.calculate();
    AltHoldVelocity.calculate();
    if (abs(loitThro - 1550) > 200 && throttleCheckFlag == false){
      ZLoiterState = RCINPUT;
    }
    if (loitThro < 1050 && motorState == FLIGHT){
      ZLoiterState = LAND;
      motorState = LANDING;
      //zTarget.val = -100.0;
      velSetPointZ.val = LAND_VEL;
    }
    break;
  case RCINPUT:
    if (throttleCheckFlag == true){
      ZLoiterState = LOITERING;
      break;
    }
    rcDifference = loitThro - 1550;
    if (abs(rcDifference) < 200){
      ZLoiterState = LOITERING;
      zTarget = imu.ZEstUp;
      if (zTarget.val <= FLOOR){
        zTarget.val = FLOOR;
      } 
      if (zTarget.val >= CEILING){
        zTarget.val = CEILING;
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
    if (loitThro < 1050 && motorState == FLIGHT){
      ZLoiterState = LAND;
      motorState = LANDING;
      //zTarget.val  = -100.0;
      velSetPointZ.val = LAND_VEL;
      break;
    }


    if (imu.ZEstUp.val >= CEILING && velSetPointZ.val > 0){
      zTarget.val = CEILING;
      AltHoldPosition.calculate();
      AltHoldVelocity.calculate();
      break;
    }
    if (imu.ZEstUp.val <= FLOOR && velSetPointZ.val < 0){
      zTarget.val = FLOOR;
      AltHoldPosition.calculate();
      AltHoldVelocity.calculate();
      break;
    }

    AltHoldVelocity.calculate();
    break;


  case LAND:
    /*AltHoldPosition.calculate();
     if (velSetPointZ.val < LAND_VEL){
     velSetPointZ.val = LAND_VEL;
     }*/
    AltHoldVelocity.calculate();

    if (loitThro > 1200 && motorState == LANDING){
      ZLoiterState = LOITERING;
      motorState = FLIGHT;
      zTarget = imu.ZEstUp;
      if (zTarget.val <= FLOOR){
        zTarget.val = FLOOR;
      } 
      if (zTarget.val >= CEILING){
        zTarget.val = CEILING;
      }
      throttleCheckFlag = true;

    }
    break;
  }


  //if (gpsFailSafe == false && drFlag == false){
  if (gpsFailSafe == false && GPSDetected == true){
    switch(XYLoiterState){
    case LOITERING:
      LoiterCalculations();
      RotatePitchRoll(&imu.yaw.val,&zero,&tiltAngleX.val,&tiltAngleY.val,&pitchSetPoint.val,&rollSetPoint.val);
      //to do switch from fabs to look for zero
      if (fabs(rollSetPointTX.val) > 0.5 || fabs(pitchSetPointTX.val) > 0.5){
        XYLoiterState = RCINPUT;
      }
      break;
    case RCINPUT:
      RotatePitchRoll(&imu.yaw.val,&controlBearing.val,&pitchSetPointTX.val,&rollSetPointTX.val,&pitchSetPoint.val,&rollSetPoint.val);
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
    if (flightMode == L2){//check
      controlBearing.val = initialYaw.val;
    }
    RotatePitchRoll(&imu.yaw.val,&controlBearing.val,&pitchSetPointTX.val,&rollSetPointTX.val,&pitchSetPoint.val,&rollSetPoint.val);
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
  if (imu.magDetected == true){
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
  else{
    rateSetPointZ.val = yawInput;
    calcYaw = false;
  }
}























