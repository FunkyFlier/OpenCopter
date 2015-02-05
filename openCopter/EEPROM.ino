#include "AUXMATH.h"

void AssignPointerArray(){
 floatPointerArray[GYRO_X_DEG] = &degreeGyroX;
  floatPointerArray[GYRO_Y_DEG] = &degreeGyroY;
  floatPointerArray[GYRO_Z_DEG] = &degreeGyroZ;
  floatPointerArray[ACC_X_FILT] = &filtAccX;
  floatPointerArray[ACC_Y_FILT] = &filtAccY;
  floatPointerArray[ACC_Z_FILT] = &filtAccZ;
  floatPointerArray[MAG_X_CALIB] = &calibMagX;
  floatPointerArray[MAG_Y_CALIB] = &calibMagY;
  floatPointerArray[MAG_Z_CALIB] = &calibMagZ;
  floatPointerArray[RAW_X] = &gpsX;
  floatPointerArray[RAW_Y] = &gpsY;
  floatPointerArray[RAW_Z] = &zMeas;
  floatPointerArray[PITCH_] = &imu.pitch;
  floatPointerArray[ROLL_] = &imu.roll;
  floatPointerArray[YAW_] = &imu.yaw;
  floatPointerArray[QUAT_0] = &imu.q0;
  floatPointerArray[QUAT_1] = &imu.q1;
  floatPointerArray[QUAT_2] = &imu.q2;
  floatPointerArray[QUAT_3] = &imu.q3;
  floatPointerArray[X_EST] = &imu.XEst;
  floatPointerArray[Y_EST] = &imu.YEst;
  floatPointerArray[Z_EST] = &imu.ZEstUp;
  floatPointerArray[VEL_X] = &imu.velX;
  floatPointerArray[VEL_Y] = &imu.velY;
  floatPointerArray[VEL_Z] = &imu.velZUp;
  floatPointerArray[KP_PITCH_RATE_] = &kp_pitch_rate;
  floatPointerArray[KI_PITCH_RATE_] = &ki_pitch_rate;
  floatPointerArray[KD_PITCH_RATE_] = &kd_pitch_rate;
  floatPointerArray[FC_PITCH_RATE_] = &fc_pitch_rate;
  floatPointerArray[KP_ROLL_RATE_] = &kp_roll_rate;
  floatPointerArray[KI_ROLL_RATE_] = &ki_roll_rate;
  floatPointerArray[KD_ROLL_RATE_] = &kd_roll_rate;
  floatPointerArray[FC_ROLL_RATE_] = &fc_roll_rate;
  floatPointerArray[KP_YAW_RATE_] = &kp_yaw_rate;
  floatPointerArray[KI_YAW_RATE_] = &ki_yaw_rate;
  floatPointerArray[KD_YAW_RATE_] = &kd_yaw_rate;
  floatPointerArray[FC_YAW_RATE_] = &fc_yaw_rate;
  floatPointerArray[KP_PITCH_ATT_] = &kp_pitch_attitude;
  floatPointerArray[KI_PITCH_ATT_] = &ki_pitch_attitude;
  floatPointerArray[KD_PITCH_ATT_] = &kd_pitch_attitude;
  floatPointerArray[FC_PITCH_ATT_] = &fc_pitch_attitude;
  floatPointerArray[KP_ROLL_ATT_] = &kp_roll_attitude;
  floatPointerArray[KI_ROLL_ATT_] = &ki_roll_attitude;
  floatPointerArray[KD_ROLL_ATT_] = &kd_roll_attitude;
  floatPointerArray[FC_ROLL_ATT_] = &fc_roll_attitude;

  floatPointerArray[KP_YAW_ATT_] = &kp_yaw_attitude; 
  floatPointerArray[KI_YAW_ATT_] = &ki_yaw_attitude;
  floatPointerArray[KD_YAW_ATT_] = &kd_yaw_attitude;
  floatPointerArray[FC_YAW_ATT_] = &fc_yaw_attitude;


  floatPointerArray[KP_ALT_POS_] = &kp_altitude_position;
  floatPointerArray[KI_ALT_POS_] = &ki_altitude_position;
  floatPointerArray[KD_ATL_POS_] = &kd_altitude_position;
  floatPointerArray[FC_ALT_POS_] = &fc_altitude_position;
  floatPointerArray[KP_ALT_VEL_] = &kp_altitude_velocity;
  floatPointerArray[KI_ALT_VEL_] = &ki_altitude_velocity;
  floatPointerArray[KD_ALT_VEL_] = &kd_altitude_velocity;
  floatPointerArray[FC_ALT_VEL_] = &fc_altitude_velocity;
  floatPointerArray[MUL_ALT_VEL_] = &mul_altitude_velocity;

  floatPointerArray[KP_LOIT_X_POS_] = &kp_loiter_pos_x;
  floatPointerArray[KI_LOIT_X_POS_] = &ki_loiter_pos_x;
  floatPointerArray[KD_LOIT_X_POS_] = &kd_loiter_pos_x;
  floatPointerArray[FC_LOIT_X_POS_] = &fc_loiter_pos_x;

  floatPointerArray[KP_LOIT_X_VEL_] = &kp_loiter_velocity_x;
  floatPointerArray[KI_LOIT_X_VEL_] = &ki_loiter_velocity_x;
  floatPointerArray[KD_LOIT_X_VEL_] = &kd_loiter_velocity_x;
  floatPointerArray[FC_LOIT_X_VEL_] = &fc_loiter_velocity_x;

  floatPointerArray[KP_LOIT_Y_POS_] = &kp_loiter_pos_y;
  floatPointerArray[KI_LOIT_Y_POS_] = &ki_loiter_pos_y;
  floatPointerArray[KD_LOIT_Y_POS_] = &kd_loiter_pos_y;
  floatPointerArray[FC_LOIT_Y_POS_] = &fc_loiter_pos_y;
  floatPointerArray[KP_LOIT_Y_VEL_] = &kp_loiter_velocity_y;
  floatPointerArray[KI_LOIT_Y_VEL_] = &ki_loiter_velocity_y;
  floatPointerArray[KD_LOIT_Y_VEL_] = &kd_loiter_velocity_y;
  floatPointerArray[FC_LOIT_Y_VEL_] = &fc_loiter_velocity_y;

  floatPointerArray[KP_WP_POS_] = &kp_waypoint_position;
  floatPointerArray[KI_WP_POS_] = &ki_waypoint_position;
  floatPointerArray[KD_WP_POS_] = &kd_waypoint_position;
  floatPointerArray[FC_WP_POS_] = &fc_waypoint_position;

  floatPointerArray[KP_WP_VEL_] = &kp_waypoint_velocity;
  floatPointerArray[KI_WP_VEL_] = &ki_waypoint_velocity;
  floatPointerArray[KD_WP_VEL_] = &kd_waypoint_velocity;
  floatPointerArray[FC_WP_VEL_] = &fc_waypoint_velocity;

  floatPointerArray[KP_CT_] = &kp_cross_track;
  floatPointerArray[KI_CT_] = &ki_cross_track;
  floatPointerArray[KD_CT_] = &kd_cross_track;
  floatPointerArray[FC_CT_] = &fc_cross_track;

  floatPointerArray[MAG_DEC_] = &imu.declination;

  floatPointerArray[RATE_SP_X] = &rateSetPointX;
  floatPointerArray[RATE_SP_Y] = &rateSetPointY;
  floatPointerArray[RATE_SP_Z] = &rateSetPointZ;
  floatPointerArray[ADJ_X] = &imu.inertialXBiased;
  floatPointerArray[ADJ_Y] = &imu.inertialYBiased;
  floatPointerArray[ADJ_Z] = &imu.inertialZBiased;
  floatPointerArray[PITCH_SP] = &pitchSetPoint;
  floatPointerArray[ROLL_SP] = &rollSetPoint;
  floatPointerArray[YAW_SP] = &yawSetPoint;
  floatPointerArray[X_TARG] = &xTarget;
  floatPointerArray[Y_TARG] = &yTarget;
  floatPointerArray[Z_TARG] = &zTarget;
  floatPointerArray[VEL_SP_X] = &velSetPointX;
  floatPointerArray[VEL_SP_Y] = &velSetPointY;
  floatPointerArray[VEL_SP_Z] = &velSetPointZ;
  floatPointerArray[TILT_X] = &tiltAngleX;
  floatPointerArray[TILT_Y] = &tiltAngleY;
  floatPointerArray[THRO_ADJ] = &throttleAdjustment;
  floatPointerArray[PITCH_SP_TX] = &pitchSetPointTX;
  floatPointerArray[ROLL_SP_TX] = &rollSetPointTX;
  floatPointerArray[DIST_TO_WP] = &tempOutput;

  floatPointerArray[TARGET_VEL_WP] = &velZMeas;
  floatPointerArray[POS_ERR] = &imu.lagEstForDebugPos;
  floatPointerArray[ACC_CIR] = &imu.lagEstForDebugVel;
  floatPointerArray[DR_VEL_X] = &velN;
  floatPointerArray[DR_VEL_Y] = &velE;
  floatPointerArray[DR_POS_X] = &baroVel;
  floatPointerArray[DR_POS_Y] = &drPosY;
  floatPointerArray[MOTOR_CMD_1] = &imu.accelBiasX;
  floatPointerArray[MOTOR_CMD_2] = &imu.accelBiasY;
  floatPointerArray[MOTOR_CMD_3] = &imu.accelBiasZ;
  floatPointerArray[MOTOR_CMD_4] = &motorCommand4;
  floatPointerArray[PITCH_OFF] = &imu.pitchOffset;
  floatPointerArray[ROLL_OFF] = &imu.rollOffset;

  floatPointerArray[INERTIAL_X] = &imu.inertialX;
  floatPointerArray[INERTIAL_Y] = &imu.inertialY;
  floatPointerArray[INERTIAL_Z] = &imu.inertialZ;


  int16PointerArray[GYRO_X] = &gyroX;
  int16PointerArray[GYRO_Y] = &gyroY;
  int16PointerArray[GYRO_Z] = &gyroZ;
  int16PointerArray[ACC_X] = &accX;
  int16PointerArray[ACC_Y] = &accY;
  int16PointerArray[ACC_Z] = &accZ;
  int16PointerArray[MAG_X] = &magX;
  int16PointerArray[MAG_Y] = &magY;
  int16PointerArray[MAG_Z] = &magZ;
  int16PointerArray[NUM_SATS] = &numSats;


  int32PointerArray[PRESSURE_] = &lattitude;
  int32PointerArray[HB_LAT] = &homeBase.lat;
  int32PointerArray[HB_LON] = &homeBase.lon;
  int32PointerArray[HB_ALT] = &homeBase.alt;
  int32PointerArray[H_DOP] = &hDop;
  int32PointerArray[LAT_] = &lattitude;
  int32PointerArray[LON_] = &longitude;

  bytePointerArray[FLIGHT_MODE] = &flightMode;
  bytePointerArray[RTB_STATE] = &RTBState;
  bytePointerArray[Z_LOIT] = &ZLoiterState;
  bytePointerArray[XY_LOIT] = &XYLoiterState;
  bytePointerArray[GPS_FS] = &gpsFailSafe;
  bytePointerArray[DR_FLAG] = &drFlag;
  bytePointerArray[MOTOR_STATE] = &imu.feedBack;



}
/*
void DEBUG_DUMP(){
 Port0<< _FLOAT(accXScalePos,7) <<"\r\n";
 Port0<< _FLOAT(accYScalePos,7) <<"\r\n";
 Port0<< _FLOAT(accZScalePos,7) <<"\r\n";
 Port0<< _FLOAT(accXScaleNeg,7) <<"\r\n";
 Port0<< _FLOAT(accYScaleNeg,7) <<"\r\n";
 Port0<< _FLOAT(accZScaleNeg,7) <<"\r\n";
 Port0<< _FLOAT(magOffSetX,7) <<"\r\n";
 Port0<< _FLOAT(magOffSetY,7) <<"\r\n";
 Port0<< _FLOAT(magOffSetZ,7) <<"\r\n";
 Port0<< _FLOAT(magWInv00,7) <<"\r\n";
 Port0<< _FLOAT(magWInv01,7) <<"\r\n";
 Port0<< _FLOAT(magWInv02,7) <<"\r\n";
 Port0<< _FLOAT(magWInv10,7) <<"\r\n";
 Port0<< _FLOAT(magWInv11,7) <<"\r\n";
 Port0<< _FLOAT(magWInv12,7) <<"\r\n";
 Port0<< _FLOAT(magWInv20,7) <<"\r\n";
 Port0<< _FLOAT(magWInv21,7) <<"\r\n";
 Port0<< _FLOAT(magWInv22,7) <<"\r\n";
 Port0<< _FLOAT(imu.pitchOffset.val,7) <<"\r\n";
 Port0<< _FLOAT(imu.rollOffset.val,7) <<"\r\n";
 Port0<< _FLOAT(kp_pitch_rate.val,7) <<"\r\n";
 Port0<< _FLOAT(ki_pitch_rate.val,7) <<"\r\n";
 Port0<< _FLOAT(kd_pitch_rate.val,7) <<"\r\n";
 Port0<< _FLOAT(fc_pitch_rate.val,7) <<"\r\n";
 Port0<< _FLOAT(kp_roll_rate.val,7) <<"\r\n";
 Port0<< _FLOAT(ki_roll_rate.val,7) <<"\r\n";
 Port0<< _FLOAT(kd_roll_rate.val,7) <<"\r\n";
 Port0<< _FLOAT(fc_roll_rate.val,7) <<"\r\n";
 Port0<< _FLOAT(kp_yaw_rate.val,7) <<"\r\n";
 Port0<< _FLOAT(ki_yaw_rate.val,7) <<"\r\n";
 Port0<< _FLOAT(kd_yaw_rate.val,7) <<"\r\n";
 Port0<< _FLOAT(fc_yaw_rate.val,7) <<"\r\n";
 Port0<< _FLOAT(kp_pitch_attitude.val,7) <<"\r\n";
 Port0<< _FLOAT(ki_pitch_attitude.val,7) <<"\r\n";
 Port0<< _FLOAT(kd_pitch_attitude.val,7) <<"\r\n";
 Port0<< _FLOAT(fc_pitch_attitude.val,7) <<"\r\n";
 Port0<< _FLOAT(kp_roll_attitude.val,7) <<"\r\n";
 Port0<< _FLOAT(ki_roll_attitude.val,7) <<"\r\n";
 Port0<< _FLOAT(kd_roll_attitude.val,7) <<"\r\n";
 Port0<< _FLOAT(fc_roll_attitude.val,7) <<"\r\n";
 Port0<< _FLOAT(kp_yaw_attitude.val,7) <<"\r\n";
 Port0<< _FLOAT(ki_yaw_attitude.val,7) <<"\r\n";
 Port0<< _FLOAT(kd_yaw_attitude.val,7) <<"\r\n";
 Port0<< _FLOAT(fc_yaw_attitude.val,7) <<"\r\n";
 Port0<< _FLOAT(kp_altitude_position.val,7) <<"\r\n";
 Port0<< _FLOAT(ki_altitude_position.val,7) <<"\r\n";
 Port0<< _FLOAT(kd_altitude_position.val,7) <<"\r\n";
 Port0<< _FLOAT(fc_altitude_position.val,7) <<"\r\n";
 Port0<< _FLOAT(kp_altitude_velocity.val,7) <<"\r\n";
 Port0<< _FLOAT(ki_altitude_velocity.val,7) <<"\r\n";
 Port0<< _FLOAT(kd_altitude_velocity.val,7) <<"\r\n";
 Port0<< _FLOAT(fc_altitude_velocity.val,7) <<"\r\n";
 Port0<< _FLOAT(mul_altitude_velocity.val,7) <<"\r\n";
 Port0<< _FLOAT(kp_loiter_pos_x.val,7) <<"\r\n";
 Port0<< _FLOAT(ki_loiter_pos_x.val,7) <<"\r\n";
 Port0<< _FLOAT(kd_loiter_pos_x.val,7) <<"\r\n";
 Port0<< _FLOAT(fc_loiter_pos_x.val,7) <<"\r\n";
 Port0<< _FLOAT(kp_loiter_velocity_x.val,7) <<"\r\n";
 Port0<< _FLOAT(ki_loiter_velocity_x.val,7) <<"\r\n";
 Port0<< _FLOAT(kd_loiter_velocity_x.val,7) <<"\r\n";
 Port0<< _FLOAT(fc_loiter_velocity_x.val,7) <<"\r\n";
 Port0<< _FLOAT(kp_loiter_pos_y.val,7) <<"\r\n";
 Port0<< _FLOAT(ki_loiter_pos_y.val,7) <<"\r\n";
 Port0<< _FLOAT(kd_loiter_pos_y.val,7) <<"\r\n";
 Port0<< _FLOAT(fc_loiter_pos_y.val,7) <<"\r\n";
 Port0<< _FLOAT(kp_loiter_velocity_y.val,7) <<"\r\n";
 Port0<< _FLOAT(ki_loiter_velocity_y.val,7) <<"\r\n";
 Port0<< _FLOAT(kd_loiter_velocity_y.val,7) <<"\r\n";
 Port0<< _FLOAT(fc_loiter_velocity_y.val,7) <<"\r\n";
 Port0<< _FLOAT(kp_waypoint_position.val,7) <<"\r\n";
 Port0<< _FLOAT(ki_waypoint_position.val,7) <<"\r\n";
 Port0<< _FLOAT(kd_waypoint_position.val,7) <<"\r\n";
 Port0<< _FLOAT(fc_waypoint_position.val,7) <<"\r\n";
 Port0<< _FLOAT(kp_waypoint_velocity.val,7) <<"\r\n";
 Port0<< _FLOAT(ki_waypoint_velocity.val,7) <<"\r\n";
 Port0<< _FLOAT(kd_waypoint_velocity.val,7) <<"\r\n";
 Port0<< _FLOAT(fc_waypoint_velocity.val,7) <<"\r\n";
 Port0<< _FLOAT(kp_cross_track.val,7) <<"\r\n";
 Port0<< _FLOAT(ki_cross_track.val,7) <<"\r\n";
 Port0<< _FLOAT(kd_cross_track.val,7) <<"\r\n";
 Port0<< _FLOAT(fc_cross_track.val,7) <<"\r\n";
 Port0<< _FLOAT(imu.declination.val,7) <<"\r\n";
 Port0<< minRCVal[THRO] <<"\r\n";
 Port0<< minRCVal[GEAR] <<"\r\n";
 Port0<< minRCVal[AUX1] <<"\r\n";
 Port0<< minRCVal[AUX2] <<"\r\n";
 Port0<< minRCVal[AUX3] <<"\r\n";
 Port0<< centerRCVal[AILE] <<"\r\n";
 Port0<< centerRCVal[ELEV] <<"\r\n";
 Port0<< centerRCVal[RUDD] <<"\r\n";
 Port0<< _FLOAT(RCScale[THRO],7) <<"\r\n";
 Port0<< _FLOAT(RCScale[AILE],7) <<"\r\n";
 Port0<< _FLOAT(RCScale[ELEV],7) <<"\r\n";
 Port0<< _FLOAT(RCScale[RUDD],7) <<"\r\n";
 Port0<< _FLOAT(RCScale[GEAR],7) <<"\r\n";
 Port0<< _FLOAT(RCScale[AUX1],7) <<"\r\n";
 Port0<< _FLOAT(RCScale[AUX2],7) <<"\r\n";
 Port0<< _FLOAT(RCScale[AUX3],7) <<"\r\n";
 
 
 }
*/
void ROMFlagsCheck(){
  uint16_t j;
  if (EEPROM.read(382) != 0xAA){
    imu.pitchOffset.val = 0;
    imu.rollOffset.val = 0;
    j = 0;
    for(uint16_t i = 73; i <=76; i++){
      EEPROM.write(i,imu.pitchOffset.buffer[j++]);
    }
    j = 0;
    for(uint16_t i = 77; i <=80; i++){
      EEPROM.write(i,imu.rollOffset.buffer[j++]);
    }
  }
  if (EEPROM.read(389) == 0xAA){
    //----
    j = 0;
    for(uint16_t i = 390; i < 394; i++){
      xSlopeAcc.buffer[j++] = EEPROM.read(i);
    }
    j = 0;
    for(uint16_t i = 394; i < 398; i++){
      ySlopeAcc.buffer[j++] = EEPROM.read(i);
    }
    j = 0;
    for(uint16_t i = 398; i < 402; i++){
      zSlopeAcc.buffer[j++] = EEPROM.read(i);
    }
    //----
    //----
    j = 0;
    for(uint16_t i = 402; i < 406; i++){
      xSlopeMag.buffer[j++] = EEPROM.read(i);
    }
    j = 0;
    for(uint16_t i = 406; i < 410; i++){
      ySlopeMag.buffer[j++] = EEPROM.read(i);
    }
    j = 0;
    for(uint16_t i = 410; i < 414; i++){
      zSlopeMag.buffer[j++] = EEPROM.read(i);
    }
    //----
    //----
    j = 0;
    for(uint16_t i = 414; i < 418; i++){
      xSlopeGyro.buffer[j++] = EEPROM.read(i);
    }
    j = 0;
    for(uint16_t i = 418; i < 422; i++){
      ySlopeGyro.buffer[j++] = EEPROM.read(i);
    }
    j = 0;
    for(uint16_t i = 422; i < 426; i++){
      zSlopeGyro.buffer[j++] = EEPROM.read(i);
    }

    j = 0;
    for(uint16_t i = 383; i < 385; i++){
      calibTempAcc.buffer[j++] = EEPROM.read(i);
    }

    j = 0;
    for(uint16_t i = 426; i < 428; i++){
      calibTempMag.buffer[j++] = EEPROM.read(i);
    }

  }
  else{
    xSlopeAcc.val = 0;
    ySlopeAcc.val = 0;
    zSlopeAcc.val = 0;

    xSlopeMag.val = 0;
    ySlopeMag.val = 0;
    zSlopeMag.val = 0;

    xSlopeGyro.val = 0;
    ySlopeGyro.val = 0;
    zSlopeGyro.val = 0;

    calibTempAcc.val = 0;
    calibTempMag.val = 0;

  }
  calibrationFlags = EEPROM.read(0);
  if ( ((calibrationFlags & (1<<RC_FLAG)) >> RC_FLAG) == 0x01 || ((calibrationFlags & (1<<ACC_FLAG)) >> ACC_FLAG) == 0x01 || ((calibrationFlags & (1<<MAG_FLAG)) >> MAG_FLAG) == 0x01 ){
    Port2.begin(115200);
    radioStream = &Port2;
    radioPrint = &Port2;
    HandShake();

    if (handShake == false){
      USBFlag = true;
      radioStream = &Port0;
      radioPrint = &Port0;
      HandShake();
    }
    if (calibrationMode == true){
      digitalWrite(RED,HIGH);
      digitalWrite(YELLOW,HIGH);
      digitalWrite(GREEN,HIGH);
      digitalWrite(13,LOW);
      return;
    }
    while(1){
      if ( ((calibrationFlags & (1<<RC_FLAG)) >> RC_FLAG) == 0x01 ){
        digitalWrite(RED,toggle);
      }
      if ( ((calibrationFlags & (1<<ACC_FLAG)) >> ACC_FLAG) == 0x01 ){
        digitalWrite(YELLOW,toggle);
      }
      if ( ((calibrationFlags & (1<<MAG_FLAG)) >> MAG_FLAG) == 0x01 ){
        digitalWrite(GREEN,toggle);
      }
      toggle = ~toggle;
      delay(300);
    }
  }

  if ( ((calibrationFlags & (1<<GAINS_FLAG)) >> GAINS_FLAG) == 0x01 ){
    SetDefaultGains();
  }
  LoadROM();


}
void SetDefaultGains(){

  uint16_t j;


  kp_pitch_rate.val = 0.891;
  ki_pitch_rate.val = 8.25;
  kd_pitch_rate.val = 0.064;
  fc_pitch_rate.val = 50.0;

  kp_roll_rate.val = 0.891;
  ki_roll_rate.val = 8.25;
  kd_roll_rate.val = 0.064;
  fc_roll_rate.val = 50.0;

  kp_yaw_rate.val = 6.0;
  ki_yaw_rate.val = 1.0;
  kd_yaw_rate.val = 0.01;
  fc_yaw_rate.val = 50.0;

  kp_pitch_attitude.val = 4.25;
  ki_pitch_attitude.val = 0;
  kd_pitch_attitude.val = 0.01;
  fc_pitch_attitude.val = 50.0;

  kp_roll_attitude.val = 4.25;
  ki_roll_attitude.val = 0;
  kd_roll_attitude.val = 0.01;
  fc_roll_attitude.val = 50;

  kp_yaw_attitude.val = 3.0;
  ki_yaw_attitude.val = 0;
  kd_yaw_attitude.val = 0.0;
  fc_yaw_attitude.val = 50.0;

  kp_altitude_position.val = 0.5;
  ki_altitude_position.val = 0;
  kd_altitude_position.val = 0.0;
  fc_altitude_position.val = 50;

  kp_altitude_velocity.val = 45;
  ki_altitude_velocity.val = 30;
  kd_altitude_velocity.val = 0.1;
  fc_altitude_velocity.val = 50;
  mul_altitude_velocity.val = 1.0;

  kp_loiter_pos_x.val = 0.5;
  ki_loiter_pos_x.val = 0;
  kd_loiter_pos_x.val = -0.0003;
  fc_loiter_pos_x.val = 50;

  kp_loiter_velocity_x.val = 4.9;
  ki_loiter_velocity_x.val = 0.1;
  kd_loiter_velocity_x.val = 0.0075;
  fc_loiter_velocity_x.val = 50;

  kp_loiter_pos_y.val = 0.5;
  ki_loiter_pos_y.val = 0;
  kd_loiter_pos_y.val = -0.0003;
  fc_loiter_pos_y.val = 50;

  kp_loiter_velocity_y.val = 4.9;
  ki_loiter_velocity_y.val = 0.1;
  kd_loiter_velocity_y.val = 0.0075;
  fc_loiter_velocity_y.val = 50;

  kp_waypoint_position.val = 0;
  ki_waypoint_position.val = 0;
  kd_waypoint_position.val = 0;
  fc_waypoint_position.val = 0;

  kp_waypoint_velocity.val = 0;
  ki_waypoint_velocity.val = 0;
  kd_waypoint_velocity.val = 0;
  fc_waypoint_velocity.val = 0;

  kp_cross_track.val = 0;
  ki_cross_track.val = 0;
  kd_cross_track.val = 0;
  fc_cross_track.val = 0;

  imu.declination.val = 3.66;
  j = 81;
  for(uint16_t i = KP_PITCH_RATE_; i <= MAG_DEC_; i++){
    EEPROM.write(j++,(*floatPointerArray[i]).buffer[0]); 
    EEPROM.write(j++,(*floatPointerArray[i]).buffer[1]); 
    EEPROM.write(j++,(*floatPointerArray[i]).buffer[2]); 
    EEPROM.write(j++,(*floatPointerArray[i]).buffer[3]); 
  }


}


void LoadROM(){
  uint16_t j;


  int16_u outShort;
  j = 0;
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

  outFloatIndex = 0;
  for (uint16_t i = 1; i <= 24; i++){//load acc values
    outFloat.buffer[outFloatIndex] = EEPROM.read(i);
    outFloatIndex++;
    switch (i){
    case 4:
      accXScale = outFloat.val;
      outFloatIndex = 0;
      break;
    case 8:
      accYScale = outFloat.val;
      outFloatIndex = 0;
      break;
    case 12:
      accZScale = outFloat.val;
      outFloatIndex = 0;
      break;  
    case 16:
      accXOffset = outFloat.val;
      outFloatIndex = 0;
      break;  
    case 20:
      accYOffset = outFloat.val;
      outFloatIndex = 0;
      break;  
    case 24:
      accZOffset = outFloat.val;
      outFloatIndex = 0;
      break;  
    default:
      break;
    }
  }

  outFloatIndex = 0;
  for (uint16_t i = 25; i <= 72; i++){//load the compass values
    outFloat.buffer[outFloatIndex] = EEPROM.read(i);
    outFloatIndex++;
    switch (i){
    case 28:
      magOffSetX = outFloat.val; 
      outFloatIndex = 0;
      break;
    case 32:
      magOffSetY = outFloat.val;
      outFloatIndex = 0;
      break;
    case 36:
      magOffSetZ = outFloat.val;
      outFloatIndex = 0;
      break;  
    case 40:
      magWInv00 = outFloat.val;
      outFloatIndex = 0;
      break;  
    case 44:
      magWInv01 = outFloat.val;
      outFloatIndex = 0;
      break;  
    case 48:
      magWInv02 = outFloat.val;
      outFloatIndex = 0;
      break;  
    case 52:
      magWInv10 = outFloat.val;
      outFloatIndex = 0;
      break;  
    case 56:
      magWInv11 = outFloat.val;
      outFloatIndex = 0;
      break;  
    case 60:
      magWInv12 = outFloat.val;
      outFloatIndex = 0;
      break;  
    case 64:
      magWInv20 = outFloat.val;
      outFloatIndex = 0;
      break;  
    case 68:
      magWInv21 = outFloat.val;
      outFloatIndex = 0;
      break;  
    case 72:
      magWInv22 = outFloat.val;
      outFloatIndex = 0;
      break;  
    default:
      break;
    }
  }
  j = 81;
  for (uint16_t i = KP_PITCH_RATE_; i <= FC_CT_; i++){//gains
    (*floatPointerArray[i]).buffer[0] = EEPROM.read(j++); 
    (*floatPointerArray[i]).buffer[1] = EEPROM.read(j++); 
    (*floatPointerArray[i]).buffer[2] = EEPROM.read(j++); 
    (*floatPointerArray[i]).buffer[3] = EEPROM.read(j++); 
  }
  j = 73;
  for (uint16_t i = PITCH_OFF; i <= ROLL_OFF; i++){//pitch and roll offsets
    (*floatPointerArray[i]).buffer[0] = EEPROM.read(j++); 
    (*floatPointerArray[i]).buffer[1] = EEPROM.read(j++); 
    (*floatPointerArray[i]).buffer[2] = EEPROM.read(j++); 
    (*floatPointerArray[i]).buffer[3] = EEPROM.read(j++); 
  }
  j = 325;
  (*floatPointerArray[MAG_DEC_]).buffer[0] = EEPROM.read(j++); 
  (*floatPointerArray[MAG_DEC_]).buffer[1] = EEPROM.read(j++); 
  (*floatPointerArray[MAG_DEC_]).buffer[2] = EEPROM.read(j++); 
  (*floatPointerArray[MAG_DEC_]).buffer[3] = EEPROM.read(j++); 


  //----
  j = 0;
  for(uint16_t i = 385; i < 387; i++){
    xAccOffset.buffer[j++] = EEPROM.read(i);
  }
  j = 0;
  for(uint16_t i = 387; i < 389; i++){
    yAccOffset.buffer[j++] = EEPROM.read(i);
  }


}









































