#include "AUXMATH.h"

void AssignPointerArray() {
  floatPointerArray[GYRO_X_DEG] = &degreeGyroX;
  floatPointerArray[GYRO_Y_DEG] = &degreeGyroY;
  floatPointerArray[GYRO_Z_DEG] = &degreeGyroZ;

  floatPointerArray[ACC_X_FILT] = &filtAccX;
  floatPointerArray[ACC_Y_FILT] = &filtAccY;
  floatPointerArray[ACC_Z_FILT] = &filtAccZ;

  floatPointerArray[ACC_X_SC] = &scaledAccX;
  floatPointerArray[ACC_Y_SC] = &scaledAccY;
  floatPointerArray[ACC_Z_SC] = &scaledAccZ;

  floatPointerArray[MAG_X_CALIB] = &calibMagX;
  floatPointerArray[MAG_Y_CALIB] = &calibMagY;
  floatPointerArray[MAG_Z_CALIB] = &calibMagZ;

  floatPointerArray[DIST_TO_CRAFT] = &distToCraft;
  floatPointerArray[HEAD_TO_CRAFT] = &headingToCraft;

  floatPointerArray[RAW_X] = &gpsX;
  floatPointerArray[RAW_Y] = &gpsY;
  floatPointerArray[RAW_Z] = &baroZ;

  floatPointerArray[VEL_N] = &velN;
  floatPointerArray[VEL_E] = &velE;
  floatPointerArray[VEL_D] = &velD;

  floatPointerArray[VEL_BARO] = &baroVel;

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

  floatPointerArray[ACC_BIAS_X] = &imu.accelBiasX;
  floatPointerArray[ACC_BIAS_Y] = &imu.accelBiasY;
  floatPointerArray[ACC_BIAS_Z] = &imu.accelBiasZ;

  floatPointerArray[INERTIAL_X] = &imu.inertialX;
  floatPointerArray[INERTIAL_Y] = &imu.inertialY;
  floatPointerArray[INERTIAL_Z] = &imu.inertialZ;

  floatPointerArray[INERTIAL_X_BIASED] = &imu.inertialXBiased;
  floatPointerArray[INERTIAL_Y_BIASED] = &imu.inertialYBiased;
  floatPointerArray[INERTIAL_Z_BIASED] = &imu.inertialZBiased;

  floatPointerArray[RAW_PITCH] = &imu.rawPitch;
  floatPointerArray[RAW_ROLL] = &imu.rawRoll;
  floatPointerArray[PITCH_OFF] = &imu.pitchOffset;
  floatPointerArray[ROLL_OFF] = &imu.rollOffset;


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

  floatPointerArray[ADJ_X] = &adjustmentX;
  floatPointerArray[ADJ_Y] = &adjustmentY;
  floatPointerArray[ADJ_Z] = &adjustmentZ;

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
  floatPointerArray[DIST_TO_WP] = &distToWayPoint;

  floatPointerArray[TARGET_VEL_WP] = &landingThroAdjustment;


  floatPointerArray[MOTOR_CMD_1] = &motorCommand1;
  floatPointerArray[MOTOR_CMD_2] = &motorCommand2;
  floatPointerArray[MOTOR_CMD_3] = &motorCommand3;
  floatPointerArray[MOTOR_CMD_4] = &motorCommand4;

  floatPointerArray[PRESSURE_] = &pressure;
  floatPointerArray[CTRL_BEARING] = &controlBearing;
  floatPointerArray[YAW_INITIAL] = &initialYaw;
  floatPointerArray[GPS_ALT] = &gpsAlt;


  floatPointerArray[LAT_] = &floatLat;
  floatPointerArray[LON_] = &floatLon;
  floatPointerArray[HB_LAT] = &homeLat;
  floatPointerArray[HB_LON] = &homeLon;

  floatPointerArray[H_ACC] = &hAcc;
  floatPointerArray[S_ACC] = &sAcc;
  floatPointerArray[P_DOP] = &pDop;



  int16PointerArray[GYRO_X] = &gyroX;
  int16PointerArray[GYRO_Y] = &gyroY;
  int16PointerArray[GYRO_Z] = &gyroZ;
  int16PointerArray[ACC_X] = &accX;
  int16PointerArray[ACC_Y] = &accY;
  int16PointerArray[ACC_Z] = &accZ;
  int16PointerArray[MAG_X] = &magX;
  int16PointerArray[MAG_Y] = &magY;
  int16PointerArray[MAG_Z] = &magZ;
  int16PointerArray[THRO_CMD] = &throttleCommand;

  int16PointerArray[PWM_HIGH] = &pwmHigh;
  int16PointerArray[PWM_LOW] = &pwmLow;


  bytePointerArray[F_MODE_] = &flightMode;
  bytePointerArray[GPS_FIX] = &gps.data.vars.gpsFix;
  bytePointerArray[XY_LOIT_STATE] = &XYLoiterState;
  bytePointerArray[Z_LOIT_STATE] = &ZLoiterState;

  bytePointerArray[RTB_STATE] = &RTBState;
  bytePointerArray[MOTOR_STATE] = &motorState;
  bytePointerArray[TELEM_FS] = &telemFailSafe;
  bytePointerArray[GPS_FS] = &gpsFailSafe;
  bytePointerArray[SWITCH_POS] = &switchPositions;

  bytePointerArray[NUM_SATS] = &gps.data.vars.numSV;
  bytePointerArray[IDLE_PERCENT] = &propIdlePercent;
  bytePointerArray[HOVER_PERCENT] = &hoverPercent;
  bytePointerArray[TX_LOSS_RTB] = &txLossRTB;
  bytePointerArray[MAG_DET] = &imu.magDetected;
  bytePointerArray[TX_FS_STATUS] = &txFailSafe;
}
/*
void DEBUG_DUMP(){
 Port0<< _FLOAT(accXScale,7) <<"\r\n";
 Port0<< _FLOAT(accYScale,7) <<"\r\n";
 Port0<< _FLOAT(accZScale,7) <<"\r\n";
 Port0<< _FLOAT(accXOffset,7) <<"\r\n";
 Port0<< _FLOAT(accYOffset,7) <<"\r\n";
 Port0<< _FLOAT(accZOffset,7) <<"\r\n";
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
 Port0<< _FLOAT(magWInv22,7) <<"--\r\n";
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
 Port0<< rcData[0].min <<"\r\n";
 Port0<< rcData[0].max <<"\r\n";
 Port0<< rcData[0].mid <<"\r\n";
 Port0<< rcData[0].chan <<"\r\n";
 Port0<< _FLOAT(rcData[0].scale,7) <<"\r\n";
 Port0<< rcData[0].reverse <<"\r\n";

 Port0<< rcData[1].min <<"\r\n";
 Port0<< rcData[1].max <<"\r\n";
 Port0<< rcData[1].mid <<"\r\n";
 Port0<< rcData[1].chan <<"\r\n";
 Port0<< _FLOAT(rcData[1].scale,7) <<"\r\n";
 Port0<< rcData[1].reverse <<"\r\n";

 Port0<< rcData[2].min <<"\r\n";
 Port0<< rcData[2].max <<"\r\n";
 Port0<< rcData[2].mid <<"\r\n";
 Port0<< rcData[2].chan <<"\r\n";
 Port0<< _FLOAT(rcData[2].scale,7) <<"\r\n";
 Port0<< rcData[2].reverse <<"\r\n";

 Port0<< rcData[3].min <<"\r\n";
 Port0<< rcData[3].max <<"\r\n";
 Port0<< rcData[3].mid <<"\r\n";
 Port0<< rcData[3].chan <<"\r\n";
 Port0<< _FLOAT(rcData[3].scale,7) <<"\r\n";
 Port0<< rcData[3].reverse <<"\r\n";

 Port0<< rcData[4].min <<"\r\n";
 Port0<< rcData[4].max <<"\r\n";
 Port0<< rcData[4].mid <<"\r\n";
 Port0<< rcData[4].chan <<"\r\n";
 Port0<< _FLOAT(rcData[4].scale,7) <<"\r\n";
 Port0<< rcData[4].reverse <<"\r\n";

 Port0<< rcData[5].min <<"\r\n";
 Port0<< rcData[5].max <<"\r\n";
 Port0<< rcData[5].mid <<"\r\n";
 Port0<< rcData[5].chan <<"\r\n";
 Port0<< _FLOAT(rcData[5].scale,7) <<"\r\n";
 Port0<< rcData[5].reverse <<"\r\n";

 Port0<< rcData[6].min <<"\r\n";
 Port0<< rcData[6].max <<"\r\n";
 Port0<< rcData[6].mid <<"\r\n";
 Port0<< rcData[6].chan <<"\r\n";
 Port0<< _FLOAT(rcData[6].scale,7) <<"\r\n";
 Port0<< rcData[6].reverse <<"\r\n";



 Port0<< rcData[7].min <<"\r\n";
 Port0<< rcData[7].max <<"\r\n";
 Port0<< rcData[7].mid <<"\r\n";
 Port0<< rcData[7].chan <<"\r\n";
 Port0<< _FLOAT(rcData[7].scale,7) <<"\r\n";
 Port0<< rcData[7].reverse <<"\r\n";



 }
 */
void ROMFlagsCheck() {
  if (EEPROM.read(VER_FLAG_1) != VER_NUM_1 || EEPROM.read(VER_FLAG_2) != VER_NUM_2) {
    for (uint16_t i = 0; i < 600; i++) {
      EEPROM.write(i, 0xFF);
    }
    EEPROM.write(VER_FLAG_1, VER_NUM_1);
    EEPROM.write(VER_FLAG_2, VER_NUM_2);
  }
  uint16_t j;
  if (EEPROM.read(TX_FS_FLAG) != 0xAA) {
    EEPROM.write(TX_FS, 0);
    EEPROM.write(TX_FS_FLAG, 0xAA);
  }
  if (EEPROM.read(PR_FLAG) != 0xAA) {
    imu.pitchOffset.val = 0;
    imu.rollOffset.val = 0;
    j = 0;
    for (uint16_t i = PITCH_OFFSET_START; i <= PITCH_OFFSET_END; i++) {
      EEPROM.write(i, imu.pitchOffset.buffer[j++]);
    }
    j = 0;
    for (uint16_t i = ROLL_OFFSET_START; i <= ROLL_OFFSET_END; i++) {
      EEPROM.write(i, imu.rollOffset.buffer[j++]);
    }
    EEPROM.write(PR_FLAG, 0xAA);
  }
  if (EEPROM.read(PWM_FLAG) != 0xAA) {
    pwmHigh.val = 2000;
    pwmLow.val = 1000;
    EEPROM.write(PWM_LIM_HIGH_START, pwmHigh.buffer[0]);
    EEPROM.write(PWM_LIM_HIGH_END, pwmHigh.buffer[1]);
    EEPROM.write(PWM_LIM_LOW_START, pwmLow.buffer[0]);
    EEPROM.write(PWM_LIM_LOW_END, pwmLow.buffer[1]);
    EEPROM.write(PWM_FLAG, 0xAA);
  }
  if (EEPROM.read(PROP_IDLE_FLAG) != 0xAA) {
    EEPROM.write(PROP_IDLE_FLAG, 0xAA);
    EEPROM.write(PROP_IDLE, 12);
  }
  if (EEPROM.read(HOVER_THRO_FLAG) != 0xAA) {
    EEPROM.write(HOVER_THRO_FLAG, 0xAA);
    EEPROM.write(HOVER_THRO, 55);

  }

  calibrationFlags = EEPROM.read(CAL_FLAGS);
  VerifyMag();
  if ( ((calibrationFlags & (1 << RC_FLAG)) >> RC_FLAG) == 0x01 || ((calibrationFlags & (1 << ACC_FLAG)) >> ACC_FLAG) == 0x01 || ( ((calibrationFlags & (1 << MAG_FLAG)) >> MAG_FLAG) == 0x01 && imu.magDetected ) ) {
    //if ( ((calibrationFlags & (1<<RC_FLAG)) >> RC_FLAG) == 0x01 || ((calibrationFlags & (1<<ACC_FLAG)) >> ACC_FLAG) == 0x01 ||  ((calibrationFlags & (1<<MAG_FLAG)) >> MAG_FLAG) == 0x01  ){
    Port2.begin(115200);
    radioStream = &Port2;
    radioPrint = &Port2;
    HandShake();

    if (handShake == false) {
      USBFlag = true;
      radioStream = &Port0;
      radioPrint = &Port0;
      HandShake();
    }
    if (calibrationMode == true) {
      digitalWrite(RED, HIGH);
      digitalWrite(YELLOW, HIGH);
      digitalWrite(GREEN, HIGH);
      digitalWrite(13, LOW);
      return;
    }
    toggle = false;
    while (1) {
      if ( ((calibrationFlags & (1 << RC_FLAG)) >> RC_FLAG) == 0x01 ) {
        digitalWrite(RED, toggle);
      }
      if ( ((calibrationFlags & (1 << ACC_FLAG)) >> ACC_FLAG) == 0x01 ) {
        digitalWrite(YELLOW, toggle);
      }
      if ( ((calibrationFlags & (1 << MAG_FLAG)) >> MAG_FLAG) == 0x01 ) {
        digitalWrite(GREEN, toggle);
      }
      toggle = toggle ^ 1;
      delay(300);
    }
  }

  if ( ((calibrationFlags & (1 << GAINS_FLAG)) >> GAINS_FLAG) == 0x01 ) {
    SetDefaultGains();
  }
  LoadROM();


}
void SetDefaultGains() {

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

  kp_pitch_attitude.val = 4.0;
  ki_pitch_attitude.val = 0;
  kd_pitch_attitude.val = 0.01;
  fc_pitch_attitude.val = 50.0;

  kp_roll_attitude.val = 4.0;
  ki_roll_attitude.val = 0;
  kd_roll_attitude.val = 0.01;
  fc_roll_attitude.val = 50;

  kp_yaw_attitude.val = 3.0;
  ki_yaw_attitude.val = 0;
  kd_yaw_attitude.val = 0.0;
  fc_yaw_attitude.val = 50.0;

  kp_altitude_position.val = 0.5;
  ki_altitude_position.val = 0;
  kd_altitude_position.val = -0.002;
  fc_altitude_position.val = 30;

  kp_altitude_velocity.val = 45;
  ki_altitude_velocity.val = 30;
  kd_altitude_velocity.val = 0;
  fc_altitude_velocity.val = 50;

  kp_loiter_pos_x.val = 0.1;
  ki_loiter_pos_x.val = 0;
  kd_loiter_pos_x.val = 0;
  fc_loiter_pos_x.val = 50;

  kp_loiter_velocity_x.val = 7.0;
  ki_loiter_velocity_x.val = 1.5;
  kd_loiter_velocity_x.val = 0.075;
  fc_loiter_velocity_x.val = 50;

  kp_loiter_pos_y.val = 0.1;
  ki_loiter_pos_y.val = 0.0;
  kd_loiter_pos_y.val = 0.0;
  fc_loiter_pos_y.val = 50;

  kp_loiter_velocity_y.val = 7.0;
  ki_loiter_velocity_y.val = 1.5;
  kd_loiter_velocity_y.val = 0.075;
  fc_loiter_velocity_y.val = 50;

  kp_waypoint_position.val = 2.5;
  ki_waypoint_position.val = 0;
  kd_waypoint_position.val = 2.5;
  fc_waypoint_position.val = 0;

  kp_waypoint_velocity.val = 0.1;
  ki_waypoint_velocity.val = 0.1;
  kd_waypoint_velocity.val = 0.2;
  fc_waypoint_velocity.val = 0.03;

  kp_cross_track.val = 0.07;
  ki_cross_track.val = 0.1;
  kd_cross_track.val = 0.01;
  fc_cross_track.val = 3.3;

  imu.declination.val = ToRad(3.3);
  j = GAINS_START;
  for (uint16_t i = KP_PITCH_RATE_; i <= FC_CT_; i++) {
    EEPROM.write(j++, (*floatPointerArray[i]).buffer[0]);
    EEPROM.write(j++, (*floatPointerArray[i]).buffer[1]);
    EEPROM.write(j++, (*floatPointerArray[i]).buffer[2]);
    EEPROM.write(j++, (*floatPointerArray[i]).buffer[3]);
  }
  j = DEC_START;
  EEPROM.write(j++, (*floatPointerArray[MAG_DEC_]).buffer[0]);
  EEPROM.write(j++, (*floatPointerArray[MAG_DEC_]).buffer[1]);
  EEPROM.write(j++, (*floatPointerArray[MAG_DEC_]).buffer[2]);
  EEPROM.write(j++, (*floatPointerArray[MAG_DEC_]).buffer[3]);


}
void LoadPWMLimits() {
  int16_u outInt16;
  outInt16.buffer[0] = EEPROM.read(PWM_LIM_HIGH_START);
  outInt16.buffer[1] = EEPROM.read(PWM_LIM_HIGH_END);
  pwmHigh.val = outInt16.val;
  if (pwmHigh.val > 2000) {
    pwmHigh.val = 2000;
  }
  if (pwmHigh.val < 1800) {
    pwmHigh.val = 1800;
  }
  outInt16.buffer[0] = EEPROM.read(PWM_LIM_LOW_START);
  outInt16.buffer[1] = EEPROM.read(PWM_LIM_LOW_END);
  pwmLow.val = outInt16.val;
  if (pwmLow.val < 1000) {
    pwmLow.val = 1000;
  }
  if (pwmLow.val > 1200) {
    pwmLow.val = 1200;
  }
  propIdlePercent = EEPROM.read(PROP_IDLE);
  if (propIdlePercent > 20) {
    propIdleCommand = pwmLow.val * (1 + (20.0 / 100.0));
  }
  else {
    propIdleCommand = pwmLow.val * (1 + ((float)propIdlePercent / 100.0));
  }
  hoverPercent = EEPROM.read(HOVER_THRO);
  if (hoverPercent > 75) {
    hoverCommand = 1000 * (1 + (75 / 100.0));
  }
  else {
    if (hoverPercent < 25) {
      hoverCommand = 1000 * (1 + (25 / 100.0));
    }
    else {
      hoverCommand = 1000 * (1 + ((float)hoverPercent / 100.0));
    }
  }
  //Serial << pwmLow.val << "," << pwmHigh.val << "," << propIdleCommand << "," << hoverCommand << "\r\n";
}
void LoadRC() {
  uint16_t j = 0; //index for input buffers
  uint16_t k = 0; //index for start of each channel's data in rom
  uint16_t l = 0; //index for each channel
  uint16_t switchControl;
  uint16_u outInt16;
  for (uint16_t i = RC_DATA_START; i <= RC_DATA_END; i++) { //index for each rom location
    switchControl = i - k;
    if (switchControl < CHAN_INDEX) { //first 16 bit ints
      outInt16.buffer[j++] = EEPROM.read(i);
    }
    if (switchControl > CHAN_INDEX && i - k < REV_INDEX) { //scale factor
      outFloat.buffer[j++] = EEPROM.read(i);
    }

    switch (switchControl) {
      case MAX_INDEX://max
        rcData[l].max = outInt16.val;
        j = 0;
        break;
      case MIN_INDEX://min
        rcData[l].min = outInt16.val;
        j = 0;
        break;
      case MID_INDEX://mid
        rcData[l].mid = outInt16.val;
        j = 0;
        break;
      case CHAN_INDEX://chan
        rcData[l].chan = EEPROM.read(i);
        break;
      case SCALE_INDEX://scale
        rcData[l].scale = outFloat.val;
        j = 0;
        break;
      case REV_INDEX://reverse
        rcData[l].reverse = EEPROM.read(i);
        k += 12;
        l += 1;
        break;
    }
  }
  //Serial<<rcData[0].reverse<<","<<rcData[1].reverse<<","<<rcData[2].reverse<<","<<rcData[3].reverse<<","<<rcData[4].reverse<<","<<rcData[5].reverse<<","<<rcData[6].reverse<<","<<rcData[7].reverse<<"\r\n";
  txLossRTB = EEPROM.read(TX_FS);
  //Serial << txLossRTB << " " << EEPROM.read(TX_FS) << "\r\n";
  if (txLossRTB > 1) {
    txLossRTB = 0;
  }

}
void LoadACC() {
  uint8_t outFloatIndex = 0;
  for (uint16_t i = ACC_CALIB_START; i <= ACC_CALIB_END; i++) { //load acc values
    outFloat.buffer[outFloatIndex] = EEPROM.read(i);
    outFloatIndex++;
    switch (i) {
      case ACC_S_X_INDEX:
        accXScale = outFloat.val;
        outFloatIndex = 0;
        break;
      case ACC_S_Y_INDEX:
        accYScale = outFloat.val;
        outFloatIndex = 0;
        break;
      case ACC_S_Z_INDEX:
        accZScale = outFloat.val;
        outFloatIndex = 0;
        break;
      case ACC_O_X_INDEX:
        accXOffset = outFloat.val;
        outFloatIndex = 0;
        break;
      case ACC_O_Y_INDEX:
        accYOffset = outFloat.val;
        outFloatIndex = 0;
        break;
      case ACC_O_Z_INDEX:
        accZOffset = outFloat.val;
        outFloatIndex = 0;
        break;
      default:
        break;
    }
  }
}

void LoadMAG() {


  uint8_t outFloatIndex = 0;
  for (uint16_t i = MAG_CALIB_START; i <= MAG_CALIB_END; i++) { //load the compass values

    outFloat.buffer[outFloatIndex] = EEPROM.read(i);
    outFloatIndex++;
    switch (i) {
      case MAG_OFF_X_INDEX:
        magOffSetX = outFloat.val;
        outFloatIndex = 0;
        break;
      case MAG_OFF_Y_INDEX:
        magOffSetY = outFloat.val;
        outFloatIndex = 0;
        break;
      case MAG_OFF_Z_INDEX:
        magOffSetZ = outFloat.val;
        outFloatIndex = 0;
        break;
      case W_00_INDEX:
        magWInv00 = outFloat.val;
        outFloatIndex = 0;
        break;
      case W_01_INDEX:
        magWInv01 = outFloat.val;
        outFloatIndex = 0;
        break;
      case W_02_INDEX:
        magWInv02 = outFloat.val;
        outFloatIndex = 0;
        break;
      case W_10_INDEX:
        magWInv10 = outFloat.val;
        outFloatIndex = 0;
        break;
      case W_11_INDEX:
        magWInv11 = outFloat.val;
        outFloatIndex = 0;
        break;
      case W_12_INDEX:
        magWInv12 = outFloat.val;
        outFloatIndex = 0;
        break;
      case W_20_INDEX:
        magWInv20 = outFloat.val;
        outFloatIndex = 0;
        break;
      case W_21_INDEX:
        magWInv21 = outFloat.val;
        outFloatIndex = 0;
        break;
      case W_22_INDEX:
        magWInv22 = outFloat.val;
        outFloatIndex = 0;
        break;
      default:
        break;
    }
  }
}

void LoadGains() {
  uint16_t j = GAINS_START;
  for (uint16_t i = KP_PITCH_RATE_; i <= FC_CT_; i++) { //gains
    (*floatPointerArray[i]).buffer[0] = EEPROM.read(j++);
    (*floatPointerArray[i]).buffer[1] = EEPROM.read(j++);
    (*floatPointerArray[i]).buffer[2] = EEPROM.read(j++);
    (*floatPointerArray[i]).buffer[3] = EEPROM.read(j++);
  }
}

void LoadPROff() {
  uint16_t j = PITCH_OFFSET_START;
  for (uint16_t i = PITCH_OFF; i <= ROLL_OFF; i++) { //pitch and roll offsets
    (*floatPointerArray[i]).buffer[0] = EEPROM.read(j++);
    (*floatPointerArray[i]).buffer[1] = EEPROM.read(j++);
    (*floatPointerArray[i]).buffer[2] = EEPROM.read(j++);
    (*floatPointerArray[i]).buffer[3] = EEPROM.read(j++);
  }
}

void LoadDEC() {
  uint16_t j = DEC_START;
  (*floatPointerArray[MAG_DEC_]).buffer[0] = EEPROM.read(j++);
  (*floatPointerArray[MAG_DEC_]).buffer[1] = EEPROM.read(j++);
  (*floatPointerArray[MAG_DEC_]).buffer[2] = EEPROM.read(j++);
  (*floatPointerArray[MAG_DEC_]).buffer[3] = EEPROM.read(j++);

  imu.COS_DEC = cos(imu.declination.val);
  imu.SIN_DEC = sin(imu.declination.val);
}
void LoadROM() {

  LoadRC();

  LoadACC();

  LoadMAG();

  LoadGains();

  LoadPROff();

  LoadPWMLimits();



}

















































