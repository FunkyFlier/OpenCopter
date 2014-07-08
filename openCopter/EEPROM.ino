#include "AUXMATH.h"

/*void DEBUG_DUMP(){
  Serial<< _FLOAT(ACC_OFFSET_X,7) <<"\r\n";
  Serial<< _FLOAT(ACC_OFFSET_Y,7) <<"\r\n";
  Serial<< _FLOAT(ACC_OFFSET_Z,7) <<"\r\n";
  Serial<< _FLOAT(ACC_W_INV_00,7) <<"\r\n";
  Serial<< _FLOAT(ACC_W_INV_01,7) <<"\r\n";
  Serial<< _FLOAT(ACC_W_INV_02,7) <<"\r\n";
  Serial<< _FLOAT(ACC_W_INV_10,7) <<"\r\n";
  Serial<< _FLOAT(ACC_W_INV_11,7) <<"\r\n";
  Serial<< _FLOAT(ACC_W_INV_12,7) <<"\r\n";
  Serial<< _FLOAT(ACC_W_INV_20,7) <<"\r\n";
  Serial<< _FLOAT(ACC_W_INV_21,7) <<"\r\n";
  Serial<< _FLOAT(ACC_W_INV_22,7) <<"\r\n";

  Serial<< _FLOAT(MAG_OFFSET_X,7) <<"\r\n";
  Serial<< _FLOAT(MAG_OFFSET_Y,7) <<"\r\n";
  Serial<< _FLOAT(MAG_OFFSET_Z,7) <<"\r\n";
  Serial<< _FLOAT(MAG_W_INV_00,7) <<"\r\n";
  Serial<< _FLOAT(MAG_W_INV_01,7) <<"\r\n";
  Serial<< _FLOAT(MAG_W_INV_02,7) <<"\r\n";
  Serial<< _FLOAT(MAG_W_INV_10,7) <<"\r\n";
  Serial<< _FLOAT(MAG_W_INV_11,7) <<"\r\n";
  Serial<< _FLOAT(MAG_W_INV_12,7) <<"\r\n";
  Serial<< _FLOAT(MAG_W_INV_20,7) <<"\r\n";
  Serial<< _FLOAT(MAG_W_INV_21,7) <<"\r\n";
  Serial<< _FLOAT(MAG_W_INV_22,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kp_pitch_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.ki_pitch_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kd_pitch_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.fc_pitch_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kp_roll_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.ki_roll_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kd_roll_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.fc_roll_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kp_yaw_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.ki_yaw_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kd_yaw_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.fc_yaw_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kp_pitch_attitude,7) <<"\r\n";
  Serial<< _FLOAT(d.v.ki_pitch_attitude,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kd_pitch_attitude,7) <<"\r\n";
  Serial<< _FLOAT(d.v.fc_pitch_attitude,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kp_roll_attitude,7) <<"\r\n";
  Serial<< _FLOAT(d.v.ki_roll_attitude,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kd_roll_attitude,7) <<"\r\n";
  Serial<< _FLOAT(d.v.fc_roll_attitude,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kp_yaw_attitude,7) <<"\r\n";
  Serial<< _FLOAT(d.v.ki_yaw_attitude,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kd_yaw_attitude,7) <<"\r\n";
  Serial<< _FLOAT(d.v.fc_yaw_attitude,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kp_altitude_position,7) <<"\r\n";
  Serial<< _FLOAT(d.v.ki_altitude_position,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kd_altitude_position,7) <<"\r\n";
  Serial<< _FLOAT(d.v.fc_altitude_position,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kp_altitude_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.ki_altitude_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kd_altitude_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.fc_altitude_rate,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kp_loiter_pos_x,7) <<"\r\n";
  Serial<< _FLOAT(d.v.ki_loiter_pos_x,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kd_loiter_pos_x,7) <<"\r\n";
  Serial<< _FLOAT(d.v.fc_loiter_pos_x,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kp_loiter_rate_x,7) <<"\r\n";
  Serial<< _FLOAT(d.v.ki_loiter_rate_x,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kd_loiter_rate_x,7) <<"\r\n";
  Serial<< _FLOAT(d.v.fc_loiter_rate_x,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kp_loiter_pos_y,7) <<"\r\n";
  Serial<< _FLOAT(d.v.ki_loiter_pos_y,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kd_loiter_pos_y,7) <<"\r\n";
  Serial<< _FLOAT(d.v.fc_loiter_pos_y,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kp_loiter_rate_y,7) <<"\r\n";
  Serial<< _FLOAT(d.v.ki_loiter_rate_y,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kd_loiter_rate_y,7) <<"\r\n";
  Serial<< _FLOAT(d.v.fc_loiter_rate_y,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kp_waypoint_position,7) <<"\r\n";
  Serial<< _FLOAT(d.v.ki_waypoint_position,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kd_waypoint_position,7) <<"\r\n";
  Serial<< _FLOAT(d.v.fc_waypoint_position,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kp_waypoint_velocity,7) <<"\r\n";
  Serial<< _FLOAT(d.v.ki_waypoint_velocity,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kd_waypoint_velocity,7) <<"\r\n";
  Serial<< _FLOAT(d.v.fc_waypoint_velocity,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kp_cross_track,7) <<"\r\n";
  Serial<< _FLOAT(d.v.ki_cross_track,7) <<"\r\n";
  Serial<< _FLOAT(d.v.kd_cross_track,7) <<"\r\n";
  Serial<< _FLOAT(d.v.fc_cross_track,7) <<"\r\n";
  Serial<< _FLOAT(d.v.declination,7) <<"\r\n";

}*/

void LEDIndicators(){
  calibrationFlags = EEPROM.read(0x00);
  generalPurposeTimer = millis();

  digitalWrite(13,LOW);
  digitalWrite(RED,LOW);
  digitalWrite(YELLOW,LOW);
  digitalWrite(GREEN,LOW);
  while(millis() - generalPurposeTimer < 3000){
    if ( ((calibrationFlags & (1<<ALL_DEF)) >>ALL_DEF) == 0x01 ){
      digitalWrite(13,toggle);
    }
    if ( ((calibrationFlags & (1<<GAIN_DEF)) >>GAIN_DEF) == 0x01 ){
      digitalWrite(RED,toggle);
    }
    if ( ((calibrationFlags & (1<<COMP_DEF)) >>COMP_DEF) == 0x01 ){
      digitalWrite(YELLOW,toggle);
    }
    if ( ((calibrationFlags & (1<<ACC_DEF)) >>ACC_DEF) == 0x01 ){
      digitalWrite(GREEN,toggle);
    }
    toggle = ~toggle;
    delay(300);
  }
  digitalWrite(13,LOW);
  digitalWrite(RED,LOW);
  digitalWrite(YELLOW,LOW);
  digitalWrite(GREEN,LOW);
}

void ROMFlagsCheck(){

  calibrationFlags = EEPROM.read(0x00);

  if (calibrationFlags == 0x00 || calibrationFlags == 0xFF){
    //0xFF is what is in an unwritten ROM cell and will never be a correct setting for the flag byte
    SetDefaultGains();
    SetDefaultCompassCal();
    SetDefaultAcc();
    EEPROM.write(0x00,0xF0);  
    LEDIndicators();
  }

  if ( calibrationFlags  == 0x0F){
    //all calibrated and gains set
    LoadROM();
    allCalibrated = true;
    return;
  }

  if ( calibrationFlags  == 0xF0){
    //defaults loaded
    LoadROM();
    LEDIndicators();
    return;
  }

  if ( ( ((calibrationFlags & 1<<GAIN_DEF) >> GAIN_DEF == 0x01) && ((calibrationFlags & 1<<GAIN_CAL) >> GAIN_CAL == 0x01) ) 
    || ( ((calibrationFlags & 1<<GAIN_CAL) >> GAIN_CAL == 0x00) && ((calibrationFlags & 1<<GAIN_DEF) >> GAIN_DEF == 0x00)) ){
    //both flags are either set or cleared
    SetDefaultGains();
  }

  if ( ( ((calibrationFlags & 1<<COMP_DEF) >> COMP_DEF == 0x01) && ((calibrationFlags & 1<<COMP_CAL) >> COMP_CAL == 0x01) ) 
    || ( ((calibrationFlags & 1<<COMP_CAL) >> COMP_CAL == 0x00) && ((calibrationFlags & 1<<COMP_DEF) >> COMP_DEF == 0x00)) ){
    //both flags are either set or cleared
    SetDefaultCompassCal();
  }

  if ( ( ((calibrationFlags & 1<<ACC_DEF) >> ACC_DEF == 0x01) && ((calibrationFlags & 1<<ACC_CAL) >> ACC_CAL == 0x01) ) 
    || ( ((calibrationFlags & 1<<ACC_CAL) >> ACC_CAL == 0x00) && ((calibrationFlags & 1<<ACC_DEF) >> ACC_DEF == 0x00)) ){
    //both flags are either set or cleared
    SetDefaultAcc();
  }

  LoadROM();
  LEDIndicators();

}

void LoadROM(){
  outFloatIndex = 0;
  for (uint16_t i = 0x01; i <= 0x30; i++){//load acc values
    outFloat.buffer[outFloatIndex] = EEPROM.read(i);
    outFloatIndex++;
    switch (i){
    case 0x04:
      ACC_OFFSET_X = outFloat.num;
      outFloatIndex = 0;
      break;
    case 0x08:
      ACC_OFFSET_Y = outFloat.num;
      outFloatIndex = 0;
      break;
    case 0x0C:
      ACC_OFFSET_Z = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x10:
      ACC_W_INV_00 = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x14:
      ACC_W_INV_01 = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x18:
      ACC_W_INV_02 = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x1C:
      ACC_W_INV_10 = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x20:
      ACC_W_INV_11 = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x24:
      ACC_W_INV_12 = outFloat.num;
      outFloatIndex = 0;
      break; 
    case 0x28:
      ACC_W_INV_20 = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x2C:
      ACC_W_INV_21 = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x30:
      ACC_W_INV_22 = outFloat.num;
      outFloatIndex = 0;
      break;       
    default:
      break;
    }
  }

  outFloatIndex = 0;
  for (uint16_t i = 0x31; i <= 0x60; i++){//load the compass values
    outFloat.buffer[outFloatIndex] = EEPROM.read(i);
    outFloatIndex++;
    switch (i){
    case 0x34:
      MAG_OFFSET_X = outFloat.num; 
      outFloatIndex = 0;
      break;
    case 0x38:
      MAG_OFFSET_Y = outFloat.num;
      outFloatIndex = 0;
      break;
    case 0x3C:
      MAG_OFFSET_Z = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x40:
      MAG_W_INV_00 = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x44:
      MAG_W_INV_01 = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x48:
      MAG_W_INV_02 = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x4C:
      MAG_W_INV_10 = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x50:
      MAG_W_INV_11 = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x54:
      MAG_W_INV_12 = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x58:
      MAG_W_INV_20 = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x5C:
      MAG_W_INV_21 = outFloat.num;
      outFloatIndex = 0;
      break;  
    case 0x60:
      MAG_W_INV_22 = outFloat.num;
      outFloatIndex = 0;
      break;  
    default:
      break;
    }
  }
  //load the gains  
  for (uint16_t i = 0x61; i <= 0x154; i++){
    d.buffer[(i - 45)] = EEPROM.read(i);
  }

}

void SetDefaultAcc(){

  ACC_OFFSET_X = 8.466008;
  ACC_OFFSET_Y = -0.685062;
  ACC_OFFSET_Z = 9.408270;
  ACC_W_INV_00 = 0.038806;
  ACC_W_INV_01 = -0.000011;
  ACC_W_INV_02 = 0.000064;
  ACC_W_INV_10 = -0.000011;
  ACC_W_INV_11 = 0.038142;
  ACC_W_INV_12 = -0.000280;
  ACC_W_INV_20 = 0.000064;
  ACC_W_INV_21 = -0.000280;
  ACC_W_INV_22 = 0.038950;
  for (uint16_t i = 0x01; i <= 0x30; i++){
    switch (i){
    case 0x01:
      outFloat.num = ACC_OFFSET_X;
      outFloatIndex = 0;
      break;
    case 0x05:
      outFloat.num = ACC_OFFSET_Y;
      outFloatIndex = 0;
      break;
    case 0x09:
      outFloat.num = ACC_OFFSET_Z;
      outFloatIndex = 0;
      break;  
    case 0x0D:
      outFloat.num = ACC_W_INV_00;
      outFloatIndex = 0;
      break;  
    case 0x11:
      outFloat.num = ACC_W_INV_01;
      outFloatIndex = 0;
      break;  
    case 0x15:
      outFloat.num = ACC_W_INV_02;
      outFloatIndex = 0;
      break;  
    case 0x19:
      outFloat.num = ACC_W_INV_10;
      outFloatIndex = 0;
      break;  
    case 0x1D:
      outFloat.num = ACC_W_INV_11;
      outFloatIndex = 0;
      break;  
    case 0x21:
      outFloat.num = ACC_W_INV_12;
      outFloatIndex = 0;
      break; 
    case 0x25:
      outFloat.num = ACC_W_INV_20;
      outFloatIndex = 0;
      break;  
    case 0x29:
      outFloat.num = ACC_W_INV_21;
      outFloatIndex = 0;
      break;  
    case 0x2D:
      outFloat.num = ACC_W_INV_22;
      outFloatIndex = 0;
      break;       
    default:
      break;
    }
    EEPROM.write(i,outFloat.buffer[outFloatIndex]);
    outFloatIndex++;
  }

  calibrationFlags = EEPROM.read(0x00);
  calibrationFlags |= (1<<ACC_DEF);
  calibrationFlags &= ~(1<<ACC_CAL);
  calibrationFlags &= ~(1<<ALL_CAL);
  EEPROM.write(0x00,calibrationFlags);  

}

void SetDefaultCompassCal(){

  MAG_OFFSET_X =10.370532;
  MAG_OFFSET_Y = 90.317025;
  MAG_OFFSET_Z = -16.633857;
  MAG_W_INV_00 = 0.956960;
  MAG_W_INV_01 = -0.004750;
  MAG_W_INV_02 = -0.011861;
  MAG_W_INV_10 = -0.004750;
  MAG_W_INV_11 = 0.950338;
  MAG_W_INV_12 = -0.005574;
  MAG_W_INV_20 = -0.011861;
  MAG_W_INV_21 = -0.005574;
  MAG_W_INV_22 = 1.170783;
  for (uint16_t i = 0x31; i <= 0x60; i++){
    switch (i){
    case 0x31:
      outFloat.num = MAG_OFFSET_X;
      outFloatIndex = 0;
      break;
    case 0x35:
      outFloat.num = MAG_OFFSET_Y;
      outFloatIndex = 0;
      break;
    case 0x39:
      outFloat.num = MAG_OFFSET_Z;
      outFloatIndex = 0;
      break;  
    case 0x3D:
      outFloat.num = MAG_W_INV_00;
      outFloatIndex = 0;
      break;  
    case 0x41:
      outFloat.num = MAG_W_INV_01;
      outFloatIndex = 0;
      break;  
    case 0x45:
      outFloat.num = MAG_W_INV_02;
      outFloatIndex = 0;
      break;  
    case 0x49:
      outFloat.num = MAG_W_INV_10;
      outFloatIndex = 0;
      break;  
    case 0x4D:
      outFloat.num = MAG_W_INV_11;
      outFloatIndex = 0;
      break;  
    case 0x51:
      outFloat.num = MAG_W_INV_12;
      outFloatIndex = 0;
      break;  
    case 0x55:
      outFloat.num = MAG_W_INV_20;
      outFloatIndex = 0;
      break;  
    case 0x59:
      outFloat.num = MAG_W_INV_21;
      outFloatIndex = 0;
      break;  
    case 0x5D:
      outFloat.num = MAG_W_INV_22;
      outFloatIndex = 0;
      break;  
    default:
      break;
    }
    EEPROM.write(i,outFloat.buffer[outFloatIndex]);
    outFloatIndex++;
  }

  calibrationFlags = EEPROM.read(0x00);
  calibrationFlags |= (1<<COMP_DEF);
  calibrationFlags &= ~(1<<COMP_CAL);
  calibrationFlags &= ~(1<<ALL_CAL);
  EEPROM.write(0x00,calibrationFlags);



}
void SetDefaultGains(){
  d.v.kp_pitch_rate = 0.4125;
  d.v.ki_pitch_rate = 2.9049296;
  d.v.kd_pitch_rate = 0.03905;
  d.v.fc_pitch_rate = 50.0;

  d.v.kp_roll_rate = 0.4125;
  d.v.ki_roll_rate = 2.9049296;
  d.v.kd_roll_rate = 0.03905;
  d.v.fc_roll_rate = 50.0;

  d.v.kp_yaw_rate = 2.25;
  d.v.ki_yaw_rate = 0.25;
  d.v.kd_yaw_rate = 0.01;
  d.v.fc_yaw_rate = 50.0;

  d.v.kp_pitch_attitude = 5.35;
  d.v.ki_pitch_attitude = 0;
  d.v.kd_pitch_attitude = 0.075;
  d.v.fc_pitch_attitude = 75.0;

  d.v.kp_roll_attitude = 5.35;
  d.v.ki_roll_attitude = 0;
  d.v.kd_roll_attitude = 0.075;
  d.v.fc_roll_attitude = 75.0;

  d.v.kp_yaw_attitude = 3.0;
  d.v.ki_yaw_attitude = 0;
  d.v.kd_yaw_attitude = 0.01;
  d.v.fc_yaw_attitude = 50.0;

  d.v.kp_altitude_position = 1.5;
  d.v.ki_altitude_position = 0;
  d.v.kd_altitude_position = 0;
  d.v.fc_altitude_position = 0;

  d.v.kp_altitude_rate = 90;
  d.v.ki_altitude_rate = 35;
  d.v.kd_altitude_rate = 0.05;
  d.v.fc_altitude_rate = 50;

  d.v.kp_loiter_pos_x = 0;
  d.v.ki_loiter_pos_x = 0;
  d.v.kd_loiter_pos_x = 0;
  d.v.fc_loiter_pos_x = 0;

  d.v.kp_loiter_rate_x = 0;
  d.v.ki_loiter_rate_x = 0;
  d.v.kd_loiter_rate_x = 0;
  d.v.fc_loiter_rate_x = 0;

  d.v.kp_loiter_pos_y = 0;
  d.v.ki_loiter_pos_y = 0;
  d.v.kd_loiter_pos_y = 0;
  d.v.fc_loiter_pos_y = 0;

  d.v.kp_loiter_rate_y = 0;
  d.v.ki_loiter_rate_y = 0;
  d.v.kd_loiter_rate_y = 0;
  d.v.fc_loiter_rate_y = 0;

  d.v.kp_waypoint_position = 10;
  d.v.ki_waypoint_position = 0;
  d.v.kd_waypoint_position = 0;
  d.v.fc_waypoint_position = 0;

  d.v.kp_waypoint_velocity = 1;
  d.v.ki_waypoint_velocity = 2.0;
  d.v.kd_waypoint_velocity = 1.0;
  d.v.fc_waypoint_velocity = 0.0001;

  d.v.kp_cross_track = 1;
  d.v.ki_cross_track = 0.0;
  d.v.kd_cross_track = 0.06;
  d.v.fc_cross_track = 0.04;

  d.v.declination = 3.66;

  for (uint16_t i = 0x61; i <= 0x154; i++){
    EEPROM.write(i,d.buffer[(i - 45)]);
  }

  calibrationFlags = EEPROM.read(0x00);
  calibrationFlags |= (1<<GAIN_DEF);//in two places - make a choice************
  calibrationFlags &= ~(1<<GAIN_CAL);
  calibrationFlags &= ~(1<<ALL_CAL);
  EEPROM.write(0x00,calibrationFlags);

}























