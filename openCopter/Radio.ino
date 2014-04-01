void TuningTransmitter(){

  if (tuningTransmit == true){
    if (millis() - tuningTimer >= refreshMillis){
      tuningTimer = millis();
      outputSum = 0;
      outputDoubleSum = 0;
      radio.write(0xAA);
      lenOfTuningDataPacket = numOfItems * 5 + 2;
      radio.write(lenOfTuningDataPacket);
      //switch to a define
      radio.write(0x65);
      outputSum += 0x65;
      outputDoubleSum += outputSum;
      radio.write(numOfItems);
      outputSum += numOfItems;
      outputDoubleSum += outputSum;
      for (uint8_t i = 0; i < numOfItems; i++){//comp filter tuning
        switch (itemBuffer[i]){
        case 0x00:
          outFloat.num = millis();
          break;
        case 0x01:
          outFloat.num = rawX;
          break;
        case 0x02:
          outFloat.num = imu.XEst;
          break;
        case 0x03:
          outFloat.num = imu.velX;
          break;
        case 0x04:
          outFloat.num = imu.inertialX;
          break;
        case 0x05:
          outFloat.num = rawY;
          break;
        case 0x06:
          outFloat.num = imu.YEst;
          break;
        case 0x07:
          outFloat.num = imu.velY;
          break;
        case 0x08:
          outFloat.num = imu.inertialY;
          break;
        case 0x09:
          outFloat.num = imu.accelBiasX;
          break;
        case 0x0A:
          outFloat.num = imu.accelBiasY;
          break;
        case 0x0B:
          outFloat.num = predictedX;
          break;
        case 0x0C:
          outFloat.num = predictedY;
          break;
        case 0x0D:
          outFloat.num = positionError;
          break;
        case 0x0E:
          outFloat.num = expandingDist;
          break;
        case 0x0F:
          outFloat.num = imu.pitch;
          break;
        case 0x10:
          outFloat.num = imu.roll;
          break;
        case 0x11:
          outFloat.num = imu.yaw;
          break;
        case 0x12:
          outFloat.num = gps.hdop();
          break;
        case 0x13:
          outFloat.num = outFloat.num = (float)GPSFlag;
          if (GPSFlag == true){
            GPSFlag = false;
          }
          break;
        case 0x14:
          outFloat.num = gps.satellites();
          break;
        case 0x15:
          outFloat.num = imu.accelBiasY;
          break;
        case 0x16:
          outFloat.num = imu.accelBiasZ;
          break;
        case 0x17:
          outFloat.num = imu.xError;
          break;
        case 0x18:
          outFloat.num = imu.yError;
          break;
        case 0x19:
          outFloat.num = d.v.lattitude;
          break;
        case 0x1A:
          outFloat.num = d.v.longitude;
          break;

        default:
          break;

        }
        /* for (uint8_t i = 0; i < numOfItems; i++){//comp filter tuning
         switch (itemBuffer[i]){
         case 0x00:
         outFloat.num = millis();
         break;
         case 0x01:
         outFloat.num = xTarget;
         //outFloat.num = acc.v.x;
         break;
         case 0x02:
         outFloat.num = imu.XEst;
         //outFloat.num = acc.v.y;
         break;
         case 0x03:
         outFloat.num = velSetPointX;
         //outFloat.num = acc.v.z;
         break;
         case 0x04:
         outFloat.num = setPointX;
         //outFloat.num = smoothAccX;
         break;
         case 0x05:
         outFloat.num = yTarget;
         //outFloat.num = smoothAccY;
         break;
         case 0x06:
         outFloat.num = imu.YEst;
         //outFloat.num = smoothAccZ;
         break;
         case 0x07:
         outFloat.num = velSetPointY;
         break;
         case 0x08:
         outFloat.num = imu.velY;
         break;
         case 0x09:
         outFloat.num = setPointY;
         break;
         case 0x0A:
         outFloat.num = targetAltitude;
         break;
         case 0x0B:
         outFloat.num = imu.ZEst;
         break;
         case 0x0C:
         outFloat.num = targetVelAlt;
         break;
         case 0x0D:
         outFloat.num = imu.velZ;
         break;
         case 0x0E:
         outFloat.num = throttleAdjustment;
         break;
         case 0x0F:
         outFloat.num = imu.pitch;
         break;
         case 0x10:
         outFloat.num = imu.roll;
         break;
         case 0x11:
         outFloat.num = imu.yaw;
         break;
         case 0x12:
         outFloat.num = gps.data.vars.hAcc * 0.001;
         break;
         case 0x13:
         outFloat.num = imu.velX;
         break;
         case 0x14:
         outFloat.num = imu.accelBiasX;
         break;
         case 0x15:
         outFloat.num = imu.accelBiasY;
         break;
         case 0x16:
         outFloat.num = imu.accelBiasZ;
         break;
         case 0x17:
         outFloat.num = imu.inertialX;
         break;
         case 0x18:
         outFloat.num = imu.inertialY;
         break;
         case 0x19:
         outFloat.num = imu.inertialZ;
         break;
         case 0x1A:
         outFloat.num = 0;
         break;
         
         default:
         break;
         
         }   */        /*
      for (uint8_t i = 0; i < numOfItems; i++){//comp filter tuning
         switch (itemBuffer[i]){
         case 0x00:
         outFloat.num = millis();
         break;
         case 0x01:
         outFloat.num = radianGyroX;
         //outFloat.num = (float)acc.v.x;
         break;
         case 0x02:
         outFloat.num = radianGyroY;
         //outFloat.num = (float)acc.v.y;
         break;
         case 0x03:
         outFloat.num = radianGyroZ;
         //outFloat.num = (float)acc.v.z;
         break;
         case 0x04:
         outFloat.num = scaledAccX;
         break;
         case 0x05:
         outFloat.num = scaledAccY;
         break;
         case 0x06:
         outFloat.num = scaledAccZ;
         break;
         case 0x07:
         outFloat.num = floatMagX;
         //outFloat.num = smoothAccX;
         break;
         case 0x08:
         outFloat.num = floatMagY;
         //outFloat.num = smoothAccY;
         break;
         case 0x09:
         outFloat.num = floatMagZ;
         //outFloat.num = smoothAccZ;
         break;
         case 0x0A:
         outFloat.num = imu.pitch;
         break;
         case 0x0B:
         outFloat.num = imu.roll;
         break;
         case 0x0C:
         outFloat.num = imu.yaw;
         break;
         case 0x0D:
         outFloat.num = imu.inertialX;
         break;
         case 0x0E:
         outFloat.num = imu.inertialY;
         break;
         case 0x0F:
         outFloat.num = imu.inertialZ;
         break;
         case 0x10:
         outFloat.num = imu.feedBack;
         break;
         case 0x11:
         outFloat.num = imu.accelBiasX;
         break;
         case 0x12:
         outFloat.num = imu.accelBiasY;
         break;
         case 0x13:
         outFloat.num = imu.accelBiasZ;
         break;
         case 0x14:
         outFloat.num = imu.XEst;
         break;
         case 0x15:
         outFloat.num = velSetPointX;
         break;
         case 0x16:
         outFloat.num = imu.velX;
         break;
         case 0x17:
         outFloat.num = yTarget;
         break;
         case 0x18:
         outFloat.num = imu.YEst;
         break;
         case 0x19:
         outFloat.num = velSetPointY;
         break;
         case 0x1A:
         outFloat.num = imu.velY;
         break;
         
         default:
         break;
         
         }*/
        /*for (uint8_t i = 0; i < numOfItems; i++){
         switch (itemBuffer[i]){
         case 0x00:
         outFloat.num = millis();
         break;
         case 0x01:
         outFloat.num = pitchSetPoint;
         break;
         case 0x02:
         outFloat.num = imu.pitch;
         break;
         case 0x03:
         outFloat.num = rateSetPointY;
         break;
         case 0x04:
         outFloat.num = degreeGyroY;
         break;
         case 0x05:
         outFloat.num = adjustmentY;
         break;
         case 0x06:
         outFloat.num = rollSetPoint;
         break;
         case 0x07:
         outFloat.num = imu.roll;
         break;
         case 0x08:
         outFloat.num = rateSetPointX;
         break;
         case 0x09:
         outFloat.num = degreeGyroX;
         break;
         case 0x0A:
         outFloat.num = adjustmentX;
         break;
         case 0x0B:
         outFloat.num = yawSetPoint;
         break;
         case 0x0C:
         outFloat.num = imu.yaw;
         break;
         case 0x0D:
         outFloat.num = rateSetPointZ;
         break;
         case 0x0E:
         outFloat.num = degreeGyroZ;
         break;
         case 0x0F:
         outFloat.num = adjustmentZ;
         break;
         case 0x10:
         outFloat.num = imu.roll;
         break;
         case 0x11:
         outFloat.num = imu.yaw;
         break;
         case 0x12:
         outFloat.num = imu.inertialZ;
         break;
         case 0x13:
         outFloat.num = xTarget;
         break;
         case 0x14:
         outFloat.num = imu.XEst;
         break;
         case 0x15:
         outFloat.num = velSetPointX;
         break;
         case 0x16:
         outFloat.num = imu.velX;
         break;
         case 0x17:
         outFloat.num = yTarget;
         break;
         case 0x18:
         outFloat.num = imu.YEst;
         break;
         case 0x19:
         outFloat.num = velSetPointY;
         break;
         case 0x1A:
         outFloat.num = imu.velY;
         break;
         
         default:
         break;
         
         }*/
        /* 
         for (uint8_t i = 0; i < numOfItems; i++){
         switch (itemBuffer[i]){
         case 0x00:
         outFloat.num = millis();
         break;
         case 0x01:
         outFloat.num = imu.inertialX;
         break;
         case 0x02:
         outFloat.num = imu.inertialY;
         break;
         case 0x03:
         outFloat.num = imu.accelBiasX;
         break;
         case 0x04:
         outFloat.num = imu.accelBiasY;
         break;
         case 0x05:
         outFloat.num = imu.velX;
         break;
         case 0x06:
         outFloat.num = imu.velY;
         break;
         case 0x07:
         outFloat.num = imu.XEst;
         break;
         case 0x08:
         outFloat.num = imu.YEst;
         break;
         case 0x09:
         outFloat.num = rawX;
         break;
         case 0x0A:
         outFloat.num = rawY;
         break;
         case 0x0B:
         outFloat.num = xTarget;
         break;
         case 0x0C:
         outFloat.num = velSetPointX;
         break;
         case 0x0D:
         outFloat.num = setPointX;
         break;
         case 0x0E:
         outFloat.num = yTarget;
         break;
         case 0x0F:
         outFloat.num = velSetPointY;
         break;
         case 0x10:
         outFloat.num = setPointY;
         break;
         case 0x11:
         outFloat.num = imu.pitch;
         break;
         case 0x12:
         outFloat.num = imu.roll;
         break;
         case 0x13:
         outFloat.num = imu.yaw;
         break;
         case 0x14:
         outFloat.num = (gps.data.vars.hAcc * 0.001);
         break;
         case 0x15:
         outFloat.num = (float)GPSFlag;
         if (GPSFlag == true){
         GPSFlag = false;
         }
         break;
         case 0x16:
         outFloat.num = 0;
         break;
         case 0x17:
         outFloat.num = 0;
         break;
         case 0x18:
         outFloat.num = 0;
         break;
         case 0x19:
         outFloat.num = 0;
         break;
         case 0x1A:
         outFloat.num = 0;
         break;
         
         default:
         break;
         
         }*/
        /*
      for (uint8_t i = 0; i < numOfItems; i++){
         switch (itemBuffer[i]){
         case 0x00:
         outFloat.num = millis();
         break;
         case 0x01:
         outFloat.num = imu.inertialZ;
         //outFloat.num = rawZ;
         break;
         case 0x02:
         outFloat.num = imu.accelBiasZ;
         //outFloat.num = smoothAlt;
         break;
         case 0x03:
         outFloat.num = imu.velZ;
         break;
         case 0x04:
         outFloat.num = imu.ZEst;
         break;
         case 0x05:
         outFloat.num = smoothAlt;
         break;
         case 0x06:
         outFloat.num = targetAltitude;
         break;
         case 0x07:
         outFloat.num = actualAltitude;
         break;
         case 0x08:
         outFloat.num = targetVelAlt;
         break;
         case 0x09:
         outFloat.num = throttleAdjustment;
         break;
         case 0x0A:
         outFloat.num = imu.pitch;
         break;
         case 0x0B:
         outFloat.num = imu.roll;
         break;
         case 0x0C:
         outFloat.num = imu.yaw;
         break;
         case 0x0D:
         outFloat.num = baroFlag;
         if (baroFlag == true){
         baroFlag = false;
         }
         
         break;
         case 0x0E:
         outFloat.num = rawZ;
         break;
         case 0x0F:
         outFloat.num = imu.yaw;
         break;
         case 0x10:
         outFloat.num = targetVelAlt;
         break;
         case 0x11:
         outFloat.num = throttleAdjustment;
         break;
         case 0x12:
         outFloat.num = imu.inertialZ;
         break;
         case 0x13:
         outFloat.num = xTarget;
         break;
         case 0x14:
         outFloat.num = imu.XEst;
         break;
         case 0x15:
         outFloat.num = velSetPointX;
         break;
         case 0x16:
         outFloat.num = imu.velX;
         break;
         case 0x17:
         outFloat.num = yTarget;
         break;
         case 0x18:
         outFloat.num = imu.YEst;
         break;
         case 0x19:
         outFloat.num = velSetPointY;
         break;
         case 0x1A:
         outFloat.num = imu.velY;
         break;
         
         default:
         break;
         
         }*/


        radio.write(itemBuffer[i]);
        outputSum += itemBuffer[i];
        outputDoubleSum += outputSum;
        for(uint8_t j = 0; j < 4; j++){
          radio.write(outFloat.buffer[j]);
          outputSum += outFloat.buffer[j];
          outputDoubleSum += outputSum;
        }
      }
      radio.write(outputSum);
      radio.write(outputDoubleSum);
    }

  }


}



void SendEndOfWPCheck(){
  outputSum = 0;
  outputDoubleSum = 0;
  radio.write(0xAA);
  radio.write(0x01);
  radio.write(0xE7);
  radio.write(0xE7);
  radio.write(0xE7);
}
void HandShake(){

  radio.begin(115200);
  radioTimer = millis();  
  if (EEPROM.read(0x3E8) == 0xAA){
    EEPROM.write(0x3E8,0xFF);
    packetTemp[0] = EEPROM.read(0x3E9);//lsb for packetNumberLocalOrdered
    packetTemp[1] = EEPROM.read(0x3EA);//msb for packetNumberLocalOrdered
    packetNumberLocalOrdered = (packetTemp[1] << 8) | packetTemp[0];
    packetTemp[0] = EEPROM.read(0x3EB);//lsb for packetNumberLocalOrdered
    packetTemp[1] = EEPROM.read(0x3EC);//msb for packetNumberLocalOrdered
    packetNumberLocalUn = (packetTemp[1] << 8) | packetTemp[0];
    handShake = true;
    return;
  }


  while(radio.available() > 0){
    radio.read();//clear any data in the buffer
  }
  radioTimer = millis();
  while(millis() - radioTimer < 2000 &&  handShake == false){
    //look for data on the radio port
    if(radio.available() > 0){
      handShake = true;
    }
  }
  if(handShake == false){
    return;
  }
  handShake = false;
  cmdState = 0;
  outputSum = 0;
  outputDoubleSum = 0;
  while(millis() - radioTimer < 2000 && handShake == false){
    //complete handshake
    if (radio.available() > 0){
      while (radio.available() > 0){
        inByteRadio = radio.read();
        switch (cmdState){
        case 0:
          outputSum = 0;
          outputDoubleSum = 0;
          if (inByteRadio == 0xAA){
            cmdState = 1;
          }
          break;
        case 1:
          if (inByteRadio == 0x02){//len will always be 2 for the HS
            cmdState = 2;
          }
          else{
            cmdState = 0;
          }
          break;
        case 2:
          if (inByteRadio == 0xFF){
            outputSum += inByteRadio;
            outputDoubleSum += outputSum;
            cmdState = 3;
          }
          else{
            cmdState = 0;
          }
          break;
        case 3:
          if (inByteRadio == 0x01){
            outputSum += inByteRadio;
            outputDoubleSum += outputSum;
            cmdState = 4;
            break;
          }
          if (inByteRadio == 0x00){
            outputDoubleSum += outputSum;
            cmdState = 7;
            break;
          }
          cmdState = 0;

          break;
        case 4://calibration HS 
          if (inByteRadio == outputSum){
            cmdState = 5;
          }
          else{
            cmdState =0;
          }
          break;
        case 5:
          if (inByteRadio == outputDoubleSum){
            cmdState = 6;
          }
          else{
            cmdState =0;
            break;
          }
        case 6://respond with HS and set calibration mode
          //repeate HS?
          outputSum = 0;
          outputDoubleSum = 0;
          radio.write(0xAA);
          radio.write(0x02);
          radio.write(0xFE);
          outputSum += 0xFE;
          outputDoubleSum += outputSum;
          radio.write(0x01);
          outputSum += 0x01;
          outputDoubleSum += outputSum;
          radio.write(outputSum);
          radio.write(outputDoubleSum);
          for (uint8_t i=0; i < 15; i++){
            radio.write(0xAA);
            radio.write(0x02);
            radio.write(0xFE);
            radio.write(0x01);
            radio.write(outputSum);
            radio.write(outputDoubleSum);
          }
          calibrationMode = true;
          handShake = true;
          break;
        case 7://calibration HS 
          if (inByteRadio == outputSum){
            cmdState = 8;
          }
          else{
            cmdState =0;
          }
          break;
        case 8:
          if (inByteRadio == outputDoubleSum){
            cmdState = 9;
          }
          else{
            cmdState =0;
            break;
          }
        case 9:
          handShake = true;
          outputSum = 0;
          outputDoubleSum = 0;
          radio.write(0xAA);
          radio.write(0x02);
          radio.write(0xFE);
          outputSum += 0xFE;
          outputDoubleSum += outputSum;
          radio.write(NUM_WAY_POINTS);
          outputSum += NUM_WAY_POINTS;
          outputDoubleSum += outputSum;
          radio.write(outputSum);
          radio.write(outputDoubleSum);
          for(uint8_t i = 0; i < 15; i++){
            radio.write(0xAA);
            radio.write(0x02);
            radio.write(0xFE);
            radio.write(NUM_WAY_POINTS);
            radio.write(outputSum);
            radio.write(outputDoubleSum);
          }
          break;
        }
      }  
    }
  }


}
void Radio(){

  while (newRadio == true){
    switch (cmdState){      
    case 0://check for start byte
      if (inBuffer[parseIndex] == 0xAA){
        inputSum = 0;
        inputDoubleSum = 0;
        cmdIndex = 0;   
        cmdState = 1;
      }
      break;
    case 1://get the packet length
      numBytesIn = inBuffer[parseIndex];
      cmdState = 2;
      inputSum = 0;
      inputDoubleSum = 0;
      break;
    case 2://
      inputSum += inBuffer[parseIndex];
      inputDoubleSum += inputSum;
      if (inBuffer[parseIndex] == 0xFD){
        ordered = true;
        cmdState = 6;
        cmdIndex = 0;
        break;
      }
      if (inBuffer[parseIndex] == 0xFA){
        ordered = false;
        cmdState = 11;
        cmdIndex = 0;
        break;

      }
      else{
        if (numBytesIn > 1){//currently the most bytes from unreliable will be 1
          cmdState = 0;
          break;
        }
        ordered = false;
        cmdIndex = 0;
        cmdBuffer[cmdIndex] = inBuffer[parseIndex];
        cmdIndex++;
        //cmdState = 3;//this code is here in case we want to expand the un-ordered un-reliable data link
        cmdState = 4;
        break;
      }

      break;
    case 3:
      cmdBuffer[cmdIndex] = inBuffer[parseIndex];
      inputSum += inBuffer[parseIndex];
      inputDoubleSum += inputSum;
      cmdIndex++;
      if (cmdIndex == numBytesIn){
        cmdState = 4;
      }
      break;
    case 4://get the sums
      outputSum = inBuffer[parseIndex];
      cmdState = 5;
      break;
    case 5:
      outputDoubleSum = inBuffer[parseIndex];
      if (inputSum == outputSum && inputDoubleSum == outputDoubleSum){
        ParseCommand();
        outputSum = 0;
        outputDoubleSum = 0;
        switch(cmdBuffer[0]){
        case 0x4C:
          //call to ypr and 2d speed
          YPR2D();
          break;
        case 0x4D:
          //call to lat lon baro alt gps alt
          GPSDat();
          break;
        default:
          SendData(varIndex,numBytesOut,numBytesIn);
          break;
        }
        //SendData(varIndex,numBytesOut,numBytesIn);        
      }      
      cmdState = 0;
      break;
    case 6://reliable 
      packetTemp[0] = inBuffer[parseIndex];
      inputSum += inBuffer[parseIndex];
      inputDoubleSum += inputSum;
      cmdState = 7;
      break;
    case 7:
      packetTemp[1] = inBuffer[parseIndex];
      inputSum += inBuffer[parseIndex];
      inputDoubleSum += inputSum;
      packetNumberRemoteOrdered = (packetTemp[1] << 8) | packetTemp[0];
      if (packetNumberRemoteOrdered != packetNumberLocalOrdered){
        outputSum = 0;
        outputDoubleSum = 0;
        SendMisOrdered();

        cmdState = 0;
        break;
      }
      cmdState = 8;
      break;
    case 8:
      inputSum += inBuffer[parseIndex];
      inputDoubleSum += inputSum;
      cmdBuffer[cmdIndex] = inBuffer[parseIndex];
      cmdIndex++;
      if (cmdIndex == (numBytesIn - 3)){
        cmdState = 9;
      }
      break;
    case 9://check the sums
      outputSum = inBuffer[parseIndex];
      cmdState = 10;
      break;
    case 10:
      outputDoubleSum = inBuffer[parseIndex];
      if (inputSum == outputSum && inputDoubleSum == outputDoubleSum){
        outputSum = 0;
        outputDoubleSum = 0;
        if (cmdBuffer[0] <= 0x4B){
          ParseCommand();
          memcpy(&d.buffer[varIndex],&cmdBuffer[1],(numBytesIn - 4));
          //EEPROM write
          for (uint8_t i = 0; i < 4; i++){
            EEPROM.write( ((varIndex + i) + 45) ,d.buffer[(varIndex + i)]);
          }
          //EEPROM flag set
          if (allCalibrated == false){
            calibrationFlags = EEPROM.read(0x00);
            calibrationFlags |= (1<<GAIN_CAL);
            calibrationFlags &= ~(1<<GAIN_DEF);
            calibrationFlags &= ~(1<<ALL_DEF);
            if ( calibrationFlags == 0x07){
              calibrationFlags |= (1<<ALL_CAL);
              allCalibrated = true;
            }
            EEPROM.write(0x00,calibrationFlags);
          }
          SendAckOrdered();
        }
        else{
          switch (cmdBuffer[0]){
          case 0x65:
            TuningHandler();
            SendAckOrdered();
            break;
          case 0xFF:
            SendAckOrdered();
            if (calibrationMode == true){
              //parse acc cal data
              AccCalHandler();
            }
            break;
          case 0xFE:
            SendAckOrdered();
            if (calibrationMode == true){
              //parse mag cal data
              MagCalHandler();
            }
            break;
          case 0xFD://reset after calibration command
            SendAckOrdered();
            packetNumberLocalOrdered++;
            if (calibrationMode == true){
              EEPROM.write(0x3E8,0xAA);//set handshake compelte flag in EEPROM
              //save the packet numbers
              temp = packetNumberLocalOrdered & 0x00FF;
              EEPROM.write(0x3E9,temp);
              temp = packetNumberLocalOrdered >> 8;
              EEPROM.write(0x3EA,temp);

              temp = packetNumberLocalUn & 0x00FF;
              EEPROM.write(0x3EB,temp);
              temp = packetNumberLocalUn >> 8;
              EEPROM.write(0x3EC,temp);
              asm volatile ("  jmp 0"); 
              //stop and reset command
            }
            break;
          default:
            WayPointHandler();
            break;
          }
        }
        packetNumberLocalOrdered++;
      }
      cmdState = 0;
      break;  
    case 11:
      packetTemp[0] = inBuffer[parseIndex];
      inputSum += inBuffer[parseIndex];
      inputDoubleSum += inputSum;
      cmdState = 12;
      break;
    case 12:
      packetTemp[1] = inBuffer[parseIndex];
      inputSum += inBuffer[parseIndex];
      inputDoubleSum += inputSum;
      packetNumberRemoteUn = (packetTemp[1] << 8) | packetTemp[0];
      if (packetNumberRemoteUn > packetNumberLocalUn){
        if ((packetNumberRemoteUn - packetNumberLocalUn) > 1000){
          cmdState = 13;
          break;
        }
        SendMisUn();
        cmdState = 0;
        break;
      }
      if (packetNumberRemoteUn == packetNumberLocalUn){
        cmdState = 13;
        break;
      }
      if (packetNumberRemoteUn < packetNumberLocalUn){
        if ((packetNumberLocalUn - packetNumberRemoteUn) > 1000){
          SendMisUn();
          cmdState = 0;
          break;
        }
        cmdState = 13;
      }
      break;
    case 13:
      inputSum += inBuffer[parseIndex];
      inputDoubleSum += inputSum;
      cmdBuffer[cmdIndex] = inBuffer[parseIndex];
      cmdIndex++;
      if (cmdIndex == (numBytesIn - 3)){
        cmdState = 14;
      }
      break;
    case 14://check the sums
      outputSum = inBuffer[parseIndex];
      cmdState = 15;
      break;
    case 15:
      outputDoubleSum = inBuffer[parseIndex];
      if (inputSum == outputSum && inputDoubleSum == outputDoubleSum){
        outputSum = 0;
        outputDoubleSum = 0;
        outputSum = 0;
        outputDoubleSum = 0;
        if (cmdBuffer[0] <= 0x4B){
          //to do change name for this function call
          ParseCommand();
          SendDataReliable(varIndex,numBytesOut,(numBytesIn - 4));
          if (allCalibrated == false){
            calibrationFlags = EEPROM.read(0x00);
            calibrationFlags |= (1<<GAIN_CAL);
            calibrationFlags &= ~(1<<GAIN_DEF);
            calibrationFlags &= ~(1<<ALL_DEF);
            if ( calibrationFlags == 0x07){
              calibrationFlags |= (1<<ALL_CAL);
              allCalibrated = true;
            }
            EEPROM.write(0x00,calibrationFlags);
          }
        }
        else{
          WayPointHandler();
        }
        if (packetNumberRemoteUn == packetNumberLocalUn){
          packetNumberLocalUn++;
        }
      }  
      cmdState = 0;
      break;  
    default:
      cmdState = 0;//should not be here ever
      break;
    }
    parseIndex++;
    if (parseIndex == RADIO_BUF_SIZE){
      parseIndex = 0;
    }
    if (parseIndex == bufferIndexRadio){
      newRadio = false;
    }
  }
}

void GPSDat(){
  radio.write(0xAA);
  radio.write(0x11);
  //radio.write(0x14);
  /*radio.write(0xF9);
   outputSum += 0xF9;
   outputDoubleSum += outputSum;
   temp = packetNumberRemoteUn & 0x00FF;
   radio.write(temp);
   outputSum += temp;
   outputDoubleSum += outputSum;
   temp = (packetNumberRemoteUn >> 8) & 0x00FF;
   radio.write(temp);
   outputSum += temp;
   outputDoubleSum += outputSum;*/
  radio.write(cmdBuffer[0]);
  outputSum += cmdBuffer[0];
  outputDoubleSum += outputSum;
  for(uint8_t i =12; i< 28;i++){
    radio.write(d.buffer[i]);
    outputSum += d.buffer[i];
    outputDoubleSum += outputSum;
  }
  radio.write(outputSum);
  radio.write(outputDoubleSum);

}

void YPR2D(){
  radio.write(0xAA);
  radio.write(0x11);
  //radio.write(0x14);//these lines are for reliable unordered
  /*radio.write(0xF9);
   outputSum += 0xF9;
   outputDoubleSum += outputSum;
   temp = packetNumberRemoteUn & 0x00FF;
   radio.write(temp);
   outputSum += temp;
   outputDoubleSum += outputSum;
   temp = (packetNumberRemoteUn >> 8) & 0x00FF;
   radio.write(temp);
   outputSum += temp;
   outputDoubleSum += outputSum;*/
  radio.write(cmdBuffer[0]);
  outputSum += cmdBuffer[0];
  outputDoubleSum += outputSum;
  for(uint8_t i =0; i< 12;i++){
    radio.write(d.buffer[i]);
    outputSum += d.buffer[i];
    outputDoubleSum += outputSum;
  }
  for(uint8_t i = 48; i < 52; i ++){
    radio.write(d.buffer[i]);
    outputSum += d.buffer[i];
    outputDoubleSum += outputSum;
  } 
  radio.write(outputSum);
  radio.write(outputDoubleSum);

}

void SendDataReliable(uint16_t index, uint8_t numOut, uint8_t numIn){
  if (numIn == 4){
    memcpy(&d.buffer[index],&cmdBuffer[1],numIn);
  }

  radio.write(0xAA);
  radio.write(numOut + 4);
  radio.write(0xF9);
  outputSum += 0xF9;
  outputDoubleSum += outputSum;
  temp = packetNumberRemoteUn & 0x00FF;
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum += outputSum;
  temp = (packetNumberRemoteUn >> 8) & 0x00FF;
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum += outputSum;
  radio.write(cmdBuffer[0]);
  outputSum += cmdBuffer[0];
  outputDoubleSum += outputSum;
  for (uint8_t j = 0;j < numOut; j++){
    radio.write(d.buffer[index]);
    outputSum += d.buffer[index];
    outputDoubleSum += outputSum;
    index++;
  }
  radio.write(outputSum);
  radio.write(outputDoubleSum);

}

void AccCalHandler(){
  for (uint8_t i = 0x01; i < 0x31; i++){
    EEPROM.write( (i), cmdBuffer[i]);
  }
  calibrationFlags = EEPROM.read(0x00);
  calibrationFlags |= (1<<ACC_CAL);
  calibrationFlags &= ~(1<<ACC_DEF);
  calibrationFlags &= ~(1<<ALL_DEF);
  EEPROM.write(0x00,calibrationFlags);

}
void MagCalHandler(){
  for (uint8_t i = 0x01; i < 0x31; i++){
    EEPROM.write( (i + 0x30), cmdBuffer[i] );
  }
  calibrationFlags = EEPROM.read(0x00);
  calibrationFlags |= (1<<COMP_CAL);
  calibrationFlags &= ~(1<<COMP_DEF);
  calibrationFlags &= ~(1<<ALL_DEF);
  EEPROM.write(0x00,calibrationFlags);

}

void TuningHandler(){
  if (cmdBuffer[1] == 0 || cmdBuffer[2] == 0){
    tuningTransmit = false;
    return;
  }
  else{
    refreshRate = cmdBuffer[1];
  }
  numOfItems = cmdBuffer[2];
  for (uint8_t i = 0; i < numOfItems; i++){
    itemBuffer[i] = cmdBuffer[i+3];
  }
  if (refreshRate > 100){
    refreshMillis = 0;
  }
  else{
    refreshMillis = uint32_t((1.0/refreshRate)*1000);
  }
  tuningTimer = millis();
  tuningTransmit = true;
}



void SendMisUn(){
  outputSum = 0;
  outputDoubleSum = 0;
  radio.write(0xAA);
  radio.write(0x03);
  radio.write(0xF8);
  outputSum += 0xF8;
  outputDoubleSum += outputSum;
  temp = packetNumberLocalUn & 0x00FF;
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum += outputSum;
  temp = (packetNumberLocalUn >> 8) & 0x00FF;
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum += outputSum;
  radio.write(outputSum);
  radio.write(outputDoubleSum);
}
void SendAckOrdered(){
  outputSum = 0;
  outputDoubleSum = 0;
  radio.write(0xAA);
  radio.write(0x03);
  radio.write(0xFC);
  outputSum += 0xFC;
  outputDoubleSum += outputSum;
  temp = packetNumberLocalOrdered & 0x00FF;
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum += outputSum;
  temp = (packetNumberLocalOrdered >> 8) & 0x00FF;
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum += outputSum;
  radio.write(outputSum);
  radio.write(outputDoubleSum);

}
void SendMisOrdered(){
  outputSum = 0;
  outputDoubleSum = 0;
  radio.write(0xAA);
  radio.write(0x03);
  radio.write(0xFB);
  outputSum += 0xFB;
  outputDoubleSum += outputSum;
  temp = packetNumberLocalOrdered & 0x00FF;
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum += outputSum;
  temp = (packetNumberLocalOrdered >> 8) & 0x00FF;
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum += outputSum;
  radio.write(outputSum);
  radio.write(outputDoubleSum);


}
void SendData(uint16_t index, uint8_t numOut, uint8_t numIn){
  if (numIn == 5){
    memcpy(&d.buffer[index],&cmdBuffer[1],(numIn - 1));
  }
  radio.write(0xAA);
  radio.write((numOut +1));
  radio.write(cmdBuffer[0]);
  outputSum += cmdBuffer[0];
  outputDoubleSum += outputSum;
  for (uint8_t j = 0;j < numOut; j++){
    radio.write(d.buffer[index]);
    outputSum += d.buffer[index];
    outputDoubleSum += outputSum;
    index++;
  }
  radio.write(outputSum);
  radio.write(outputDoubleSum);
}



void ParseCommand(){
  numBytesOut = 4;
  switch (cmdBuffer[0]){
  case 0x00:
    varIndex = 0;
    break;
  case 0x01:
    varIndex = 4;
    break;
  case 0x02:
    varIndex = 8;
    break;
  case 0x03:
    varIndex = 12;
    break;
  case 0x04:
    varIndex = 16;
    break;
  case 0x05:
    varIndex = 20;
    break;
  case 0x06:
    varIndex = 24;
    break;
  case 0x07:
    varIndex = 28;
    break;
  case 0x08:
    varIndex = 32;
    break;
  case 0x09:
    varIndex = 36;
    break;
  case 0x0A:
    varIndex = 40;
    break;
  case 0x0B:
    varIndex = 44;
    break;
  case 0x0C:
    varIndex = 48;
    break;
  case 0x0D:
    varIndex = 296;
    numBytesOut = 1;
    break;
  case 0x0E:
    varIndex = 297;
    numBytesOut = 1;
    break;
  case 0x0F:
    varIndex = 52;
    break;
  case 0x10:
    varIndex = 56;
    break;
  case 0x11:
    varIndex = 60;
    break;
  case 0x12:
    varIndex = 64;
    break;
  case 0x13:
    varIndex = 68;
    break;
  case 0x14:
    varIndex = 72;
    break;
  case 0x15:
    varIndex = 76;
    break;
  case 0x16:
    varIndex = 80;
    break;
  case 0x17:
    varIndex = 84;
    break;
  case 0x18:
    varIndex = 88;
    break;
  case 0x19:
    varIndex = 92;
    break;
  case 0x1A:
    varIndex = 96;
    break;
  case 0x1B:
    varIndex = 100;
    break;
  case 0x1C:
    varIndex = 104;
    break;
  case 0x1D:
    varIndex = 108;
    break;
  case 0x1E:
    varIndex = 112;
    break;
  case 0x1F:
    varIndex = 116;
    break;
  case 0x20:
    varIndex = 120;
    break;
  case 0x21:
    varIndex = 124;
    break;
  case 0x22:
    varIndex = 128;
    break;
  case 0x23:
    varIndex = 132;
    break;
  case 0x24:
    varIndex = 136;
    break;
  case 0x25:
    varIndex = 140;
    break;
  case 0x26:
    varIndex = 144;
    break;
  case 0x27:
    varIndex = 148;
    break;
  case 0x28:
    varIndex = 152;
    break;
  case 0x29:
    varIndex = 156;
    break;
  case 0x2A:
    varIndex = 160;
    break;
  case 0x2B:
    varIndex = 164;
    break;
  case 0x2C:
    varIndex = 168;
    break;
  case 0x2D:
    varIndex = 172;
    break;
  case 0x2E:
    varIndex = 176;
    break;
  case 0x2F:
    varIndex = 180;
    break;
  case 0x30:
    varIndex = 184;
    break;
  case 0x31:
    varIndex = 188;
    break;
  case 0x32:
    varIndex = 192;
    break;
  case 0x33:
    varIndex = 196;
    break;
  case 0x34:
    varIndex = 200;
    break;
  case 0x35:
    varIndex = 204;
    break;
  case 0x36:
    varIndex = 208;
    break;
  case 0x37:
    varIndex = 212;
    break;
  case 0x38:
    varIndex = 216;
    break;
  case 0x39:
    varIndex = 220;
    break;
  case 0x3A:
    varIndex = 224;
    break;
  case 0x3B:
    varIndex = 228;
    break;
  case 0x3C:
    varIndex = 232;
    break;
  case 0x3D:
    varIndex = 236;
    break;
  case 0x3E:
    varIndex = 240;    
    break;
  case 0x3F:
    varIndex = 244;    
    break;
  case 0x40:
    varIndex = 248;    
    break;
  case 0x41:
    varIndex = 252;    
    break;
  case 0x42:
    varIndex = 256;    
    break;
  case 0x43:
    varIndex = 260;    
    break;
  case 0x44:
    varIndex = 264;    
    break;
  case 0x45:
    varIndex = 268;    
    break;
  case 0x46:
    varIndex = 272;    
    break;
  case 0x47:
    varIndex = 276;    
    break;
  case 0x48:
    varIndex = 280;    
    break;
  case 0x49:
    varIndex = 284;    
    break;
  case 0x4A:
    varIndex = 288;    
    break;   
  case 0x4B:
    varIndex = 292;    
    break;  
  default:
    break;
  }
}

void WayPointHandler(){
  if(d.v.flightMode != WAYPOINT){
    if (ordered == true){
      SendAckOrdered();
    }
    else{
      radio.write(0xAA);
      radio.write(0x05);
      radio.write(0xF9);
      outputSum += 0xF9;
      outputDoubleSum += outputSum;
      temp = packetNumberRemoteUn & 0x00FF;
      radio.write(temp);
      outputSum += temp;
      outputDoubleSum += outputSum;
      temp = (packetNumberRemoteUn >> 8) & 0x00FF;
      radio.write(temp);
      outputSum += temp;
      outputDoubleSum += outputSum;
      radio.write(0xEE);
      outputSum += 0xEE;
      outputDoubleSum += outputSum;
      radio.write(0xFF);
      outputSum += 0xFF;
      outputDoubleSum += outputSum;
      radio.write(outputSum);
      radio.write(outputDoubleSum);
    }
    return;
  }
  switch (cmdBuffer[0]){
  case 0xEF://load up way points
    SendAckOrdered();
    if (inputWayPointNumber != NUM_WAY_POINTS){
      memcpy(&wayPoints[inputWayPointNumber].buffer[0],&cmdBuffer[1],12);
      inputWayPointNumber++;

    }
    break;
  case 0xEE://query waypoint
    if(cmdBuffer[1] >= inputWayPointNumber){
      radio.write(0xAA);
      radio.write(0x05);
      radio.write(0xF9);
      outputSum += 0xF9;
      outputDoubleSum += outputSum;

      temp = packetNumberRemoteUn & 0x00FF;
      radio.write(temp);
      outputSum += temp;
      outputDoubleSum += outputSum;

      temp = (packetNumberRemoteUn >> 8) & 0x00FF;
      radio.write(temp);
      outputSum += temp;
      outputDoubleSum += outputSum;
      radio.write(0xEE);
      outputSum += 0xEE;
      outputDoubleSum +=outputSum;
      radio.write(0xFE);
      outputSum += (uint8_t)0xFE;
      outputDoubleSum += outputSum;
      radio.write(outputSum);
      radio.write(outputDoubleSum);
    }
    else{
      radio.write(0xAA);

      radio.write(0x11);

      radio.write(0xF9);
      outputSum += 0xF9;
      outputDoubleSum += outputSum;

      temp = packetNumberRemoteUn & 0x00FF;
      radio.write(temp);
      outputSum += temp;
      outputDoubleSum += outputSum;

      temp = (packetNumberRemoteUn >> 8) & 0x00FF;
      radio.write(temp);
      outputSum += temp;
      outputDoubleSum += outputSum;

      radio.write(0xEE);
      outputSum += 0xEE;
      outputDoubleSum += outputSum;

      radio.write(cmdBuffer[1]);
      outputSum += cmdBuffer[1];
      outputDoubleSum += outputSum;
      for (int j = 0; j < 12; j ++){
        radio.write(wayPoints[cmdBuffer[1]].buffer[j]);
        outputSum += wayPoints[cmdBuffer[1]].buffer[j];
        outputDoubleSum += outputSum;
      }
      radio.write(outputSum);
      radio.write(outputDoubleSum);
    }

    break;
  case 0xED://go to waypoints
    if (wayPointState == WP_HOLD){
      startFlight = true;
    }
    SendAckOrdered();

    break;
  case 0xEC://query if on course
    if (wayPointState == WP_TRAVEL){
      radio.write(0xAA);
      radio.write(0x05);

      radio.write(0xF9);
      outputSum += 0xF9;
      outputDoubleSum += outputSum;

      temp = packetNumberRemoteUn & 0x00FF;
      radio.write(temp);
      outputSum += temp;
      outputDoubleSum += outputSum;

      temp = (packetNumberRemoteUn >> 8) & 0x00FF;
      radio.write(temp);
      outputSum += temp;
      outputDoubleSum += outputSum;      
      radio.write(0xEC);
      outputSum += 0xEC;
      outputDoubleSum += outputSum;
      radio.write(0x00);
      outputSum += (uint8_t)0x00;
      outputDoubleSum += outputSum;
      radio.write(outputSum);
      radio.write(outputDoubleSum);
    }
    else{

      radio.write(0xAA);
      radio.write(0x11);

      radio.write(0xF9);
      outputSum += 0xF9;
      outputDoubleSum += outputSum;

      temp = packetNumberRemoteUn & 0x00FF;
      radio.write(temp);
      outputSum += temp;
      outputDoubleSum += outputSum;

      temp = (packetNumberRemoteUn >> 8) & 0x00FF;
      radio.write(temp);
      outputSum += temp;
      outputDoubleSum += outputSum;      

      radio.write(0xEC);
      outputSum += 0xEC;
      outputDoubleSum += outputSum;
      radio.write(0x01);
      outputSum += 0x01;
      outputDoubleSum += outputSum;
      loiterWP.coord.lat = d.v.lattitude;
      loiterWP.coord.lon = d.v.longitude;
      loiterWP.coord.alt = d.v.gpsAltitude;
      for (int j =0; j < 12; j++){
        radio.write(loiterWP.buffer[j]);
        outputSum += loiterWP.buffer[j];
        outputDoubleSum += outputSum;
      }
      radio.write(outputSum);
      radio.write(outputDoubleSum);
    }
    break;
  case 0xEB:
    if (inputWayPointNumber == 0){
      radio.write(0xAA);
      radio.write(0x05);
      radio.write(0xF9);
      outputSum += 0xF9;
      outputDoubleSum += outputSum;

      temp = packetNumberRemoteUn & 0x00FF;
      radio.write(temp);
      outputSum += temp;
      outputDoubleSum += outputSum;

      temp = (packetNumberRemoteUn >> 8) & 0x00FF;
      radio.write(temp);
      outputSum += temp;
      outputDoubleSum += outputSum;
      radio.write(0xEB);
      outputSum += 0xEB;
      outputDoubleSum += outputSum;
      radio.write(0xFF);
      outputSum += 0xFF;
      outputDoubleSum += outputSum;
      radio.write(outputSum);
      radio.write(outputDoubleSum);

    }
    else{
      radio.write(0xAA);
      radio.write(0x11);
      radio.write(0xF9);
      outputSum += 0xF9;
      outputDoubleSum += outputSum;

      temp = packetNumberRemoteUn & 0x00FF;
      radio.write(temp);
      outputSum += temp;
      outputDoubleSum += outputSum;

      temp = (packetNumberRemoteUn >> 8) & 0x00FF;
      radio.write(temp);
      outputSum += temp;
      outputDoubleSum += outputSum;
      radio.write(0xEB);
      outputSum += 0xEB;
      outputDoubleSum += outputSum;
      radio.write(currentWayPointNumber);
      outputSum += currentWayPointNumber;
      outputDoubleSum += outputSum;
      for (int j =0; j < 12; j++){
        radio.write(wayPoints[currentWayPointNumber].buffer[j]);
        outputSum += wayPoints[currentWayPointNumber].buffer[j];
        outputDoubleSum += outputSum;
      }
      radio.write(outputSum);
      radio.write(outputDoubleSum);
    }

    break;
  case 0xEA://clear way points and loiter
    //to do
    //get current lat lon alt and set the loiter coords
    targetAltitude = imu.ZEst;
    latTarget = d.v.lattitude;
    lonTarget = d.v.longitude;
    inputWayPointNumber = 0;
    wayPointState = WP_HOLD;
    //set way point mode to HOLD
    SendAckOrdered();
    break;
  case 0xE9://override home GPS location
    memcpy(&homeBase.buffer[0],&cmdBuffer[1],12);
    SendAckOrdered();
    break;
  case 0xE8:
    radio.write(0xAA);
    radio.write(0x10);
    radio.write(0xF9);
    outputSum += 0xF9;
    outputDoubleSum += outputSum;

    temp = packetNumberRemoteUn & 0x00FF;
    radio.write(temp);
    outputSum += temp;
    outputDoubleSum += outputSum;

    temp = (packetNumberRemoteUn >> 8) & 0x00FF;
    radio.write(temp);
    outputSum += temp;
    outputDoubleSum += outputSum;
    radio.write(0xE8);
    outputSum += 0xE8;
    outputDoubleSum += outputSum;
    for (int j =0; j < 12; j++){
      radio.write(homeBase.buffer[j]);
      outputSum += homeBase.buffer[j];
      outputDoubleSum += outputSum;
    }
    radio.write(outputSum);
    radio.write(outputDoubleSum); 
    break;
  case 0xE7:
    SendAckOrdered();
    if (cmdBuffer[1] == 0xFF){
      //to do set loiter flag
      endOfWPCheck = true; 
      break;
    }
    if (cmdBuffer[1] == 0x00){
      RTBFlag = true;
      //to do set non fail safe RTB
      break;
    }
    //set fail safe RTB in event that ff or 00 not recieved
    break;
  case 0xE6:
    break;
  default:
    break;
  }
}







































