void Radio() {
  uint8_t j;
  while (radioStream->available() > 0) { //---

    radioByte = radioStream->read();
    switch (radioState) { //+++
    case 0://check for start byte
      rxSum = 0;
      rxDoubleSum = 0;
      if (radioByte == 0xAA) {
        radioState = 1;
      }
      break;
    case 1:
      packetLength = radioByte;
      numRXbytes = 0;
      radioState = 2;
      break;

    case 2:
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      if (radioByte == 0xFA) { //reliable querie
        radioState = 6;
        break;
      }
      if (radioByte == 0xFD) { //reliable set
        radioState = 12;
        break;
      }
      typeNum = radioByte;
      if (typeNum == 11 && packetLength == 18) {
        radioState = 19;
        break;
      }
      if (packetLength == 2) { //length for unrelaible will always be 2
        radioState = 3;//unrelaible data
      }
      else {
        radioState = 0;
      }
      break;

    case 3://unrelaible
      cmdNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      radioState = 4;
      break;
    case 4://unreliable checksum 1
      if (rxSum == radioByte) {
        radioState = 5;
        break;
      }
      radioState = 0;
      break;
    case 5://unreliable check sum 2
      if (rxDoubleSum == radioByte) {
        UnReliableTransmit();
      }
      radioState = 0;
      break;
    case 6://reliable queries - get packet num LSB
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      packetTemp[0] = radioByte;
      radioState = 7;
      break;
    case 7://packet num MSB and verify
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      packetTemp[1] = radioByte;
      remotePacketNumberUn = (packetTemp[1] << 8 ) | packetTemp[0];
      if (remotePacketNumberUn > localPacketNumberUn) {
        if ( (remotePacketNumberUn - localPacketNumberUn) > 1000) {
          radioState = 8;
          break;
        }
        SendUnMis();
        radioState = 0;
        break;
      }
      if (remotePacketNumberUn == localPacketNumberUn) {
        radioState = 8;
        break;
      }
      if (remotePacketNumberUn < localPacketNumberUn) {
        if ((localPacketNumberUn - remotePacketNumberUn) > 1000) {
          SendUnMis();
          radioState = 0;
        }
        radioState = 8;
      }
      break;
    case 8://get typeNum
      typeNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      radioState = 9;
      break;
    case 9://get cmdNum
      cmdNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      itemIndex = 0;
      radioState = 10;
      break;

    case 10://check the first sum
      if (rxSum == radioByte) {
        radioState = 11;
        break;
      }
      radioState = 0;
      break;

    case 11://check the second sum
      if (rxDoubleSum == radioByte) {
        SendUnAck();
        if (remotePacketNumberUn == localPacketNumberUn) {
          localPacketNumberUn++;
        }
      }
      radioState = 0;

      break;

    case 12://reliable set get packet num lsb

      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      packetTemp[0] = radioByte;
      radioState = 13;
      break;

    case 13://get packet num msb and verify
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      packetTemp[1] = radioByte;
      remotePacketNumberOrdered = (packetTemp[1] << 8 ) | packetTemp[0];
      if (remotePacketNumberOrdered != localPacketNumberOrdered) {
        SendOrdMis();
        radioState = 0;
        break;
      }
      radioState = 14;
      break;

    case 14:
      typeNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      radioState = 15;
      break;

    case 15:
      cmdNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      itemIndex = 0;
      radioState = 16;
      if (typeNum == 6 || typeNum == 8 ) {
        radioState = 17;
      }
      if (typeNum == 7 && cmdNum == 3) {
        radioState = 17;
      }
      break;

    case 16://buffer in data
      itemBuffer[itemIndex++] = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      if (packetLength > 250) {
        radioState = 0;
      }
      if (numRXbytes == packetLength) {
        radioState = 17;
      }
      break;
    case 17://check first sum
      if (rxSum != radioByte) {
        radioState = 0;
        break;
      }
      radioState = 18;
      break;
    case 18:
      if (rxDoubleSum == radioByte) {
        if (calibrationMode == true) {
          if (typeNum == 6) {
            sendCalibrationData = true;
          }
          if (typeNum == 7) {
            WriteCalibrationDataToRom();
            sendCalibrationData = false;
          }
        }
        else {
          if (typeNum < 3) {
            OrderedSet();
          }
          if (typeNum == 4 || typeNum == 5) {
            SetTransmissionRate();
          }
          if (typeNum == 8) {

            imu.pitchOffset.val = imu.rawPitch.val;
            imu.rollOffset.val = imu.rawRoll.val;
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
        }
        SendOrdAck();
      }
      /*else{
       if (remotePacketNumberOrdered != localPacketNumberOrdered){
       SendOrdMis();
       radioState = 0;
       break;
       }
       }*/
      radioState = 0;
      break;
    case 19:
      cmdNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      itemIndex = 0;
      if (cmdNum == 8) {
        radioState = 20;
      }
      else {
        radioState = 0;
      }
      break;
    case 20:
      itemBuffer[itemIndex++] = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      if (itemIndex == (cmdNum * 2)) {
        radioState = 21;
      }
      break;
    case 21:
      if (rxSum == radioByte) {
        radioState = 22;
      }
      else {
        radioState = 0;
      }
      break;
    case 22:
      if (rxDoubleSum == radioByte) {
        HandleGSRCData();

      }
      radioState = 0;
      break;

    }//+++
  }//---

}

void HandleGSRCData() {
  int16_u inShort;
  itemIndex = 0;
  for (uint8_t i = 0; i < cmdNum; i++) {
    switch (i) {
    case THRO:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[THRO] = inShort.val;
      break;
    case AILE:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[AILE] = inShort.val;
      break;
    case ELEV:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[ELEV] = inShort.val;
      break;
    case RUDD:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[RUDD] = inShort.val;
      break;
    case GEAR:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[GEAR] = inShort.val;
      break;
    case AUX1:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[AUX1] = inShort.val;
      break;
    case AUX2:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[AUX2] = inShort.val;
      break;
    case AUX3:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[AUX3] = inShort.val;
      break;
    }
  }
  newGSRC = true;
  groundFSCount = 0;
}

void TuningTransmitter() { //
  uint32_u now;
  now.val = millis();


  if (hsTX == true) { //---

    if (now.val - hsTXTimer >= hsMillis) { //+++
      hsTXTimer = now.val;
      txSum = 0;
      txDoubleSum = 0;
      hsListIndex = 0;
      tuningItemIndex = 0;
      packetLength = 0;
      //assemble the transmit buffer
      liveDataBuffer[tuningItemIndex++] = 0xAA;
      tuningItemIndex++;//skip packet length for now
      liveDataBuffer[tuningItemIndex++] = 4;
      txSum += 4;
      txDoubleSum += txSum;
      packetLength++;

      liveDataBuffer[tuningItemIndex++] = hsRequestNumber;
      txSum += hsRequestNumber;
      txDoubleSum += txSum;
      packetLength++;

      liveDataBuffer[tuningItemIndex++] = hsNumItems;
      txSum += hsNumItems;
      txDoubleSum += txSum;
      packetLength++;

      for (uint8_t i = 0; i < 4; i++) { //always include millis
        liveDataBuffer[tuningItemIndex++] = now.buffer[i];
        txSum += now.buffer[i];
        txDoubleSum += txSum;
        packetLength++;
      }


      for (uint8_t i = 0; i < hsNumItems; i++) { //***
        switch (hsList[hsListIndex++]) {
        case 0://floats
          for (uint8_t j = 0; j < 4; j++) {
            liveDataBuffer[tuningItemIndex++] = (*floatPointerArray[hsList[hsListIndex]]).buffer[j];
            txSum += (*floatPointerArray[hsList[hsListIndex]]).buffer[j];
            txDoubleSum += txSum;
            packetLength++;
          }
          hsListIndex++;
          break;
        case 1://int16
          for (uint8_t j = 0; j < 2; j++) {
            liveDataBuffer[tuningItemIndex++] = (*int16PointerArray[hsList[hsListIndex]]).buffer[j];
            txSum += (*int16PointerArray[hsList[hsListIndex]]).buffer[j];
            txDoubleSum += txSum;
            packetLength++;
          }
          hsListIndex++;
          break;
        case 2://int32
          /*for(uint8_t j = 0; j < 4; j++){
           liveDataBuffer[tuningItemIndex++] = (*int32PointerArray[hsList[hsListIndex]]).buffer[j];
           txSum += (*int32PointerArray[hsList[hsListIndex]]).buffer[j];
           txDoubleSum += txSum;
           packetLength++;
           }
           hsListIndex++;*/
          break;
        case 3:
          liveDataBuffer[tuningItemIndex++] = *bytePointerArray[hsList[hsListIndex]];
          txSum += *bytePointerArray[hsList[hsListIndex]];
          txDoubleSum += txSum;
          packetLength++;

          hsListIndex++;
          break;
        }
      }//***

      liveDataBuffer[1] = packetLength;
      for (uint8_t i = 0; i < (packetLength + 2); i++) {
        radioPrint->write(liveDataBuffer[i]);
      }
      radioPrint->write(txSum);
      radioPrint->write(txDoubleSum);


    }//+++

  }//---

  if (lsTX == true) { //---

    if (now.val - lsTXTimer >= lsMillis) { //+++
      lsTXTimer = now.val;
      txSum = 0;
      txDoubleSum = 0;
      lsListIndex = 0;
      tuningItemIndex = 0;
      packetLength = 0;
      //assemble the transmit buffer
      liveDataBuffer[tuningItemIndex++] = 0xAA;
      tuningItemIndex++;//skip packet length for now
      liveDataBuffer[tuningItemIndex++] = 5;
      txSum += 5;
      txDoubleSum += txSum;
      packetLength++;

      liveDataBuffer[tuningItemIndex++] = lsRequestNumber;
      txSum += lsRequestNumber;
      txDoubleSum += txSum;
      packetLength++;

      liveDataBuffer[tuningItemIndex++] = lsNumItems;
      txSum += lsNumItems;
      txDoubleSum += txSum;
      packetLength++;

      for (uint8_t i = 0; i < 4; i++) { //always include millis
        liveDataBuffer[tuningItemIndex++] = now.buffer[i];
        txSum += now.buffer[i];
        txDoubleSum += txSum;
        packetLength++;
      }


      for (uint8_t i = 0; i < lsNumItems; i++) { //***
        switch (lsList[lsListIndex++]) {
        case 0://floats
          for (uint8_t j = 0; j < 4; j++) {
            liveDataBuffer[tuningItemIndex++] = (*floatPointerArray[lsList[lsListIndex]]).buffer[j];
            txSum += (*floatPointerArray[lsList[lsListIndex]]).buffer[j];
            txDoubleSum += txSum;
            packetLength++;
          }
          lsListIndex++;
          break;
        case 1://int16
          for (uint8_t j = 0; j < 2; j++) {
            liveDataBuffer[tuningItemIndex++] = (*int16PointerArray[lsList[lsListIndex]]).buffer[j];
            txSum += (*int16PointerArray[lsList[lsListIndex]]).buffer[j];
            txDoubleSum += txSum;
            packetLength++;
          }
          lsListIndex++;
          break;
        case 2://int32
          /*for(uint8_t j = 0; j < 4; j++){
           liveDataBuffer[tuningItemIndex++] = (*int32PointerArray[lsList[lsListIndex]]).buffer[j];
           txSum += (*int32PointerArray[lsList[lsListIndex]]).buffer[j];
           txDoubleSum += txSum;
           packetLength++;
           }
           lsListIndex++;*/
          break;
        case 3:
          liveDataBuffer[tuningItemIndex++] = *bytePointerArray[lsList[lsListIndex]];
          txSum += *bytePointerArray[lsList[lsListIndex]];
          txDoubleSum += txSum;
          packetLength++;

          lsListIndex++;
          break;
        default:
          break;
        }
      }//***

      liveDataBuffer[1] = packetLength;
      for (uint8_t i = 0; i < (packetLength + 2); i++) {
        radioPrint->write(liveDataBuffer[i]);
      }
      radioPrint->write(txSum);
      radioPrint->write(txDoubleSum);

    }//+++

  }//---

}//


void SetTransmissionRate() {
  if (typeNum == 4) {
    if (cmdNum == 0) {
      hsTX = false;
    }
    else {
      hsTX = true;
      hsMillis = uint32_t((1.0 / cmdNum) * 1000);
      hsRequestNumber = itemBuffer[0];
      hsNumItems = itemBuffer[1];
      if (hsNumItems > 20) {
        hsTX = false;
      }
      else {
        memcpy( &hsList[0], &itemBuffer[2], (hsNumItems * 2) );
      }
    }



  }
  else {

    if (cmdNum == 0) {
      lsTX = false;
    }
    else {
      lsTX = true;
      lsMillis = uint32_t((1.0 / cmdNum) * 1000);
      lsRequestNumber = itemBuffer[0];
      lsNumItems = itemBuffer[1];
      if (lsNumItems > 20) {
        lsTX = false;
      }
      else {
        memcpy( &lsList[0], &itemBuffer[2], (lsNumItems * 2) );
      }
    }


  }
}

void WriteCalibrationDataToRom() {
  uint8_t temp;
  //int16_u temp16;
  itemIndex = 0;
  switch (cmdNum) {
  case 0://mag calibration data
    if (imu.magDetected == true) {
      for (uint16_t i = MAG_CALIB_START; i <= MAG_CALIB_END; i++) {
        EEPROM.write(i, itemBuffer[itemIndex++]);
      }

      calibrationFlags = EEPROM.read(CAL_FLAGS);
      calibrationFlags &= ~(1 << MAG_FLAG);
      EEPROM.write(CAL_FLAGS, calibrationFlags);
    }
    break;//--------------------------------------------
  case 1://acc calibration data
    for (uint16_t i = ACC_CALIB_START; i <= ACC_CALIB_END; i++) {
      EEPROM.write(i, itemBuffer[itemIndex++]);
    }

    calibrationFlags = EEPROM.read(CAL_FLAGS);
    calibrationFlags &= ~(1 << ACC_FLAG);
    EEPROM.write(CAL_FLAGS, calibrationFlags);


    break;//--------------------------------------------


  case 2://RC calibration data
    for (uint16_t i = RC_DATA_START; i <= RC_DATA_END; i++) {
      EEPROM.write(i, itemBuffer[itemIndex++]);
    }
    calibrationFlags = EEPROM.read(CAL_FLAGS);
    calibrationFlags &= ~(1 << RC_FLAG);
    EEPROM.write(CAL_FLAGS, calibrationFlags);
    break;//--------------------------------------------
  case 3://command to end calibration and reset controller
    //save the packet numbers
    if (USBFlag == true) {
      EEPROM.write(HS_FLAG, 0xBB); //set handshake compelte flag in EEPROM
    }
    else {
      EEPROM.write(HS_FLAG, 0xAA); //set handshake compelte flag in EEPROM
    }
    SendOrdAck();
    //save the packet numbers
    temp = localPacketNumberOrdered & 0x00FF;
    EEPROM.write(PKT_LOCAL_ORD_L, temp);
    temp = localPacketNumberOrdered >> 8;
    EEPROM.write(PKT_LOCAL_ORD_M, temp);

    temp = localPacketNumberUn & 0x00FF;
    EEPROM.write(PKT_LOCAL_UN_L, temp);
    temp = localPacketNumberUn >> 8;
    EEPROM.write(PKT_LOCAL_UN_M, temp);
    Motor1WriteMicros(0);//set the output compare value
    Motor2WriteMicros(0);
    Motor3WriteMicros(0);
    Motor4WriteMicros(0);
    Motor5WriteMicros(0);
    Motor6WriteMicros(0);
    delay(500);
    asm volatile ("  jmp 0");

    break;//--------------------------------------
  case 4://tx failsafe
    txLossRTB = itemBuffer[0];
    if (txLossRTB < 0 ||txLossRTB > 1){
      txLossRTB = 0;
    }

    EEPROM.write(TX_FS_FLAG, 0xAA);
    EEPROM.write(TX_FS, txLossRTB);
    txLossRTB = EEPROM.read(TX_FS);
    break;//--------------------------------------------
  case 5://pwms
    EEPROM.write(HOVER_THRO, itemBuffer[itemIndex++]);
    EEPROM.write(HOVER_THRO_FLAG, 0xAA);
    EEPROM.write(PROP_IDLE, itemBuffer[itemIndex++]);
    EEPROM.write(PROP_IDLE_FLAG, 0xAA);
    EEPROM.write(PWM_LIM_HIGH_START, itemBuffer[itemIndex++]);
    EEPROM.write(PWM_LIM_HIGH_END, itemBuffer[itemIndex++]);
    EEPROM.write(PWM_LIM_LOW_START, itemBuffer[itemIndex++]);
    EEPROM.write(PWM_LIM_LOW_END, itemBuffer[itemIndex++]);
    EEPROM.write(PWM_FLAG, 0xAA);

    break;//--------------------------------------------
  }



}


void OrderedSet() {
  switch (typeNum) {
  case 0:
    if (cmdNum >= KP_PITCH_RATE_ && cmdNum <= MAG_DEC_) {
      for (uint8_t i = 0; i < 4; i++) {
        (*floatPointerArray[cmdNum]).buffer[i] =  itemBuffer[i];
      }
      saveGainsFlag = true;
      romWriteDelayTimer = millis();

    }
    break;
  case 1:
    /*
      for (uint8_t i = 0; i < 2; i++){
     (*int16PointerArray[cmdNum]).buffer[i] =  itemBuffer[i];
     }*/
    break;
  case 2:
    /*
      for (uint8_t i = 0; i < 4; i++){
     (*int32PointerArray[cmdNum]).buffer[i] =  itemBuffer[i];
     }
     */
    break;
  }

}

void SendOrdAck() {
 
  txSum = 0;
  txDoubleSum = 0;
  radioPrint->write(0xAA);
  radioPrint->write(3);
  radioPrint->write(0xFC);
  txSum = 0xFC;
  txDoubleSum += txSum;
  temp = localPacketNumberOrdered & 0x00FF;
  radioPrint->write(temp);
  txSum += temp;
  txDoubleSum += txSum;
  temp = (localPacketNumberOrdered >> 8) & 0x00FF;
  radioPrint->write(temp);
  txSum += temp;
  txDoubleSum += txSum;
  radioPrint->write(txSum);
  radioPrint->write(txDoubleSum);
  localPacketNumberOrdered++;

}

void SendOrdMis() {
  txSum = 0;
  txDoubleSum = 0;
  radioPrint->write(0xAA);
  radioPrint->write(3);
  radioPrint->write(0xFB);
  txSum = 0xFB;
  txDoubleSum += txSum;
  temp = localPacketNumberOrdered & 0x00FF;
  radioPrint->write(temp);
  txSum += temp;
  txDoubleSum += txSum;
  temp = (localPacketNumberOrdered >> 8) & 0x00FF;
  radioPrint->write(temp);
  txSum += temp;
  txDoubleSum += txSum;
  radioPrint->write(txSum);
  radioPrint->write(txDoubleSum);
}

void OrderedQuery() {

  switch (typeNum) {
  case 0:
    for (uint8_t i = 0; i < 4; i++) {
      itemBuffer[i] = (*floatPointerArray[cmdNum]).buffer[i];
    }
    break;
  case 1:
    for (uint8_t i = 0; i < 2; i++) {
      itemBuffer[i] = (*int16PointerArray[cmdNum]).buffer[i];
    }
    break;
  case 2:
    /*for (uint8_t i = 0; i < 4; i++){
     itemBuffer[i] = (*int32PointerArray[cmdNum]).buffer[i];
     }*/
    break;
  case 3:

    break;
  }

}

void SendUnAck() {
  txSum = 0;
  txDoubleSum = 0;
  radioPrint->write(0XAA);

  switch (typeNum) {
  case 0:
    radioPrint->write(9);
    radioPrint->write(0xF9);
    txSum = 0xF9;
    txDoubleSum += txSum;
    temp = remotePacketNumberUn & 0x00FF;
    radioPrint->write(temp);
    txSum += temp;
    txDoubleSum += txSum;
    temp = (remotePacketNumberUn >> 8) & 0x00FF;
    radioPrint->write(temp);
    txSum += temp;
    txDoubleSum += txSum;
    radioPrint->write(typeNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    radioPrint->write(cmdNum);
    txSum += cmdNum;
    txDoubleSum += txSum;
    radioPrint->write((*floatPointerArray[cmdNum]).buffer[0]);
    txSum += (*floatPointerArray[cmdNum]).buffer[0];
    txDoubleSum += txSum;
    radioPrint->write((*floatPointerArray[cmdNum]).buffer[1]);
    txSum += (*floatPointerArray[cmdNum]).buffer[1];
    txDoubleSum += txSum;
    radioPrint->write((*floatPointerArray[cmdNum]).buffer[2]);
    txSum += (*floatPointerArray[cmdNum]).buffer[2];
    txDoubleSum += txSum;
    radioPrint->write((*floatPointerArray[cmdNum]).buffer[3]);
    txSum += (*floatPointerArray[cmdNum]).buffer[3];
    txDoubleSum += txSum;
    radioPrint->write(txSum);
    radioPrint->write(txDoubleSum);

    break;
  case 1:
    radioPrint->write(7);
    radioPrint->write(0xF9);
    txSum += 0xF9;
    txDoubleSum += txSum;
    temp = remotePacketNumberUn & 0x00FF;
    radioPrint->write(temp);
    txSum += temp;
    txDoubleSum += txSum;
    temp = (remotePacketNumberUn >> 8) & 0x00FF;
    radioPrint->write(temp);
    txSum += temp;
    txDoubleSum += txSum;
    radioPrint->write(typeNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    radioPrint->write(cmdNum);
    txSum += cmdNum;
    txDoubleSum += txSum;
    radioPrint->write((*int16PointerArray[cmdNum]).buffer[0]);
    txSum += (*int16PointerArray[cmdNum]).buffer[0];
    txDoubleSum += txSum;
    radioPrint->write((*int16PointerArray[cmdNum]).buffer[1]);
    txSum += (*int16PointerArray[cmdNum]).buffer[1];
    txDoubleSum += txSum;
    radioPrint->write(txSum);
    radioPrint->write(txDoubleSum);

    break;
  case 2:
    /*radioPrint->write(9);
     radioPrint->write(0xF9);
     txSum = 0xF9;
     txDoubleSum += txSum;
     temp = remotePacketNumberUn & 0x00FF;
     radioPrint->write(temp);
     txSum += temp;
     txDoubleSum += txSum;
     temp = (remotePacketNumberUn >> 8) & 0x00FF;
     radioPrint->write(temp);
     txSum += temp;
     txDoubleSum += txSum;
     radioPrint->write(typeNum);
     txSum += typeNum;
     txDoubleSum += txSum;
     radioPrint->write(cmdNum);
     txSum += cmdNum;
     txDoubleSum += txSum;
     radioPrint->write((*int32PointerArray[cmdNum]).buffer[0]);
     txSum += (*int32PointerArray[cmdNum]).buffer[0];
     txDoubleSum += txSum;
     radioPrint->write((*int32PointerArray[cmdNum]).buffer[1]);
     txSum += (*int32PointerArray[cmdNum]).buffer[1];
     txDoubleSum += txSum;
     radioPrint->write((*int32PointerArray[cmdNum]).buffer[2]);
     txSum += (*int32PointerArray[cmdNum]).buffer[2];
     txDoubleSum += txSum;
     radioPrint->write((*int32PointerArray[cmdNum]).buffer[3]);
     txSum += (*int32PointerArray[cmdNum]).buffer[3];
     txDoubleSum += txSum;
     radioPrint->write(txSum);
     radioPrint->write(txDoubleSum);*/

    break;
  case 3:
    radioPrint->write(6);
    radioPrint->write(0xF9);
    txSum += 0xF9;
    txDoubleSum += txSum;
    temp = remotePacketNumberUn & 0x00FF;
    radioPrint->write(temp);
    txSum += temp;
    txDoubleSum += txSum;
    temp = (remotePacketNumberUn >> 8) & 0x00FF;
    radioPrint->write(temp);
    txSum += temp;
    txDoubleSum += txSum;
    radioPrint->write(typeNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    radioPrint->write(cmdNum);
    txSum += cmdNum;
    txDoubleSum += txSum;
    radioPrint->write(*bytePointerArray[cmdNum]);
    txSum += *bytePointerArray[cmdNum];
    txDoubleSum += txSum;

    radioPrint->write(txSum);
    radioPrint->write(txDoubleSum);
    break;
  }



  if (typeNum == 1) {
    radioPrint->write(7);
    radioPrint->write(0xF9);
    txSum += 0xF9;
    txDoubleSum += txSum;
    temp = remotePacketNumberUn & 0x00FF;
    radioPrint->write(temp);
    txSum += temp;
    txDoubleSum += txSum;
    temp = (remotePacketNumberUn >> 8) & 0x00FF;
    radioPrint->write(temp);
    txSum += temp;
    txDoubleSum += txSum;
    radioPrint->write(typeNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    radioPrint->write(cmdNum);
    txSum += cmdNum;
    txDoubleSum += txSum;
    radioPrint->write(itemBuffer[0]);
    txSum += itemBuffer[0];
    txDoubleSum += txSum;
    radioPrint->write(itemBuffer[1]);
    txSum += itemBuffer[1];
    txDoubleSum += txSum;
    radioPrint->write(txSum);
    radioPrint->write(txDoubleSum);
  }
  else {
    radioPrint->write(9);
    radioPrint->write(0xF9);
    txSum = 0xF9;
    txDoubleSum += txSum;
    temp = remotePacketNumberUn & 0x00FF;
    radioPrint->write(temp);
    txSum += temp;
    txDoubleSum += txSum;
    temp = (remotePacketNumberUn >> 8) & 0x00FF;
    radioPrint->write(temp);
    txSum += temp;
    txDoubleSum += txSum;
    radioPrint->write(typeNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    radioPrint->write(cmdNum);
    txSum += cmdNum;
    txDoubleSum += txSum;
    radioPrint->write(itemBuffer[0]);
    txSum += itemBuffer[0];
    txDoubleSum += txSum;
    radioPrint->write(itemBuffer[1]);
    txSum += itemBuffer[1];
    txDoubleSum += txSum;
    radioPrint->write(itemBuffer[2]);
    txSum += itemBuffer[2];
    txDoubleSum += txSum;
    radioPrint->write(itemBuffer[3]);
    txSum += itemBuffer[3];
    txDoubleSum += txSum;
    radioPrint->write(txSum);
    radioPrint->write(txDoubleSum);

  }
}

void SendUnMis() {

  txSum = 0;
  txDoubleSum = 0;
  radioPrint->write(0xAA);
  radioPrint->write(3);
  radioPrint->write(0xFB);
  txSum = 0xFB;
  txDoubleSum += 0;
  temp = localPacketNumberUn & 0x00FF;
  radioPrint->write(temp);
  txSum += temp;
  txDoubleSum += txSum;
  temp = (localPacketNumberUn >> 8) & 0x00FF;
  radioPrint->write(temp);
  txSum += temp;
  txDoubleSum += txSum;
  radioPrint->write(txSum);
  radioPrint->write(txDoubleSum);

}

void UnReliableTransmit() {
  txSum = 0;
  txDoubleSum = 0;
  radioPrint->write(0xAA);
  switch (typeNum) {
  case 0://float
    radioPrint->write(4);
    radioPrint->write(typeNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    radioPrint->write(cmdNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    for (uint8_t i = 0; i < 4; i++) {
      radioPrint->write((*floatPointerArray[cmdNum]).buffer[i]);
      txSum += (*floatPointerArray[cmdNum]).buffer[i];
      txDoubleSum += txSum;
    }
    break;
  case 1://int16
    radioPrint->write(2);
    radioPrint->write(typeNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    radioPrint->write(cmdNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    for (uint8_t i = 0; i < 2; i++) {
      radioPrint->write((*int16PointerArray[cmdNum]).buffer[i]);
      txSum += (*int16PointerArray[cmdNum]).buffer[i];
      txDoubleSum += txSum;
    }
    break;
  case 2://int32
    /*radioPrint->write(4);
     radioPrint->write(typeNum);
     txSum += typeNum;
     txDoubleSum += txSum;
     radioPrint->write(cmdNum);
     txSum += typeNum;
     txDoubleSum += txSum;
     for(uint8_t i = 0; i < 4; i++){
     radioPrint->write((*int32PointerArray[cmdNum]).buffer[i]);
     txSum += (*int32PointerArray[cmdNum]).buffer[i];
     txDoubleSum += txSum;
     }*/
    break;
  }
  radioPrint->write(txSum);
  radioPrint->write(txDoubleSum);
}



void HandShake() {
  handShakeState = 0;


  radioTimer = millis();

  if (EEPROM.read(HS_FLAG) == 0xAA || EEPROM.read(HS_FLAG) == 0xBB) { //Check for handshake from calibration
    if (EEPROM.read(HS_FLAG) == 0xBB) {
      radioStream = &Port0;
      radioPrint = &Port0;
    }
    EEPROM.write(HS_FLAG, 0xFF);
    packetTemp[0] = EEPROM.read(PKT_LOCAL_ORD_L);//lsb for packetNumberLocalOrdered
    packetTemp[1] = EEPROM.read(PKT_LOCAL_ORD_M);//msb for packetNumberLocalOrdered
    localPacketNumberOrdered = (packetTemp[1] << 8) | packetTemp[0];
    packetTemp[0] = EEPROM.read(PKT_LOCAL_UN_L);//lsb for packetNumberLocalUnOrdered
    packetTemp[1] = EEPROM.read(PKT_LOCAL_UN_M);//msb for packetNumberLocalUnOrdered
    localPacketNumberUn = (packetTemp[1] << 8) | packetTemp[0];
    handShake = true;
    return;
  }
  localPacketNumberOrdered = 0;
  localPacketNumberUn = 0;
  while (radioStream->available() > 0) {
    radioStream->read();//clear any data in the buffer
  }
  radioTimer = millis();
  while (millis() - radioTimer < 2000 && handShake == false) {
    //look for data on the radio port
    if (radioStream->available() > 0) {
      handShake = true;
    }
  }
  if (handShake == false) {
    return;
  }
  handShake = false;

  while (millis() - radioTimer < 2000 && handShake == false) { //***

    if (radioStream->available() > 0) { //---

      while (radioStream->available() > 0) { //+++

        radioByte = radioStream->read();

        switch (handShakeState) { //^^^

        case 0://check for 0xAA
          rxSum = 0;
          rxDoubleSum = 0;
          calibrationMode = false;
          if (radioByte == 0xAA) {
            handShakeState = 1;
          }
          break;

        case 1://get and verify the length
          if (radioByte == 0x02) { //len will always be 2 for the HS
            handShakeState = 2;
          }
          else {
            handShakeState = 0;
          }
          break;

        case 2://check for correct command byte
          if (radioByte == 0xFF) {
            rxSum += radioByte;
            rxDoubleSum += rxSum;
            handShakeState = 3;
          }
          else {
            handShakeState = 0;
          }
          break;

        case 3://check handshake type
          if (radioByte == 0x01) {
            rxSum += radioByte;
            rxDoubleSum += rxSum;
            handShakeState = 4;
            calibrationMode = true;
            break;
          }
          if (radioByte == 0x00) {
            rxDoubleSum += rxSum;
            handShakeState = 4;
            break;
          }
          handShakeState = 0;

          break;

        case 4://verify sum
          if (radioByte == rxSum) {
            handShakeState = 5;
            break;
          }
          handShakeState = 0;
          break;

        case 5://verify double sum
          if (radioByte == rxDoubleSum) {
            SendHandShakeResponse();
            handShake = true;
          }
          handShakeState = 0;
          break;

        }//^^^

      }//+++

    }//---

  }//***

  if (handShake == false) {
    calibrationMode = false;
  }

}

void SendHandShakeResponse() {
  txSum = 0;
  txDoubleSum = 0;
  radioPrint->write(0xAA);
  radioPrint->write(0x04);//packet length
  if (calibrationMode == true) {

    radioPrint->write(0xF7);//cmd byte
    txSum += 0xF7;
    txDoubleSum += txSum;
    radioPrint->write(1);//version number
    txSum += 1;
    txDoubleSum += txSum;
    radioPrint->write(1);//sub version number
    txSum += 1;
    txDoubleSum += txSum;
    radioPrint->write(NUM_WAY_POINTS);
    txSum += NUM_WAY_POINTS;
    txDoubleSum += txSum;
    radioPrint->write(txSum);
    radioPrint->write(txDoubleSum);
    for (uint8_t i = 0; i < 15; i++) {
      radioPrint->write(0xAA);
      radioPrint->write(0x04);//packet length
      radioPrint->write(0xF7);//cmd byte
      radioPrint->write(1);//version number
      radioPrint->write(1);//sub version number
      radioPrint->write(NUM_WAY_POINTS);
      radioPrint->write(txSum);
      radioPrint->write(txDoubleSum);
    }

  }
  else {
    radioPrint->write(0xFE);//cmd byte
    txSum += 0xFE;
    txDoubleSum += txSum;
    radioPrint->write(1);//version number
    txSum += 1;
    txDoubleSum += txSum;
    radioPrint->write(1);//sub version number
    txSum += 1;
    txDoubleSum += txSum;
    radioPrint->write(NUM_WAY_POINTS);
    txSum += NUM_WAY_POINTS;
    txDoubleSum += txSum;
    radioPrint->write(txSum);
    radioPrint->write(txDoubleSum);
    for (uint8_t i = 0; i < 15; i++) {
      radioPrint->write(0xAA);
      radioPrint->write(0x04);//packet length
      radioPrint->write(0xFE);//cmd byte
      radioPrint->write(1);//version number
      radioPrint->write(1);//sub version number
      radioPrint->write(NUM_WAY_POINTS);
      radioPrint->write(txSum);
      radioPrint->write(txDoubleSum);
    }

  }
}




































