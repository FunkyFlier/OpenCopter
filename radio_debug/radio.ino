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

  while(radio.available() > 0){
    radio.read();//clear any data in the buffer
  }
  radioTimer = millis();
  while(millis() - radioTimer < 2000 && handShake == false){//wait 0.5s for data from ground station
    //Serial<<"waiting for HS\r\n";
    if(radio.available() > 0){
      handShake = true;
    }
  }
  if(handShake == false){
    return;
  }
  handShake = false;
  while(radio.available() < 5){
  }
  while(radio.available() > 0){
    switch(cmdState){
    case 0:
      inByteRadio = radio.read();
      if (inByteRadio == 0xAA){
        cmdState = 1;
      }
      break;
    case 1:
      inByteRadio = radio.read();
      if (inByteRadio == 0x01){
        cmdState = 2;
      }
      else{
        cmdState = 0;
      }
      break;
    case 2:
      inByteRadio = radio.read();
      if (inByteRadio == 0xFF){
        cmdState = 3;
      }
      else{
        cmdState = 0;
      }
      break;
    case 3:
      inByteRadio = radio.read();
      if (inByteRadio == 0xFF){
        cmdState = 4;
      }
      else{
        cmdState = 0;
      }
      break;
    case 4:
      cmdState = 0;
      outputSum = 0;
      outputDoubleSum = 0;
      inByteRadio = radio.read();
      if (inByteRadio == 0xFF){
        handShake = true;
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
      }
      break;
    default:
      break;
    }
  }
}
void Radio(){
  while(radio.available() > 0){
    newRadio = true;
    inByteRadio = radio.read();
    inBuffer[bufferIndexRadio] = inByteRadio;
    bufferIndexRadio++;
    if (bufferIndexRadio == RADIO_BUF_SIZE){
      bufferIndexRadio = 0;
    }
  }
  while (newRadio == true){
    switch (cmdState){      
    case 0://check for start byte
      if (inBuffer[parseIndex] == 0xAA){
        cmdState = 1;
      }
      break;
    case 1://get the packet length
      numBytesIn = inBuffer[parseIndex];
      if (numBytesIn >= 32){//to do make a define
        cmdState = 0;
        inputSum = 0;
        inputDoubleSum = 0;
        cmdIndex = 0;       
        break;
      }
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
        /*if (cmdIndex == numBytesIn){
         cmdState = 4;
         }*/
        break;
      }

      break;
    case 3:
      cmdBuffer[cmdIndex] = inBuffer[parseIndex];
      inputSum += inBuffer[parseIndex];
      inputDoubleSum += inputSum;
      cmdIndex++;
      if (cmdIndex >=32){
        cmdState = 0;
        inputSum = 0;
        inputDoubleSum = 0;
        cmdIndex = 0;
      }
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
        SendData(varIndex,numBytesOut,numBytesIn);        
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
        if (cmdBuffer[0] <= 0x46){
          ParseCommand();
          memcpy(&d.buffer[varIndex],&cmdBuffer[1],(numBytesIn - 4));
          SendAckOrdered();
        }
        else{
          if (cmdBuffer[0] == 0x47){
            TuningHandler();
            SendAckOrdered();
          }
          else{
            WayPointHandler();
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
        if (cmdBuffer[0] <= 0x46){
          //to do change name for this function call
          ParseCommand();
          SendDataReliable(varIndex,numBytesOut,(numBytesIn - 4));
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
void SendData(uint8_t index, uint8_t numOut, uint8_t numIn){
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
void SendDataReliable(uint8_t index, uint8_t numOut, uint8_t numIn){
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
      //Serial<<wayPoints[inputWayPointNumber].v.lat<<","<<wayPoints[inputWayPointNumber].v.lon<<","<<wayPoints[inputWayPointNumber].v.alt<<"\r\n";
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
      //wayPointState = WP_TRAVEL;
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
    latTarget = gps.data.vars.lat;
    lonTarget = gps.data.vars.lon;
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
    varIndex = 276;
    numBytesOut = 1;
    break;
  case 0x0E:
    varIndex = 277;
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
  default:
    break;
  }
}

void ShowHex(byte convertByte){
  Serial << hex[(convertByte >>4) & 0x0F];
  Serial << hex[convertByte & 0x0F]<<"\r\n";
}
void InitVars(){

  d.v.pitch = 1.1;//0
  d.v.roll = 2.2;//4
  d.v.yaw = 3.3;//8
  d.v.lattitude = 329729470;//12
  d.v.longitude = -967089360;//16
  d.v.baroAltitude = 6.6;//20
  d.v.gpsAltitude = 7;//24
  d.v.velN = -8;//28
  d.v.velE = -9;//32
  d.v.velD = -10;//36
  d.v.gpsHeading = -11;//40
  d.v._3DSpeed = 12;
  d.v.groundSpeed = 13;

  d.v.kp_pitch_rate = 0.4125;
  d.v.ki_pitch_rate = 2.9049296;
  d.v.kd_pitch_rate = 0.03905;//60
  d.v.fc_pitch_rate = 50.0;

  d.v.kp_roll_rate = 0.4125;
  d.v.ki_roll_rate = 2.9049296;
  d.v.kd_roll_rate = 0.03905;
  d.v.fc_roll_rate = 50.0;//80

    d.v.kp_yaw_rate = 2.25;
  d.v.ki_yaw_rate = 0.25;
  d.v.kd_yaw_rate = 0.01;
  d.v.fc_yaw_rate = 50.0;




  d.v.kp_pitch_attitude = 5.35;//100
  d.v.ki_pitch_attitude = 0;
  d.v.kd_pitch_attitude = 0.075;
  d.v.fc_pitch_attitude = 75.0;

  d.v.kp_roll_attitude = 5.35;
  d.v.ki_roll_attitude = 0;//120
  d.v.kd_roll_attitude = 0.075;
  d.v.fc_roll_attitude = 75.0;

  d.v.kp_yaw_attitude = 3.0;
  d.v.ki_yaw_attitude = 0;
  d.v.kd_yaw_attitude = 0.01;//140
  d.v.fc_yaw_attitude = 50.0;

  d.v.kp_altitude_position = 1.5;
  d.v.ki_altitude_position = 0;
  d.v.kd_altitude_position = 0;
  d.v.fc_altitude_position = 0;

  d.v.kp_altitude_rate = 90;
  d.v.ki_altitude_rate = 35;
  d.v.kd_altitude_rate = 0.05;
  d.v.fc_altitude_rate = 50;

  d.v.kp_loiter_pos_x = 0;//180
  d.v.ki_loiter_pos_x = 0;
  d.v.kd_loiter_pos_x = 0;
  d.v.fc_loiter_pos_x = 0;

  d.v.kp_loiter_rate_x = 0;//180
  d.v.ki_loiter_rate_x = 0;
  d.v.kd_loiter_rate_x = 0;
  d.v.fc_loiter_rate_x = 0;

  d.v.kp_loiter_pos_y = 0;
  d.v.ki_loiter_pos_y = 0;//200
  d.v.kd_loiter_pos_y = 0;
  d.v.fc_loiter_pos_y = 0;

  d.v.kp_loiter_rate_y = 0;
  d.v.ki_loiter_rate_y = 0;//200
  d.v.kd_loiter_rate_y = 0;
  d.v.fc_loiter_rate_y = 0;

  d.v.kp_waypoint_position = 10;
  d.v.ki_waypoint_position = 0;
  d.v.kd_waypoint_position = 0;//220
  d.v.fc_waypoint_position = 0;

  d.v.kp_waypoint_velocity = 1;
  d.v.ki_waypoint_velocity = 10;
  d.v.kd_waypoint_velocity = 0;
  d.v.fc_waypoint_velocity = 0;

  d.v.gpsFix = 0x3E;//244

  d.v.flightMode = STABLE;
}











