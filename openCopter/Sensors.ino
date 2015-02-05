

void GetInitialQuat(){
  uint8_t i = 0;
  float inertialSumX,inertialSumY,inertialSumZ;
  imu.InitialQuat();


  imu.GenerateRotationMatrix();
  imuTimer = micros();


  imu.GetEuler();


  imu.initialAccMagnitude.val = 0;
  inertialSumZ = 0;
  for(int i = 0; i < 200; i ++){
    GetAcc();
    imu.GetInertial();

    inertialSumZ += imu.inertialZ.val;
    delayMicroseconds(2500);
  }

  imu.initialAccMagnitude.val = inertialSumZ / 200.0;

  imu.accelBiasX.val = 0;
  imu.accelBiasY.val = 0;
  imu.accelBiasZ.val = 0;

  imu.currentEstIndex = (uint8_t)kp_waypoint_velocity.val;
  imu.lagIndex = 0;

}
void CalibrateSensors(){

  generalPurposeTimer = millis();
  while(1){
    if ( millis() - generalPurposeTimer >= 10){
      generalPurposeTimer = millis();

      SPI.setDataMode(SPI_MODE3);
      AccSSLow();
      SPI.transfer(DATAX0 | READ | MULTI);
      accX.buffer[0] = SPI.transfer(0x00);
      accX.buffer[1] = SPI.transfer(0x00);
      accY.buffer[0] = SPI.transfer(0x00);
      accY.buffer[1] = SPI.transfer(0x00);
      accZ.buffer[0] = SPI.transfer(0x00);
      accZ.buffer[1] = SPI.transfer(0x00);
      AccSSHigh();  
      accY.val *= -1;
      accZ.val *= -1;

      I2c.read(MAG_ADDRESS,LSM303_OUT_X_H,6);
      magX.buffer[1] = I2c.receive();//X
      magX.buffer[0] = I2c.receive();
      magZ.buffer[1] = I2c.receive();//Z
      magZ.buffer[0] = I2c.receive();
      magY.buffer[1] = I2c.receive();//Y
      magY.buffer[0] = I2c.receive();
      magY.val *= -1;
      magZ.val *= -1;
      
      PollPressure();
      if (newBaro == true){
        newBaro = false;
      } 
      if (sendCalibrationData == true){
        SendCalData();
      }
    }
    Radio();
  }
}

void SendCalData(){
  int16_u temp;

  txSum = 0;
  txDoubleSum = 0;
  radioPrint->write(0xAA);
  switch(cmdNum){
  case 0:
    radioPrint->write(7);


    radioPrint->write((uint8_t)0x00);
    txSum += 0;
    txDoubleSum += txSum;

    radioPrint->write(magX.buffer[0]);
    txSum += magX.buffer[0];
    txDoubleSum += txSum;

    radioPrint->write(magX.buffer[1]);
    txSum += magX.buffer[1];
    txDoubleSum += txSum;

    radioPrint->write(magY.buffer[0]);
    txSum += magY.buffer[0];
    txDoubleSum += txSum;

    radioPrint->write(magY.buffer[1]);
    txSum += magY.buffer[1];
    txDoubleSum += txSum;

    radioPrint->write(magZ.buffer[0]);
    txSum += magZ.buffer[0];
    txDoubleSum += txSum;

    radioPrint->write(magZ.buffer[1]);
    txSum += magZ.buffer[1];
    txDoubleSum += txSum;

    radioPrint->write(txSum);
    radioPrint->write(txDoubleSum);

    break;
  case 1:
    radioPrint->write(7);


    radioPrint->write(1);
    txSum += 1;
    txDoubleSum += txSum;

    radioPrint->write(accX.buffer[0]);
    txSum += accX.buffer[0];
    txDoubleSum += txSum;

    radioPrint->write(accX.buffer[1]);
    txSum += accX.buffer[1];
    txDoubleSum += txSum;

    radioPrint->write(accY.buffer[0]);
    txSum += accY.buffer[0];
    txDoubleSum += txSum;

    radioPrint->write(accY.buffer[1]);
    txSum += accY.buffer[1];
    txDoubleSum += txSum;

    radioPrint->write(accZ.buffer[0]);
    txSum += accZ.buffer[0];
    txDoubleSum += txSum;

    radioPrint->write(accZ.buffer[1]);
    txSum += accZ.buffer[1];
    txDoubleSum += txSum;

    radioPrint->write(txSum);
    radioPrint->write(txDoubleSum);
    break;
  case 2:
    radioPrint->write(17);

    radioPrint->write(2);
    txSum += 2;
    txDoubleSum += txSum;

    temp.val = rawRCVal[THRO];
    radioPrint->write(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    radioPrint->write(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    temp.val = rawRCVal[AILE];
    radioPrint->write(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    radioPrint->write(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    temp.val = rawRCVal[ELEV];
    radioPrint->write(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    radioPrint->write(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    temp.val = rawRCVal[RUDD];
    radioPrint->write(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    radioPrint->write(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    temp.val = rawRCVal[GEAR];
    radioPrint->write(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    radioPrint->write(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    temp.val = rawRCVal[AUX1];
    radioPrint->write(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    radioPrint->write(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    temp.val = rawRCVal[AUX2];
    radioPrint->write(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    radioPrint->write(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    temp.val = rawRCVal[AUX3];
    radioPrint->write(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    radioPrint->write(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    radioPrint->write(txSum);
    radioPrint->write(txDoubleSum);
    break;
  }
}

void StartUpAHRSRun(){
  if ( micros() - imuTimer >= 13333){  
    imuDT = (micros() - imuTimer) * 0.000001;
    imuTimer = micros();
    GetAcc();
    GetMag();
    GetGyro();
    PollPressure();
    if (newBaro == true){
      newBaro = false;
    } 
    imu.AHRSupdate();
  } 
}



void GPSStart(){
  uint8_t LEDState;
  gps.init();


  generalPurposeTimer = millis();
  while ((millis() - generalPurposeTimer < 1000) && (gps.newData == false)){
    gps.Monitor();
    if (gps.newData == true){
      GPSDetected = true;
    }
  }
  //Serial<<"gps det: "<<GPSDetected<<"\r\n";
  //to do add feed back with leds
  if (GPSDetected == true){
    gpsFailSafe = false;
    while (gps.data.vars.gpsFix != 3){

      gps.Monitor();
      if (millis() - generalPurposeTimer > 500){
        generalPurposeTimer = millis();
        LEDState++;
        if (LEDState == 4){
          LEDState = 0;
        }
      }
      switch (LEDState){
      case 0:
        digitalWrite(13,HIGH);
        digitalWrite(RED,LOW);
        digitalWrite(YELLOW,LOW);
        digitalWrite(GREEN,LOW);
        break;
      case 1:
        digitalWrite(13,LOW);
        digitalWrite(RED,HIGH);
        digitalWrite(YELLOW,LOW);
        digitalWrite(GREEN,LOW);
        break;
      case 2:
        digitalWrite(13,LOW);
        digitalWrite(RED,LOW);
        digitalWrite(YELLOW,HIGH);
        digitalWrite(GREEN,LOW);
        break;
      case 3:
        digitalWrite(13,LOW);
        digitalWrite(RED,LOW);
        digitalWrite(YELLOW,LOW);
        digitalWrite(GREEN,HIGH);
        break;
      }
    }
    while(gps.data.vars.hAcc > 500000){
      gps.Monitor();
      if (millis() - generalPurposeTimer > 500){
        generalPurposeTimer = millis();
        LEDState++;
        if (LEDState == 4){
          LEDState = 0;
        }
      }
      switch (LEDState){
      case 0:
        digitalWrite(13,HIGH);
        digitalWrite(RED,LOW);
        digitalWrite(YELLOW,LOW);
        digitalWrite(GREEN,LOW);
        break;
      case 1:
        digitalWrite(13,HIGH);
        digitalWrite(RED,HIGH);
        digitalWrite(YELLOW,LOW);
        digitalWrite(GREEN,LOW);
        break;
      case 2:
        digitalWrite(13,HIGH);
        digitalWrite(RED,LOW);
        digitalWrite(YELLOW,HIGH);
        digitalWrite(GREEN,LOW);
        break;
      case 3:
        digitalWrite(13,HIGH);
        digitalWrite(RED,LOW);
        digitalWrite(YELLOW,LOW);
        digitalWrite(GREEN,HIGH);
        break;
      }
    }

    gps.newData = false;
    while (gps.newData == false){
      gps.Monitor();
    }
    homeBase.lat.val = gps.data.vars.lat;
    homeBase.lon.val = gps.data.vars.lon;
  }  



}
void GetAltitude(long *press,long *pressInit, float *alti){
  pressureRatio = (float) *press / (float) *pressInit;
  *alti = (1.0f - pow(pressureRatio, 0.190295f)) * 44330.0f;
}

void PollPressure(void){
  if (millis() - baroPollTimer > POLL_RATE){
    switch (pressureState){
    case 0://read ut
      StartUT();
      pressureState = 1;
      baroTimer = millis();
      break;
    case 1://wait for ready signal
      if (millis() - baroTimer > 5){
        pressureState = 2;
        ut = ReadUT();
        StartUP();
        baroTimer = millis();
      }

      break;
    case 2://read up
      if (millis() - baroTimer > CONV_TIME){
        up = ReadUP();
        temperature = Temperature(ut);
        pressure.val = Pressure(up);
        pressureState = 0;
        newBaro = true;
        baroPollTimer = millis();
      }
      break;

    }
  }
}

long Pressure(unsigned long up){


  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  return p;
}

short Temperature(unsigned int ut){

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);
}

void StartUT(void){
  I2c.write(BMP085_ADDRESS,0xF4,0x2E);
}

unsigned int ReadUT(void){



  I2c.read(BMP085_ADDRESS,0xF6,2);
  msb = I2c.receive();
  lsb = I2c.receive();

  return ((msb << 8) | lsb);
}

void StartUP(void){
  I2c.write(BMP085_ADDRESS,0xF4,(0x34 + (OSS<<6)));
}

unsigned long ReadUP(void){

  I2c.read(BMP085_ADDRESS,0xF6,3);
  msb = I2c.receive();
  lsb = I2c.receive();
  xlsb = I2c.receive();
  return ((((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS));
}

void BaroInit(void){
  pressureState = 0;
  newBaro = false;
  I2c.read(BMP085_ADDRESS,0xAA,22);
  msb = I2c.receive();
  lsb = I2c.receive();
  ac1 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  ac2 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  ac3 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  ac4 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  ac5 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  ac6 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  b1 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  b2 = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  mb = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  mc = (msb << 8) | lsb;

  msb = I2c.receive();
  lsb = I2c.receive();
  md = (msb << 8) | lsb;
  //this is to get the ground pressure for relative altitude
  //lower pressure than this means positive altitude
  //higher pressure than this means negative altitude
  baroCount = 0;
  baroSum = 0;
  while (baroCount < 10){//use a while instead of a for loop because the for loop runs too fast
    PollPressure();
    if (newBaro == true){
      newBaro = false;
      baroCount++;
      baroSum += pressure.val;
    }    
  }
  pressureInitial = baroSum / 10;   


  initialTemp.val = temperature;

  //use the line below for altitdue above sea level
  //pressureInitial = 101325;

}


void MagInit(){
  //continous conversion 220Hz
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)LSM303_CRA_REG,(uint8_t)0x18);
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)LSM303_CRB_REG,(uint8_t)0x60);
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)LSM303_MR_REG,(uint8_t)0x00);
  I2c.read(MAG_ADDRESS,LSM303_OUT_X_H,6);
  magX.buffer[1] = I2c.receive();//X
  magX.buffer[0] = I2c.receive();
  magZ.buffer[1] = I2c.receive();//Z
  magZ.buffer[0] = I2c.receive();
  magY.buffer[1] = I2c.receive();//Y
  magY.buffer[0] = I2c.receive();
  magY.val *= -1;
  magZ.val *= -1;
  shiftedMagX  = magX.val - magOffSetX;
  shiftedMagY  = magY.val - magOffSetY;
  shiftedMagZ  = magZ.val - magOffSetZ;
  scaledMagX = magWInv00 * shiftedMagX + magWInv01 * shiftedMagY + magWInv02 * shiftedMagZ;
  scaledMagY = magWInv10 * shiftedMagX + magWInv11 * shiftedMagY + magWInv12 * shiftedMagZ;
  scaledMagZ = magWInv20 * shiftedMagX + magWInv21 * shiftedMagY + magWInv22 * shiftedMagZ;
  calibMagX.val = scaledMagX;
  calibMagY.val = scaledMagY;
  calibMagZ.val = scaledMagZ;
  for (uint8_t i = 0; i < 100; i++){
    GetMag();
    delay(5);
  }

}
void AccInit(){

  SPI.setDataMode(SPI_MODE3);

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | BW_RATE);
  SPI.transfer(0x0C);
  AccSSHigh();

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | POWER_CTL);
  SPI.transfer(0x08);//start measurment
  AccSSHigh();

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | DATA_FORMAT);
  SPI.transfer(0x08);//full resolution + / - 16g
  AccSSHigh();

  GetAcc();

  accY.val *= -1;
  accZ.val *= -1;

 shiftedAccX.val  = accX.val - accXOffset;
  shiftedAccY.val  = accY.val - accYOffset;
  shiftedAccZ.val  = accZ.val - accZOffset;
  scaledAccX.val = shiftedAccX.val * accXScale;
  scaledAccY.val = shiftedAccY.val * accYScale;
  scaledAccZ.val = shiftedAccZ.val * accZScale;

/*  shiftedAccX.val  = accX.val - 0;
  shiftedAccY.val  = accY.val - 0;
  shiftedAccZ.val  = accZ.val - 0;
  scaledAccX.val = shiftedAccX.val * 0.03828125;
  scaledAccY.val = shiftedAccY.val * 0.03828125;
  scaledAccZ.val = shiftedAccZ.val * 0.03828125;*/


  for (uint16_t i = 0;i < 100; i++){
    GetAcc();
    delayMicroseconds(2500);
  }

}


void GyroInit(){
  SPI.setDataMode(SPI_MODE0);
  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG2 | WRITE | SINGLE);
  SPI.transfer(0x00); //high pass filter disabled
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG3 | WRITE | SINGLE);
  SPI.transfer(0x00); //not using interrupts
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG4 | WRITE | SINGLE);
  SPI.transfer(0x20); //2000dps scale
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG5 | WRITE | SINGLE);
  SPI.transfer(0x02); //out select to use the second LPF
  //not using HPF or interrupts
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG1 | WRITE | SINGLE);
  SPI.transfer(0x8F);
  GyroSSHigh();
  //this section takes an average of 500 samples to calculate the offset
  //if this step is skipped the IMU will still work, but this simple step gives better results
  gyroOffsetX = 0;
  gyroOffsetY = 0;
  gyroOffsetZ = 0;
  gyroSumX = 0;
  gyroSumY = 0;
  gyroSumZ = 0;
  for (uint16_t j = 0; j < 500; j ++){
    SPI.setDataMode(SPI_MODE0);
    GyroSSLow();
    SPI.transfer(L3G_OUT_X_L  | READ | MULTI);
    gyroX.buffer[0] = SPI.transfer(0x00);
    gyroX.buffer[1] = SPI.transfer(0x00);
    gyroY.buffer[0] = SPI.transfer(0x00);
    gyroY.buffer[1] = SPI.transfer(0x00);
    gyroZ.buffer[0] = SPI.transfer(0x00);
    gyroZ.buffer[1] = SPI.transfer(0x00);

    GyroSSHigh();
    delay(3);
  }
  for (uint16_t j = 0; j < 500; j ++){
    SPI.setDataMode(SPI_MODE0);
    GyroSSLow();
    SPI.transfer(L3G_OUT_X_L  | READ | MULTI);
    gyroX.buffer[0] = SPI.transfer(0x00);
    gyroX.buffer[1] = SPI.transfer(0x00);
    gyroY.buffer[0] = SPI.transfer(0x00);
    gyroY.buffer[1] = SPI.transfer(0x00);
    gyroZ.buffer[0] = SPI.transfer(0x00);
    gyroZ.buffer[1] = SPI.transfer(0x00);

    GyroSSHigh();
    gyroSumX += gyroX.val;
    gyroSumY += (gyroY.val * -1);
    gyroSumZ += (gyroZ.val * -1);
    delay(3);
  }
  gyroOffsetX = gyroSumX / 500;
  gyroOffsetY = gyroSumY / 500;
  gyroOffsetZ = gyroSumZ / 500;
  GetGyro();

}

void GetMag(){
  I2c.read(MAG_ADDRESS,LSM303_OUT_X_H,6);
  magX.buffer[1] = I2c.receive();//X
  magX.buffer[0] = I2c.receive();
  magZ.buffer[1] = I2c.receive();//Z
  magZ.buffer[0] = I2c.receive();
  magY.buffer[1] = I2c.receive();//Y
  magY.buffer[0] = I2c.receive();
  magY.val *= -1;
  magZ.val *= -1;

  shiftedMagX  = magX.val - magOffSetX;
  shiftedMagY  = magY.val - magOffSetY;
  shiftedMagZ  = magZ.val - magOffSetZ;
  scaledMagX = magWInv00 * shiftedMagX + magWInv01 * shiftedMagY + magWInv02 * shiftedMagZ;
  scaledMagY = magWInv10 * shiftedMagX + magWInv11 * shiftedMagY + magWInv12 * shiftedMagZ;
  scaledMagZ = magWInv20 * shiftedMagX + magWInv21 * shiftedMagY + magWInv22 * shiftedMagZ;


  calibMagX.val = scaledMagX;
  calibMagY.val = scaledMagY;
  calibMagZ.val = scaledMagZ;


  magToFiltX = calibMagX.val;
  magToFiltY = calibMagY.val;
  magToFiltZ = calibMagZ.val;
}
void UpdateOffset(){
  gyroOffsetX = 0;
  gyroOffsetY = 0;
  gyroOffsetZ = 0;
  gyroSumX = 0;
  gyroSumY = 0;
  gyroSumZ = 0;

  for (uint16_t j = 0; j < 150; j ++){
    GyroSSLow();
    SPI.transfer(L3G_OUT_X_L  | READ | MULTI);
    gyroX.buffer[0] = SPI.transfer(0x00);
    gyroX.buffer[1] = SPI.transfer(0x00);
    gyroY.buffer[0] = SPI.transfer(0x00);
    gyroY.buffer[1] = SPI.transfer(0x00);
    gyroZ.buffer[0] = SPI.transfer(0x00);
    gyroZ.buffer[1] = SPI.transfer(0x00);

    GyroSSHigh();
    gyroSumX += gyroX.val;
    gyroSumY += (gyroY.val * -1);
    gyroSumZ += (gyroZ.val * -1);

    delay(3);
  }
  gyroOffsetX = gyroSumX / 150;
  gyroOffsetY = gyroSumY / 150;
  gyroOffsetZ = gyroSumZ / 150;
}
void GetGyro(){

  SPI.setDataMode(SPI_MODE0);
  GyroSSLow();
  SPI.transfer(L3G_OUT_X_L  | READ | MULTI);
  gyroX.buffer[0] = SPI.transfer(0x00);
  gyroX.buffer[1] = SPI.transfer(0x00);
  gyroY.buffer[0] = SPI.transfer(0x00);
  gyroY.buffer[1] = SPI.transfer(0x00);
  gyroZ.buffer[0] = SPI.transfer(0x00);
  gyroZ.buffer[1] = SPI.transfer(0x00);
  GyroSSHigh();

  gyroY.val *= -1;
  gyroZ.val *= -1;



  gyroX.val -= gyroOffsetX; 
  gyroY.val -= gyroOffsetY;
  gyroZ.val -= gyroOffsetZ;


  degreeGyroX.val = gyroX.val * 0.07;
  degreeGyroY.val = gyroY.val * 0.07;
  degreeGyroZ.val = gyroZ.val * 0.07;

  radianGyroX = ToRad(degreeGyroX.val);
  radianGyroY = ToRad(degreeGyroY.val);
  radianGyroZ = ToRad(degreeGyroZ.val);



}
void GetAcc(){
  SPI.setDataMode(SPI_MODE3);
  AccSSLow();
  SPI.transfer(DATAX0 | READ | MULTI);
  accX.buffer[0] = SPI.transfer(0x00);
  accX.buffer[1] = SPI.transfer(0x00);
  accY.buffer[0] = SPI.transfer(0x00);
  accY.buffer[1] = SPI.transfer(0x00);
  accZ.buffer[0] = SPI.transfer(0x00);
  accZ.buffer[1] = SPI.transfer(0x00);
  AccSSHigh();  

  accY.val *= -1;
  accZ.val *= -1;


  shiftedAccX.val  = accX.val - accXOffset;
  shiftedAccY.val  = accY.val - accYOffset;
  shiftedAccZ.val  = accZ.val - accZOffset;
  scaledAccX.val = shiftedAccX.val * accXScale;
  scaledAccY.val = shiftedAccY.val * accYScale;
  scaledAccZ.val = shiftedAccZ.val * accZScale;

/*  shiftedAccX.val  = accX.val - 0;
  shiftedAccY.val  = accY.val - 0;
  shiftedAccZ.val  = accZ.val - 0;
  scaledAccX.val = shiftedAccX.val * 0.03828125;
  scaledAccY.val = shiftedAccY.val * 0.03828125;
  scaledAccZ.val = shiftedAccZ.val * 0.03828125;*/


  filtAccX.val = filtAccX.val * 0.9 + scaledAccX.val * 0.1;
  filtAccY.val = filtAccY.val * 0.9 + scaledAccY.val * 0.1;
  filtAccZ.val = filtAccZ.val * 0.9 + scaledAccZ.val * 0.1;

  accToFilterX = -1.0 * filtAccX.val;//if the value from the smoothing filter is sent it will not work when the algorithm normalizes the vector
  accToFilterY = -1.0 * filtAccY.val;
  accToFilterZ = -1.0 * filtAccZ.val;


}








































