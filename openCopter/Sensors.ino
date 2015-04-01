
void GetInitialQuat() {

  float inertialSumZ;
  imu.InitialQuat();


  imu.GenerateRotationMatrix();
  imuTimer = micros();


  imu.GetEuler();


  imu.initialAccMagnitude.val = 0;
  inertialSumZ = 0;
  for (int i = 0; i < 200; i ++) {
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
void CalibrateSensors() {

  generalPurposeTimer = millis();
  while (1) {
    if ( millis() - generalPurposeTimer >= 10) {
      generalPurposeTimer = millis();
#ifdef V2
      AccSSLow();
      SPI.transfer(OUT_X_L_A | READ | MULTI);
      accX.buffer[0] = SPI.transfer(0x00);
      accX.buffer[1] = SPI.transfer(0x00);
      accY.buffer[0] = SPI.transfer(0x00);
      accY.buffer[1] = SPI.transfer(0x00);
      accZ.buffer[0] = SPI.transfer(0x00);
      accZ.buffer[1] = SPI.transfer(0x00);
      accX.val = accX.val >> 4;
      accY.val = accY.val >> 4;
      accZ.val = accZ.val >> 4;
      AccSSHigh();
#endif
#ifdef V1
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
      SPI.setDataMode(SPI_MODE0);

      accY.val *= -1;
      accZ.val *= -1;
#endif
#ifdef ROT_45
      tempX = accX.val *  0.7071067 + accY.val * 0.7071067;
      tempY = accX.val * -0.7071067 + accY.val * 0.7071067;
      accX.val = tempX;
      accY.val = tempY;
#endif

      I2c.read(MAG_ADDRESS, HMC5983_OUT_X_H, 6);
      magX.buffer[1] = I2c.receive();//X
      magX.buffer[0] = I2c.receive();
      magZ.buffer[1] = I2c.receive();//Z
      magZ.buffer[0] = I2c.receive();
      magY.buffer[1] = I2c.receive();//Y
      magY.buffer[0] = I2c.receive();
#ifdef V1
      magY.val *= -1;
      magZ.val *= -1;
#endif

      PollPressure();
      if (newBaro == true) {
        newBaro = false;
      }
      if (sendCalibrationData == true) {
        SendCalData();
      }
    }
    Radio();
  }
}

void SendCalData() {
  int16_u temp;

  txSum = 0;
  txDoubleSum = 0;
  radioPrint->write(0xAA);
  switch (cmdNum) {
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
      //Serial<<"mag cal sendingr\n";
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
      //Serial<<"acc cal sending\r\n";
      break;
    case 2:
      radioPrint->write(17);

      radioPrint->write(2);
      txSum += 2;
      txDoubleSum += txSum;

      temp.val = rcData[0].rcvd;
      radioPrint->write(temp.buffer[0]);
      txSum += temp.buffer[0];
      txDoubleSum += txSum;
      radioPrint->write(temp.buffer[1]);
      txSum += temp.buffer[1];
      txDoubleSum += txSum;

      temp.val = rcData[1].rcvd;
      radioPrint->write(temp.buffer[0]);
      txSum += temp.buffer[0];
      txDoubleSum += txSum;
      radioPrint->write(temp.buffer[1]);
      txSum += temp.buffer[1];
      txDoubleSum += txSum;

      temp.val = rcData[2].rcvd;
      radioPrint->write(temp.buffer[0]);
      txSum += temp.buffer[0];
      txDoubleSum += txSum;
      radioPrint->write(temp.buffer[1]);
      txSum += temp.buffer[1];
      txDoubleSum += txSum;

      temp.val = rcData[3].rcvd;
      radioPrint->write(temp.buffer[0]);
      txSum += temp.buffer[0];
      txDoubleSum += txSum;
      radioPrint->write(temp.buffer[1]);
      txSum += temp.buffer[1];
      txDoubleSum += txSum;

      temp.val = rcData[4].rcvd;
      radioPrint->write(temp.buffer[0]);
      txSum += temp.buffer[0];
      txDoubleSum += txSum;
      radioPrint->write(temp.buffer[1]);
      txSum += temp.buffer[1];
      txDoubleSum += txSum;

      temp.val = rcData[5].rcvd;
      radioPrint->write(temp.buffer[0]);
      txSum += temp.buffer[0];
      txDoubleSum += txSum;
      radioPrint->write(temp.buffer[1]);
      txSum += temp.buffer[1];
      txDoubleSum += txSum;

      temp.val = rcData[6].rcvd;
      radioPrint->write(temp.buffer[0]);
      txSum += temp.buffer[0];
      txDoubleSum += txSum;
      radioPrint->write(temp.buffer[1]);
      txSum += temp.buffer[1];
      txDoubleSum += txSum;

      temp.val = rcData[7].rcvd;
      radioPrint->write(temp.buffer[0]);
      txSum += temp.buffer[0];
      txDoubleSum += txSum;
      radioPrint->write(temp.buffer[1]);
      txSum += temp.buffer[1];
      txDoubleSum += txSum;

      radioPrint->write(txSum);
      radioPrint->write(txDoubleSum);
      //Serial<<"rc cal sending\r\n";
      break;
  }
}


void GPSStart() {
  uint8_t LEDState = 0;
  gps.init();


  generalPurposeTimer = millis();
  while ((millis() - generalPurposeTimer < 1000) && (gps.newData == false)) {
    gps.Monitor();
    if (gps.newData == true) {
      GPSDetected = true;
    }
  }
  //Serial<<"gps det: "<<GPSDetected<<"\r\n";
  //to do add feed back with leds
  if (GPSDetected == true) {
    gpsFailSafe = false;
    while (gps.data.vars.gpsFix != 3) {

      gps.Monitor();
      if (millis() - generalPurposeTimer > 500) {
        generalPurposeTimer = millis();
        LEDState++;
        if (LEDState == 4) {
          LEDState = 0;
        }
      }
      switch (LEDState) {
        case 0:
          digitalWrite(13, HIGH);
          digitalWrite(RED, HIGH);
          digitalWrite(YELLOW, HIGH);
          digitalWrite(GREEN, HIGH);
          break;
        case 1:
          digitalWrite(13, LOW);
          digitalWrite(RED, HIGH);
          digitalWrite(YELLOW, HIGH);
          digitalWrite(GREEN, LOW);
          break;
        case 2:
          digitalWrite(13, HIGH);
          digitalWrite(RED, LOW);
          digitalWrite(YELLOW, LOW);
          digitalWrite(GREEN, HIGH);
          break;
        case 3:
          digitalWrite(13, LOW);
          digitalWrite(RED, HIGH);
          digitalWrite(YELLOW, LOW);
          digitalWrite(GREEN, LOW);
          break;
      }
    }
    while (gps.data.vars.numSV < (MIN_SATS + 2) ) {
      gps.Monitor();
      if (millis() - generalPurposeTimer > 500) {
        generalPurposeTimer = millis();
        LEDState++;
        if (LEDState == 4) {
          LEDState = 0;
        }
      }
      switch (LEDState) {
        case 0:
          digitalWrite(13, HIGH);
          digitalWrite(RED, LOW);
          digitalWrite(YELLOW, LOW);
          digitalWrite(GREEN, LOW);
          break;
        case 1:
          digitalWrite(13, LOW);
          digitalWrite(RED, HIGH);
          digitalWrite(YELLOW, LOW);
          digitalWrite(GREEN, LOW);
          break;
        case 2:
          digitalWrite(13, LOW);
          digitalWrite(RED, LOW);
          digitalWrite(YELLOW, HIGH);
          digitalWrite(GREEN, LOW);
          break;
        case 3:
          digitalWrite(13, LOW);
          digitalWrite(RED, LOW);
          digitalWrite(YELLOW, LOW);
          digitalWrite(GREEN, HIGH);
          break;
      }
    }
    while (gps.data.vars.hAcc * 0.001 > (HACC_MAX - 0.5) ) {
      gps.Monitor();
      if (millis() - generalPurposeTimer > 500) {
        generalPurposeTimer = millis();
        LEDState++;
        if (LEDState == 4) {
          LEDState = 0;
        }
      }
      switch (LEDState) {
        case 0:
          digitalWrite(13, HIGH);
          digitalWrite(RED, HIGH);
          digitalWrite(YELLOW, LOW);
          digitalWrite(GREEN, LOW);
          break;
        case 1:
          digitalWrite(13, LOW);
          digitalWrite(RED, LOW);
          digitalWrite(YELLOW, HIGH);
          digitalWrite(GREEN, HIGH);
          break;
        case 2:
          digitalWrite(13, HIGH);
          digitalWrite(RED, HIGH);
          digitalWrite(YELLOW, LOW);
          digitalWrite(GREEN, LOW);
          break;
        case 3:
          digitalWrite(13, LOW);
          digitalWrite(RED, LOW);
          digitalWrite(YELLOW, HIGH);
          digitalWrite(GREEN, HIGH);
          break;
      }
    }
    while (gps.data.vars.sAcc * 0.001 > (SACC_MAX - 0.25) ) {
      gps.Monitor();
      if (millis() - generalPurposeTimer > 500) {
        generalPurposeTimer = millis();
        LEDState++;
        if (LEDState == 4) {
          LEDState = 0;
        }
      }
      switch (LEDState) {
        case 0:
          digitalWrite(13, HIGH);
          digitalWrite(RED, LOW);
          digitalWrite(YELLOW, LOW);
          digitalWrite(GREEN, HIGH);
          break;
        case 1:
          digitalWrite(13, LOW);
          digitalWrite(RED, HIGH);
          digitalWrite(YELLOW, HIGH);
          digitalWrite(GREEN, LOW);
          break;
        case 2:
          digitalWrite(13, HIGH);
          digitalWrite(RED, LOW);
          digitalWrite(YELLOW, LOW);
          digitalWrite(GREEN, HIGH);
          break;
        case 3:
          digitalWrite(13, LOW);
          digitalWrite(RED, HIGH);
          digitalWrite(YELLOW, HIGH);
          digitalWrite(GREEN, LOW);
          break;
      }
    }

    gps.newData = false;
    while (gps.newData == false) {
      gps.Monitor();
    }
    homeBase.lat.val = gps.data.vars.lat;
    homeBase.lon.val = gps.data.vars.lon;
    homeLat.val = (gps.data.vars.lat) * 0.0000001;
    homeLon.val = (gps.data.vars.lon) * 0.0000001;
  }



}

void VerifyMag() {
  I2c.read((uint8_t)MAG_ADDRESS, (uint8_t)HMC5983_ID_A, (uint8_t)3);
  imu.magDetected = true;
  if (I2c.receive() != 0x48) {
    imu.magDetected = false;
    return;
  }

  if (I2c.receive() != 0x34) {
    imu.magDetected = false;
    return;
  }

  if (I2c.receive() != 0x33) {
    imu.magDetected = false;
    return;
  }
}
void GetAltitude(float *press, float *pressInit, float *alti) {

  float pressureRatio =  *press /  *pressInit;
  *alti = (1.0f - pow(pressureRatio, 0.190295f)) * 44330.0f;
}


#ifdef V1
void PollPressure(void) {
  if (millis() - baroPollTimer > POLL_RATE) {
    switch (pressureState) {
      case 0://read ut
        baroDelayTimer = millis();
        StartUT();
        pressureState = 1;
        break;
      case 1://wait for ready signal
        if (millis() - baroDelayTimer > 5) {
          baroDelayTimer = millis();
          pressureState = 2;
          ut = ReadUT();
          StartUP();
        }

        break;
      case 2://read up
        if (millis() - baroDelayTimer > CONV_TIME) {
          baroPollTimer = millis();
          up = ReadUP();
          temperature = Temperature(ut);
          pressure.val = (float)Pressure(up);
          pressureState = 0;
          newBaro = true;
        }
        break;

    }
  }
}

long Pressure(unsigned long up) {


  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6) >> 12) >> 11;
  x2 = (ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((long)ac1) * 4 + x3) << OSS) + 2) >> 2;

  // Calculate B4
  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;

  b7 = ((unsigned long)(up - b3) * (50000 >> OSS));
  if (b7 < 0x80000000)
    p = (b7 << 1) / b4;
  else
    p = (b7 / b4) << 1;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p += (x1 + x2 + 3791) >> 4;

  return p;
}

short Temperature(unsigned int ut) {

  x1 = (((long)ut - (long)ac6) * (long)ac5) >> 15;
  x2 = ((long)mc << 11) / (x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8) >> 4);
}

void StartUT(void) {
  I2c.write(BMP085_ADDRESS, 0xF4, 0x2E);
}

unsigned int ReadUT(void) {



  I2c.read(BMP085_ADDRESS, 0xF6, 2);
  msb = I2c.receive();
  lsb = I2c.receive();

  return ((msb << 8) | lsb);
}

void StartUP(void) {
  I2c.write(BMP085_ADDRESS, 0xF4, (0x34 + (OSS << 6)));
}

unsigned long ReadUP(void) {

  I2c.read(BMP085_ADDRESS, 0xF6, 3);
  msb = I2c.receive();
  lsb = I2c.receive();
  xlsb = I2c.receive();
  return ((((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8 - OSS));
}

void BaroInit(void) {
  pressureState = 0;
  newBaro = false;
  I2c.read(BMP085_ADDRESS, 0xAA, 22);
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
  
  I2c.read(BMP085_ADDRESS,0xD0,1);
  if (I2c.receive() != 0x55){
    while(1){
      Serial<<"missing baro\r\n";
    }
  }
  //this is to get the ground pressure for relative altitude
  //lower pressure than this means positive altitude
  //higher pressure than this means negative altitude
  baroCount = 0;
  baroSum = 0;
  while (baroCount < 10) { //use a while instead of a for loop because the for loop runs too fast
    PollPressure();
    if (newBaro == true) {
      newBaro = false;
      baroCount++;
      baroSum += pressure.val;
    }
  }
  initialPressure = baroSum / 10;



}


#endif//#ifdef V1


#ifdef V2
void PollPressure(void) {


  if (millis() - baroPollTimer >= BARO_CONV_TIME) {
    switch (baroState) {
      case 0://start temp conv
        BaroSSLow();
        SPI.transfer(CONVERT_D2_OSR4096);
        BaroSSHigh();
        baroState = 1;
        baroDelayTimer = millis();
        break;
      case 1:
        if (millis() - baroDelayTimer >= 10) {
          BaroSSLow();
          SPI.transfer(ADC_READ);
          D_rcvd.buffer[2] = SPI.transfer(0x00);
          D_rcvd.buffer[1] = SPI.transfer(0x00);
          D_rcvd.buffer[0] = SPI.transfer(0x00);
          D2 = (float)D_rcvd.val;
          BaroSSHigh();
          baroState = 2;
        }
        break;
      case 2:
        BaroSSLow();
        SPI.transfer(CONVERT_D1_OSR4096);
        BaroSSHigh();
        baroState = 3;
        baroDelayTimer = millis();
        break;
      case 3:
        if (millis() - baroDelayTimer >= 10) {
          BaroSSLow();
          SPI.transfer(ADC_READ);
          D_rcvd.buffer[2] = SPI.transfer(0x00);
          D_rcvd.buffer[1] = SPI.transfer(0x00);
          D_rcvd.buffer[0] = SPI.transfer(0x00);
          D1 = (float)D_rcvd.val;
          BaroSSHigh();
          baroState = 0;
          baroPollTimer = millis();
          GetBaro();
          newBaro = true;
        }
        break;
    }
  }
}
void GetBaro() {


  dT = D2 - (((uint32_t)C5.val) << 8);
  TEMP = (dT * C6.val) / 8388608;
  OFF = C2.val * 65536.0 + (C4.val * dT) / 128;
  SENS = C1.val * 32768.0 + (C3.val * dT) / 256;

  if (TEMP < 0) {
    // second order temperature compensation when under 20 degrees C
    float T2 = (dT * dT) / 0x80000000;
    float Aux = TEMP * TEMP;
    float OFF2 = 2.5 * Aux;
    float SENS2 = 1.25 * Aux;
    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
  }

  P = (D1 * SENS / 2097152 - OFF) / 32768;
  temperature = TEMP + 2000;
  pressure.val = P;

}

void BaroInit(void) {

  BaroSSLow();
  SPI.transfer(MS5611_RESET);
  BaroSSHigh();
  delay(5);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_Setup);
  promSetup.buffer[1] = SPI.transfer(0x00);
  promSetup.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_C1);
  C1.buffer[1] = SPI.transfer(0x00);
  C1.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_C2);
  C2.buffer[1] = SPI.transfer(0x00);
  C2.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_C3);
  C3.buffer[1] = SPI.transfer(0x00);
  C3.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_C4);
  C4.buffer[1] = SPI.transfer(0x00);
  C4.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_C5);
  C5.buffer[1] = SPI.transfer(0x00);
  C5.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_C6);
  C6.buffer[1] = SPI.transfer(0x00);
  C6.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_C6);
  C6.buffer[1] = SPI.transfer(0x00);
  C6.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPI.transfer(MS5611_PROM_CRC);
  promCRC.buffer[1] = SPI.transfer(0x00);
  promCRC.buffer[0] = SPI.transfer(0x00);
  BaroSSHigh();


  if (C1.val == 0 || C2.val == 0 || C3.val == 0 || C4.val == 0 || C5.val == 0 || C6.val == 0 ) {
    while (1) {
      Serial << "missing barometer\r\n";
    }
  }

  CheckCRC();
  //this is to get the ground pressure for relative altitude
  //lower pressure than this means positive altitude
  //higher pressure than this means negative altitude
  baroCount = 0;
  baroSum = 0;
  while (baroCount < 10) { //use a while instead of a for loop because the for loop runs too fast
    PollPressure();
    if (newBaro == true) {
      newBaro = false;
      baroCount++;
      baroSum += pressure.val;
    }
  }
  initialPressure = baroSum / 10;




  //use the line below for altitdue above sea level
  //pressureInitial = 101325;

}
void CheckCRC() {
  int16_t cnt;
  uint16_t n_rem;
  uint16_t crc_read;
  uint8_t n_bit;
  uint16_t n_prom[8] = {
    promSetup.val, C1.val, C2.val, C3.val, C4.val, C5.val, C6.val, promCRC.val
  };
  n_rem = 0x00;

  crc_read = n_prom[7];

  n_prom[7] = (0xFF00 & (n_prom[7]));

  for (cnt = 0; cnt < 16; cnt++) {
    if (cnt & 1) {
      n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

    }
    else {
      n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
    }

    for (n_bit = 8; n_bit > 0; n_bit--) {
      if (n_rem & 0x8000) {
        n_rem = (n_rem << 1) ^ 0x3000;

      }
      else {
        n_rem = (n_rem << 1);
      }
    }
  }

  n_rem = (0x000F & (n_rem >> 12));
  n_prom[7] = crc_read;


  if ((0x000F & crc_read) != (n_rem ^ 0x00)) {
    Serial << "CRC failed\r\n";
  }

}

#endif

void MagInit() {
  //continous conversion 220Hz
  I2c.write((uint8_t)MAG_ADDRESS, (uint8_t)HMC5983_CRA_REG, (uint8_t)0x9C);
  I2c.write((uint8_t)MAG_ADDRESS, (uint8_t)HMC5983_CRB_REG, (uint8_t)0x60);
  I2c.write((uint8_t)MAG_ADDRESS, (uint8_t)HMC5983_MR_REG, (uint8_t)0x80);


  VerifyMag();



  I2c.read(MAG_ADDRESS, HMC5983_OUT_X_H, 6);
  magX.buffer[1] = I2c.receive();//X
  magX.buffer[0] = I2c.receive();
  magZ.buffer[1] = I2c.receive();//Z
  magZ.buffer[0] = I2c.receive();
  magY.buffer[1] = I2c.receive();//Y
  magY.buffer[0] = I2c.receive();
#ifdef V1
  magY.val *= -1;
  magZ.val *= -1;
#endif
  shiftedMagX  = magX.val - magOffSetX;
  shiftedMagY  = magY.val - magOffSetY;
  shiftedMagZ  = magZ.val - magOffSetZ;
  scaledMagX = magWInv00 * shiftedMagX + magWInv01 * shiftedMagY + magWInv02 * shiftedMagZ;
  scaledMagY = magWInv10 * shiftedMagX + magWInv11 * shiftedMagY + magWInv12 * shiftedMagZ;
  scaledMagZ = magWInv20 * shiftedMagX + magWInv21 * shiftedMagY + magWInv22 * shiftedMagZ;
  calibMagX.val = scaledMagX;
  calibMagY.val = scaledMagY;
  calibMagZ.val = scaledMagZ;
  for (uint8_t i = 0; i < 100; i++) {
    GetMag();
    delay(5);
  }

}

void AccInit() {

#ifdef V2
  AccSSLow();
  SPI.transfer(CTRL_REG1_A | WRITE | SINGLE);
  SPI.transfer(0x77);//400Hz all axes enabled
  AccSSHigh();

  AccSSLow();
  SPI.transfer(CTRL_REG2_A | WRITE | SINGLE);
  SPI.transfer(0x00);//high pass filter not used
  AccSSHigh();

  AccSSLow();
  SPI.transfer(CTRL_REG3_A | WRITE | SINGLE);
  SPI.transfer(0x00);//not using interrupts for polling sensor
  AccSSHigh();

  AccSSLow();
  SPI.transfer(CTRL_REG4_A | WRITE | SINGLE);
  SPI.transfer(0x18);//little endian
  AccSSHigh();

  AccSSLow();
  SPI.transfer(CTRL_REG5_A | WRITE | SINGLE);
  SPI.transfer(0x00);//not using interrupts for polling sensor
  AccSSHigh();

  AccSSLow();
  SPI.transfer(CTRL_REG6_A | WRITE | SINGLE);
  SPI.transfer(0x00);//not using interrupts for polling sensor
  AccSSHigh();
#endif

#ifdef V1
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
  SPI.setDataMode(SPI_MODE0);

#endif


  GetAcc();


  shiftedAccX  = accX.val - accXOffset;
  shiftedAccY  = accY.val - accYOffset;
  shiftedAccZ  = accZ.val - accZOffset;
  scaledAccX.val = shiftedAccX * accXScale;
  scaledAccY.val = shiftedAccY * accYScale;
  scaledAccZ.val = shiftedAccZ * accZScale;


  filtAccX.val = scaledAccX.val;
  filtAccY.val = scaledAccY.val;
  filtAccZ.val = scaledAccZ.val;

  for (uint16_t i = 0; i < 100; i++) {
    GetAcc();
    delayMicroseconds(2500);
  }

}

void GyroInit() {
  SPI.setDataMode(SPI_MODE0);

  GyroSSLow();
  SPI.transfer(L3G_WHO_AM_I  | READ | SINGLE);
  if (SPI.transfer(0x00) != 0xD4) {
    while (1) {
      Serial << "gyro failed\r\n";
    }
  }
  GyroSSHigh();


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
  SPI.transfer(0x30); //2000dps scale
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
  /*gyroOffsetX = 0;
   gyroOffsetY = 0;
   gyroOffsetZ = 0;
   gyroSumX = 0;
   gyroSumY = 0;
   gyroSumZ = 0;*/
  for (uint16_t j = 0; j < 500; j ++) {
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
  UpdateOffset();
  /* for (uint16_t j = 0; j < 500; j ++){
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
   gyroSumY += (gyroY.val);
   gyroSumZ += (gyroZ.val);

   if (

   delay(3);
   }
   gyroOffsetX = gyroSumX / 500.0;
   gyroOffsetY = gyroSumY / 500.0;
   gyroOffsetZ = gyroSumZ / 500.0;*/

  GetGyro();

}

void GetMag() {
  I2c.read(MAG_ADDRESS, HMC5983_OUT_X_H, 6);
  magX.buffer[1] = I2c.receive();//X
  magX.buffer[0] = I2c.receive();
  magZ.buffer[1] = I2c.receive();//Z
  magZ.buffer[0] = I2c.receive();
  magY.buffer[1] = I2c.receive();//Y
  magY.buffer[0] = I2c.receive();
#ifdef V1
  magY.val *= -1;
  magZ.val *= -1;
#endif

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
void UpdateOffset() {
  gyroOffsetX = 0;
  gyroOffsetY = 0;
  gyroOffsetZ = 0;
  gyroSumX = 0;
  gyroSumY = 0;
  gyroSumZ = 0;

  GyroSSLow();
  SPI.transfer(L3G_OUT_X_L  | READ | MULTI);
  gyroX.buffer[0] = SPI.transfer(0x00);
  gyroX.buffer[1] = SPI.transfer(0x00);
  gyroY.buffer[0] = SPI.transfer(0x00);
  gyroY.buffer[1] = SPI.transfer(0x00);
  gyroZ.buffer[0] = SPI.transfer(0x00);
  gyroZ.buffer[1] = SPI.transfer(0x00);
#ifdef V1
  gyroY.val *= -1;
  gyroZ.val *= -1;
#endif

  GyroSSHigh();
  gyroPrevX = gyroX.val;
  gyroPrevY = gyroY.val;
  gyroPrevZ = gyroZ.val;

  for (uint16_t j = 0; j < 150; j ++) {
    GyroSSLow();
    SPI.transfer(L3G_OUT_X_L  | READ | MULTI);
    gyroX.buffer[0] = SPI.transfer(0x00);
    gyroX.buffer[1] = SPI.transfer(0x00);
    gyroY.buffer[0] = SPI.transfer(0x00);
    gyroY.buffer[1] = SPI.transfer(0x00);
    gyroZ.buffer[0] = SPI.transfer(0x00);
    gyroZ.buffer[1] = SPI.transfer(0x00);
#ifdef V1
    gyroY.val *= -1;
    gyroZ.val *= -1;
#endif
    GyroSSHigh();
    gyroSumX += gyroX.val;
    gyroSumY += (gyroY.val);
    gyroSumZ += (gyroZ.val);
    if ( abs(gyroPrevX - gyroX.val) > 25 || abs(gyroPrevY - gyroY.val) > 25 || abs(gyroPrevZ - gyroZ.val) > 25 ) {
      gyroSumX = gyroX.val;
      gyroSumY = gyroY.val;
      gyroSumZ = gyroZ.val;
      j = 0;
    }
    gyroPrevX = gyroX.val;
    gyroPrevY = gyroY.val;
    gyroPrevZ = gyroZ.val;
    watchDogFailSafeCounter = 0;
    delay(3);
  }
  gyroOffsetX = gyroSumX / 150;
  gyroOffsetY = gyroSumY / 150;
  gyroOffsetZ = gyroSumZ / 150;
}
void GetGyro() {

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
#ifdef V1
  gyroY.val *= -1;
  gyroZ.val *= -1;
#endif



  gyroX.val -= gyroOffsetX;
  gyroY.val -= gyroOffsetY;
  gyroZ.val -= gyroOffsetZ;
#ifdef ROT_45
  tempX = gyroX.val *  0.7071067 + gyroY.val * 0.7071067;
  tempY = gyroX.val * -0.7071067 + gyroY.val * 0.7071067;
  gyroX.val = tempX;
  gyroY.val = tempY;
#endif

  degreeGyroX.val = gyroX.val * 0.07;
  degreeGyroY.val = gyroY.val * 0.07;
  degreeGyroZ.val = gyroZ.val * 0.07;

  radianGyroX = ToRad(degreeGyroX.val);
  radianGyroY = ToRad(degreeGyroY.val);
  radianGyroZ = ToRad(degreeGyroZ.val);


}
void GetAcc() {
#ifdef V2
  AccSSLow();
  SPI.transfer(OUT_X_L_A | READ | MULTI);
  accX.buffer[0] = SPI.transfer(0x00);
  accX.buffer[1] = SPI.transfer(0x00);
  accY.buffer[0] = SPI.transfer(0x00);
  accY.buffer[1] = SPI.transfer(0x00);
  accZ.buffer[0] = SPI.transfer(0x00);
  accZ.buffer[1] = SPI.transfer(0x00);
  accX.val = accX.val >> 4;
  accY.val = accY.val >> 4;
  accZ.val = accZ.val >> 4;

  AccSSHigh();
#endif
#ifdef V1
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
  SPI.setDataMode(SPI_MODE0);

  accY.val *= -1;
  accZ.val *= -1;
#endif

#ifdef ROT_45
  tempX = accX.val *  0.7071067 + accY.val * 0.7071067;
  tempY = accX.val * -0.7071067 + accY.val * 0.7071067;
  accX.val = tempX;
  accY.val = tempY;
#endif
  shiftedAccX  = accX.val - accXOffset;
  shiftedAccY  = accY.val - accYOffset;
  shiftedAccZ  = accZ.val - accZOffset;
  scaledAccX.val = shiftedAccX * accXScale;
  scaledAccY.val = shiftedAccY * accYScale;
  scaledAccZ.val = shiftedAccZ * accZScale;




  if (lpfDT > 0.005 || lpfDT <= 0) {
    lpfDT = 0.005;
  }

  alpha.val = lpfDT / (lpfDT + RC_CONST);
  beta = 1.0 - alpha.val;
  filtAccX.val = filtAccX.val * beta + scaledAccX.val * alpha.val;
  filtAccY.val = filtAccY.val * beta + scaledAccY.val * alpha.val;
  filtAccZ.val = filtAccZ.val * beta + scaledAccZ.val * alpha.val;

  accToFilterX = -1.0 * filtAccX.val;//if the value from the filter is sent it will not work when the algorithm normalizes the vector
  accToFilterY = -1.0 * filtAccY.val;
  accToFilterZ = -1.0 * filtAccZ.val;

}
















































