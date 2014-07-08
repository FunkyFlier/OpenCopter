void CalcGravityOffSet(){
  imuTimer = micros();

  for (int i =0; i < 500; i++){

    while (micros() - imuTimer < 10000){
    }
    imuDT = (micros() - imuTimer) * 0.000001;
    imuTimer = micros();
    GetAcc();
    GetMag();
    GetGyro();
    imu.AHRSupdate();
  }
  imu.GetEuler();
  gravSum = 0;
  for (int i = 0; i < 50; i++){
    GetAcc();
    gravSum += imu.GetGravOffset();
    imu.GetInertial();
    delayMicroseconds(2500);
  }
  gravAvg = gravSum / 50.0;
  imu.gravityOffSet = gravAvg;
  /*for (int i = 0; i < 50; i++){
   GetAcc();
   gravSum += -1.0*sqrt(scaledAccX * scaledAccX + scaledAccY * scaledAccY + scaledAccZ * scaledAccZ );
   //Serial<<imu.GetGravOffset()<<"\r\n";;
   delayMicroseconds(2500);
   }
   //imu.gravityOffSet = -1.0*sqrt(scaledAccX * scaledAccX + scaledAccY * scaledAccY + scaledAccZ * scaledAccZ );
   gravAvg = gravSum / 50.0;
   imu.gravityOffSet = gravAvg;*/
}


void CalibrateSensors(){

  generalPurposeTimer = millis();
  while(1){
    if ( millis() - generalPurposeTimer >= 10){
      generalPurposeTimer = millis();
      GetAcc();
      GetMag();
      SendCalData();

    }
    Radio();
    watchDogFailSafeCounter = 0;
  }
}

void SendCalData(){
  outputSum = 0;
  outputDoubleSum = 0;
  radio.write(0xAA);
  radio.write(0x0D);

  radio.write(0x00);

  temp = (acc.v.x & 0x00FF);
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum +=outputSum;

  temp = (acc.v.x >> 8);
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum +=outputSum;

  temp = (acc.v.y & 0x00FF);
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum +=outputSum;

  temp = (acc.v.y >> 8);
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum +=outputSum;

  temp = (acc.v.z & 0x00FF);
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum +=outputSum;

  temp = (acc.v.z >> 8);
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum +=outputSum;

  temp = (mag.v.x & 0x00FF);
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum +=outputSum;

  temp = (mag.v.x >> 8);
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum +=outputSum;

  temp = (mag.v.y & 0x00FF);
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum +=outputSum;

  temp = (mag.v.y >> 8);
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum +=outputSum;

  temp = (mag.v.z & 0x00FF);
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum +=outputSum;

  temp = (mag.v.z >> 8);
  radio.write(temp);
  outputSum += temp;
  outputDoubleSum +=outputSum;

  radio.write(outputSum);
  radio.write(outputDoubleSum);
}

//gpsUpdate
void GPSStart(){
  uint8_t gpsStartState;
  unsigned long chars;
  unsigned short goodSentences,csFailures;


  gpsPort.begin(115200);

  GPSDetected = false;
  gpsStartState = 0;
  GPSDetected = false;
  generalPurposeTimer = millis();


  //first make sure there is data on the GPS port
  while ((millis() - generalPurposeTimer < 1000) && (GPSDetected == false)){
    digitalWrite(13,HIGH);
    digitalWrite(RED,LOW);
    digitalWrite(YELLOW,LOW);
    digitalWrite(GREEN,LOW);
    if (gpsPort.available() > 0){
      GPSDetected = true;
    }
  }

  //make sure that the data is gps data
  GPSDetected = false;
  generalPurposeTimer = millis();
  while ((millis() - generalPurposeTimer < 1000) ){
    digitalWrite(13,HIGH);
    digitalWrite(RED,LOW);
    digitalWrite(YELLOW,HIGH);
    digitalWrite(GREEN,LOW);
    while(gpsPort.available() > 0){
      inByte = gpsPort.read();
      gpsUpdate = gps.encode(inByte);
      if (inByte == '$'){//consider switching to validate that the reciver is recieving GPGGA
        GPSDetected = true;
      }
      //gpsUpdate = gps.encode(gpsPort.read());
    }
  }

  if (GPSDetected == false){
    return;
  }
  GPSDetected = false;

  //wait for a good gps fix
  while(GPSDetected == false){

    while(gpsPort.available() > 0){
      gpsUpdate = gps.encode(gpsPort.read());
      gps.get_position(&d.v.lattitude,&d.v.longitude,&gpsFixAge);
      if (gpsFixAge > 500){
        gpsStartState = 0;
        gpsUpdate = false;
      }
      switch (gpsStartState){
      case 0:
        digitalWrite(13,LOW);
        digitalWrite(RED,HIGH);
        digitalWrite(YELLOW,LOW);
        digitalWrite(GREEN,LOW);
        if (gpsUpdate == true){
          gpsUpdate = false;
          gpsStartState = 1;
        }
        break;
      case 1:
        digitalWrite(13,LOW);
        digitalWrite(RED,LOW);
        digitalWrite(YELLOW,HIGH);
        digitalWrite(GREEN,LOW);
        //Serial<<gps.hdop()<<","<<gps.satellites()<<"\r\n";
        if (gpsUpdate == true){
          //Serial<<"*\r\n";
          gpsUpdate = false;
          if (gps.hdop() < 200 && gps.satellites() >= 6){
            gpsStartState = 2;
            generalPurposeTimer = millis();
          }
        }
        break;
      case 2:
        digitalWrite(13,HIGH);
        digitalWrite(RED,LOW);
        digitalWrite(YELLOW,LOW);
        digitalWrite(GREEN,LOW);
        if (gpsUpdate == true){
          gpsUpdate = false;
          if (gps.hdop() > 200 || gps.satellites() < 6){
            gpsStartState = 1;
          }
        }
        if (millis() - generalPurposeTimer > 15000){
          gps.get_position(&d.v.lattitude,&d.v.longitude,&gpsFixAge);
          homeBase.coord.lat = d.v.lattitude;
          homeBase.coord.lon = d.v.longitude;
          gpsStartState = 3;
          generalPurposeTimer = millis();
        }
        break;
      case 3:
        digitalWrite(13,LOW);
        digitalWrite(RED,LOW);
        digitalWrite(YELLOW,LOW);
        digitalWrite(GREEN,HIGH);
        if (gpsUpdate == true){
          gpsUpdate = false;
          if (gps.hdop() > 200 || gps.satellites() < 6){
            gpsStartState = 1;
          }
        }
        if (millis() - generalPurposeTimer > 15000){
          gps.get_position(&d.v.lattitude,&d.v.longitude,&gpsFixAge);
          DistBearing(&homeBase.coord.lat,&homeBase.coord.lon,&d.v.lattitude,&d.v.longitude,&rawX,&rawY,&beeLineDist,&beeLineHeading);
          if ( sqrt( sq(rawX) + sq(rawY)) <= 3 ){
            imu.XEst = rawX;
            imu.YEst = rawY;
            homeBaseXOffset = rawX;
            homeBaseYOffset = rawY;
            drPosX = rawX;
            drPosY = rawY;
            GPSDetected = true;
          }
          else{
            gpsStartState = 1;
          }
        }
        break;
      }

    }

  }

}



/*  if (GPSDetected == true){
 gpsUpdate = false;
 while(gpsUpdate == false){
 Serial<<"2\r\n";
 while(gpsPort.available() > 0){
 gpsUpdate = gps.encode(gpsPort.read());
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
 }
 //Serial<<"1\r\n";
 while (gps.satellites() < 8 || gps.hdop() >= 200){
 gps.get_position(&d.v.lattitude,&d.v.longitude,&gpsFixAge);
 Serial<<gps.satellites()<<","<<gps.hdop()<<","<<d.v.lattitude<<","<<d.v.longitude<<","<<gpsFixAge<<"\r\n";
 while(gpsPort.available() > 0){
 gpsUpdate = gps.encode(gpsPort.read());
 }
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
 generalPurposeTimer = millis();
 while(millis() - generalPurposeTimer < 10000){
 Serial<<"4\r\n";
 digitalWrite(13,HIGH);
 while(gpsPort.available() > 0){
 gpsUpdate = gps.encode(gpsPort.read());
 }
 gps.get_position(&d.v.lattitude,&d.v.longitude,&gpsFixAge);
 Serial<<gpsFixAge<<"\r\n";
 if (gps.satellites() < 8 || gps.hdop() >= 200 || gpsFixAge > 300){
 Serial<<"******************\r\n";
 gpsUpdate = false;
 }
 }
 Serial<<"5\r\n";
 gpsUpdate = false;
 while(gpsUpdate == false){
 while(gpsPort.available() > 0){
 gpsUpdate = gps.encode(gpsPort.read());
 }
 }
 Serial<<"6\r\n";
 digitalWrite(13,LOW);
 gps.get_position(&d.v.lattitude,&d.v.longitude,&gpsFixAge);
 homeBase.coord.lat = d.v.lattitude;
 homeBase.coord.lon = d.v.longitude;
 //Serial<<homeBase.coord.lat<<","<<homeBase.coord.lon<<"\r\n";
 gpsUpdate = false;
 }  */

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
        pressure = Pressure(up);
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
  //pinMode(READY_PIN,INPUT);
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
  while (baroCount < 10){//use a while instead of a for loop because the for loop runs too fast
    PollPressure();
    if (newBaro == true){
      newBaro = false;
      baroCount++;
      baroSum += pressure;
    }    
  }
  pressureInitial = baroSum / 10;    
  //use the line below for altitdue above sea level
  //pressureInitial = 101325;

}


void MagInit(){
  //continous conversion 220Hz
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)LSM303_CRA_REG,(uint8_t)0x1C);
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)LSM303_CRB_REG,(uint8_t)0x60);
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)LSM303_MR_REG,(uint8_t)0x00);
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
  SPI.transfer(0x0B);//full resolution + / - 16g
  AccSSHigh();

  GetAcc();

  if (acc.v.x > 0){
    scaledAccX = acc.v.x * accXScalePos;
  }
  else{
    scaledAccX = acc.v.x * accXScaleNeg;
  }
  if (acc.v.y > 0){
    scaledAccY = acc.v.y  * accYScalePos;
  }
  else{
    scaledAccY = acc.v.y  * accYScaleNeg;
  }
  if (acc.v.z > 0){
    scaledAccZ = acc.v.z * accZScalePos;
  }
  else{
    scaledAccZ = acc.v.z * accZScaleNeg;
  }
  smoothAccX = scaledAccX;
  smoothAccY = scaledAccY;
  smoothAccZ = scaledAccZ;  
  for (uint8_t i = 0; i < 3 ; i++){
    inBufferX[i] = smoothAccX;
    inBufferY[i] = smoothAccY;
    inBufferZ[i] = smoothAccZ;
    outBufferX[i] = smoothAccX;
    outBufferY[i] = smoothAccY;
    outBufferZ[i] = smoothAccZ;
  }
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
  //SPI.transfer(0x04);
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
  offsetX = 0;
  offsetY = 0;
  offsetZ = 0;
  gyroSumX = 0;
  gyroSumY = 0;
  gyroSumZ = 0;
  for (uint16_t j = 0; j < 500; j ++){
    GetGyro();
    delay(3);
  }
  for (uint16_t j = 0; j < 500; j ++){
    GetGyro();
    gyroSumX += gyro.v.x;
    gyroSumY += gyro.v.y;
    gyroSumZ += gyro.v.z;
    delay(3);
  }
  offsetX = gyroSumX / 500;
  offsetY = gyroSumY / 500;
  offsetZ = gyroSumZ / 500;
  GetGyro();

}

void GetMag(){
  I2c.read(MAG_ADDRESS,LSM303_OUT_X_H,6);
  mag.buffer[1] = I2c.receive();//X
  mag.buffer[0] = I2c.receive();
  mag.buffer[5] = I2c.receive();//Z
  mag.buffer[4] = I2c.receive();
  mag.buffer[3] = I2c.receive();//Y
  mag.buffer[2] = I2c.receive();

  mag.v.y *= -1;
  mag.v.z *= -1;
  shiftedMagX  = mag.v.x - MAG_OFFSET_X;
  shiftedMagY  = mag.v.y - MAG_OFFSET_Y;
  shiftedMagZ  = mag.v.z - MAG_OFFSET_Z;

  floatMagX = MAG_W_INV_00 * shiftedMagX + MAG_W_INV_01 * shiftedMagY + MAG_W_INV_02 * shiftedMagZ;
  floatMagY = MAG_W_INV_10 * shiftedMagX + MAG_W_INV_11 * shiftedMagY + MAG_W_INV_12 * shiftedMagZ;
  floatMagZ = MAG_W_INV_20 * shiftedMagX + MAG_W_INV_21 * shiftedMagY + MAG_W_INV_22 * shiftedMagZ;
}

void GetGyro(){
  SPI.setDataMode(SPI_MODE0);
  GyroSSLow();
  SPI.transfer(L3G_OUT_X_L  | READ | MULTI);
  for (uint8_t i = 0; i < 6; i++){//the endianness matches as does the axis order
    gyro.buffer[i] = SPI.transfer(0x00);
  }
  GyroSSHigh();

  degreeGyroX = (gyro.v.x - offsetX) * 0.07;
  degreeGyroY = -1.0 * ((gyro.v.y - offsetY) * 0.07);
  degreeGyroZ = -1.0 * ((gyro.v.z - offsetZ) * 0.07);

  radianGyroX = ToRad(degreeGyroX);
  radianGyroY = ToRad(degreeGyroY);
  radianGyroZ = ToRad(degreeGyroZ);


}
void GetAcc(){
  SPI.setDataMode(SPI_MODE3);
  AccSSLow();
  SPI.transfer(DATAX0 | READ | MULTI);
  for (uint8_t i = 0; i < 6; i++){//the endianness matches as does the axis order
    acc.buffer[i] = SPI.transfer(0x00);
  }
  AccSSHigh();  

  acc.v.y *= -1;
  acc.v.z *= -1;


  //Filter(&acc.v.x,&acc.v.y,&acc.v.z,&smoothAccX,&smoothAccY,&smoothAccZ);

  /*shiftedAccX = (smoothAccX - ACC_OFFSET_X);
   shiftedAccY = (smoothAccY - ACC_OFFSET_Y);
   shiftedAccZ = (smoothAccZ - ACC_OFFSET_Z);*/
  if (acc.v.x > 0){
    scaledAccX = acc.v.x * accXScalePos;
  }
  else{
    scaledAccX = acc.v.x * accXScaleNeg;
  }
  if (acc.v.y > 0){
    scaledAccY = acc.v.y  * accYScalePos;
  }
  else{
    scaledAccY = acc.v.y  * accYScaleNeg;
  }
  if (acc.v.z > 0){
    scaledAccZ = acc.v.z * accZScalePos;
  }
  else{
    scaledAccZ = acc.v.z * accZScaleNeg;
  }
  Filter(&scaledAccX,&scaledAccY,&scaledAccZ,&smoothAccX,&smoothAccY,&smoothAccZ);

  /*scaledAccX = shiftedAccX * ACC_W_INV_00;
   scaledAccY = shiftedAccY * ACC_W_INV_01;
   scaledAccZ = shiftedAccZ * ACC_W_INV_02;*/
  /*scaledAccX = (ACC_W_INV_00 * shiftedAccX + ACC_W_INV_01 * shiftedAccY + ACC_W_INV_02 * shiftedAccZ);
   scaledAccY = (ACC_W_INV_10 * shiftedAccX + ACC_W_INV_11 * shiftedAccY + ACC_W_INV_12 * shiftedAccZ);
   scaledAccZ = (ACC_W_INV_20 * shiftedAccX + ACC_W_INV_21 * shiftedAccY + ACC_W_INV_22 * shiftedAccZ);*/
  /*if (imu.feedBack == false){
   accToFilterX = -1.0 * smoothAccX;//if the value from the smoothing filter is sent it will not work when the algorithm normalizes the vector
   accToFilterY = -1.0 * smoothAccY;
   accToFilterZ = -1.0 * smoothAccZ;
   }*/


  //------------------------------------------------------------------------------------------------------------------------------


  /*shiftedAccX = ((float)acc.v.x - ACC_OFFSET_X) ;
   shiftedAccY = ((float)acc.v.y - ACC_OFFSET_Y) ;
   shiftedAccZ = ((float)acc.v.z - ACC_OFFSET_Z);
   
   scaledAccX = (ACC_W_INV_00 * shiftedAccX + ACC_W_INV_01 * shiftedAccY + ACC_W_INV_02 * shiftedAccZ);
   scaledAccY = (ACC_W_INV_10 * shiftedAccX + ACC_W_INV_11 * shiftedAccY + ACC_W_INV_12 * shiftedAccZ);
   scaledAccZ = (ACC_W_INV_20 * shiftedAccX + ACC_W_INV_21 * shiftedAccY + ACC_W_INV_22 * shiftedAccZ);
   
   Filter(&scaledAccX,&scaledAccY,&scaledAccZ,&smoothAccX,&smoothAccY,&smoothAccZ);
   
   accToFilterX = -1.0 * smoothAccX;//if the value from the smoothing filter is sent it will not work when the algorithm normalizes the vector
   accToFilterY = -1.0 * smoothAccY;
   accToFilterZ = -1.0 * smoothAccZ;*/

}




























