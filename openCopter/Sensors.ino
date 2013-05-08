void LevelAngles(){
  timer = micros();
  delay(5);
  for( int i = 0; i < 500; i++){//run the IMU so that the error can be driven to zero - keep it still for this
    dt = ((micros() - timer) / 1000000.0);
    timer = micros();
    GetGyro();
    GetAcc();
    GetMag();
    imu.AHRSupdate();
    delay(5);
  }
  imu.GetEuler();
  imu.pitchOffset = imu.pitch;
  imu.rollOffset = imu.roll;
}

void GetAltitude(long *press,long *pressInit, float *alti){
  pressureRatio = (float) *press / (float) *pressInit;
  *alti = (1.0f - pow(pressureRatio, 0.190295f)) * 44330.0f;
}

void PollPressure(void){
  if (millis() - baroTimer > POLL_RATE){
    switch (pressureState){
    case 0://read ut
      StartUT();
      pressureState = 1;
      break;
    case 1://wait for ready signal
      if (digitalRead(READY_PIN) == 1){
        pressureState = 2;
        ut = ReadUT();
      }
      break;
    case 2://read up
      StartUP();
      pressureState = 3;
      break;
    case 3://wait for ready signal
      if (digitalRead(READY_PIN) == 1){
        pressureState = 4;
        up = ReadUP();
      }
      break;
    case 4://
      temperature = Temperature(ut);
      pressure = Pressure(up);
      pressureState = 0;
      newBaro = true;
      baroTimer = millis();
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
  /*Wire.beginTransmission(BMP085_ADDRESS);
   Wire.write(0xF4);
   Wire.write(0x2E);
   Wire.endTransmission();*/
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

unsigned ReadUP(void){
  I2c.read(BMP085_ADDRESS,0xF6,3);
  msb = I2c.receive();
  lsb = I2c.receive();
  xlsb = I2c.receive();

  return ((((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS));
}

void BaroInit(void){
  pinMode(READY_PIN,INPUT);
  pressureState = 0;
  baroTimer = millis();
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
  while (newBaro == false){
    PollPressure();
  }
  newBaro = false;

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
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)LSM303_CRA_REG,(uint8_t)0x1C);
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)LSM303_CRB_REG,(uint8_t)0x60);
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)LSM303_MR_REG,(uint8_t)0x00);
}

void AccInit(){
  SPI.setDataMode(SPI_MODE3);

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | BW_RATE);
  SPI.transfer(0x0C);//400hz
  AccSSHigh();

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | POWER_CTL);
  SPI.transfer(0x08);//start measurment
  AccSSHigh();

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | DATA_FORMAT);
  SPI.transfer(0x0B);//full resolution + / - 16g
  AccSSHigh();

  for(j = 0; j < 50; j++){
    GetAcc();//to get the smoothing filters caugt up
    delay(5);
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
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG1 | WRITE | SINGLE);
  SPI.transfer(0xCF); //fastest update rate 30Hz cutoff
  GyroSSHigh();
  //this section takes an average of 500 samples to calculate the offset
  //if this step is skipped the IMU will still work, but this simple step gives better results
  offsetX = 0;
  offsetY = 0;
  offsetZ = 0;
  gyroSumX = 0;
  gyroSumY = 0;
  gyroSumZ = 0;
  for (j = 0; j < 50; j ++){//give the internal LPF time to warm up
    GetGyro();
    delay(3);
  }
  for (j = 0; j < 50; j ++){//give the internal LPF time to warm up
    GetGyro();
    gyroSumX += gyro.v.x;
    gyroSumY += gyro.v.y;
    gyroSumZ += gyro.v.z;
    delay(3);
  }
  offsetX = gyroSumX / 50;
  offsetY = gyroSumY / 50;
  offsetZ = gyroSumZ / 50;

}

void GetMag(){
  I2c.read(MAG_ADDRESS,LSM303_OUT_X_H,6);
  mag.buffer[1] = I2c.receive();//X
  mag.buffer[0] = I2c.receive();
  mag.buffer[5] = I2c.receive();//Z
  mag.buffer[4] = I2c.receive();
  mag.buffer[3] = I2c.receive();//Y
  mag.buffer[2] = I2c.receive();

  floatMagX = ((float)mag.v.x - compassXMin) * inverseXRange - 1.0;
  floatMagY = -1.0 * (((float)mag.v.y - compassYMin) * inverseYRange - 1.0);
  floatMagZ = -1.0 * (((float)mag.v.z - compassZMin) * inverseZRange - 1.0);

}

void GetGyro(){
  SPI.setDataMode(SPI_MODE0);
  GyroSSLow();
  SPI.transfer(L3G_OUT_X_L  | READ | MULTI);
  for (i = 0; i < 6; i++){//the endianness matches as does the axis order
    gyro.buffer[i] = SPI.transfer(0x00);
  }
  GyroSSHigh();
  //don't forget to convert to radians per second. This absolutely will not work otherwise
  //check the data sheet for more info on this
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
  for (i = 0; i < 6; i++){//the endianness matches as does the axis order
    acc.buffer[i] = SPI.transfer(0x00);
  }
  AccSSHigh();  

  //the filter expects gravity to be in NED coordinates
  //switching the sign will fix this
  //the raw values are not negated because the accelerometer values are used for the altimeter
  //one must be careful to make sure that all of the sensors are in the North, East, Down convention
  rawX = acc.v.x;
  Smoothing(&rawX,&smoothAccX);//this is a very simple low pass digital filter
  rawY = acc.v.y * -1.0;
  Smoothing(&rawY,&smoothAccY);//it helps significiantlly with vibrations. 
  rawZ = acc.v.z * -1.0;
  Smoothing(&rawZ,&smoothAccZ);//if the applicaion is not prone to vibrations this can skipped and the raw value simply recast as a float
  accToFilterX = -1.0 * smoothAccX;//if the value from the smoothing filter is sent it will not work when the algorithm normalizes the vector
  accToFilterY = -1.0 * smoothAccY;
  accToFilterZ = -1.0 * smoothAccZ;

  if (smoothAccX > 0){
    scaledAccX = smoothAccX * ACC_SC_X_POS;
  }
  else{
    scaledAccX = smoothAccX * ACC_SC_X_NEG;
  }
  if (smoothAccY > 0){
    scaledAccY = smoothAccY * ACC_SC_Y_POS;
  }
  else{
    scaledAccY = smoothAccY * ACC_SC_Y_NEG;
  }
  if (smoothAccZ > 0){
    scaledAccZ = smoothAccZ * ACC_SC_Z_POS;
  }
  else{
    scaledAccZ = smoothAccZ * ACC_SC_Z_NEG;
  }  

}





