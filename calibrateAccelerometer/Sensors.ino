void MagInit(){
  
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)LSM303_CRA_REG,(uint8_t)0x1C);
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)LSM303_CRB_REG,(uint8_t)0x60);
  I2c.write((uint8_t)MAG_ADDRESS,(uint8_t)LSM303_MR_REG,(uint8_t)0x00);
}

void AccInit(){

  SPI.setDataMode(SPI_MODE3);

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | BW_RATE);
  SPI.transfer(0x0B);
  AccSSHigh();

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | POWER_CTL);
  SPI.transfer(0x08);//start measurment
  AccSSHigh();

  AccSSLow();
  SPI.transfer(WRITE | SINGLE | DATA_FORMAT);
  SPI.transfer(0x0B);//full resolution + / - 16g
  AccSSHigh();


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
  Serial<<"*"<<mag.v.x<<","<<mag.v.y<<","<<mag.v.z<<"\r\n";

}
void GetAcc(){
  SPI.setDataMode(SPI_MODE3);
  AccSSLow();
  SPI.transfer(DATAX0 | READ | MULTI);
  for (int i = 0; i < 6; i++){//the endianness matches as does the axis order
    acc.buffer[i] = SPI.transfer(0x00);
  }
  acc.v.y *= -1;
  acc.v.z *= -1;
  Serial<<acc.v.x<<","<<acc.v.y<<","<<acc.v.z<<"\r\n";
  AccSSHigh();  


}

