void WriteROM(){
  for (uint16_t i = 0x01; i <= 0x18; i++){
    switch (i){
    case 0x01:
      outFloat.num = beta[0];
      outFloatIndex = 0;
      break;
    case 0x05:
      outFloat.num = beta[1];
      outFloatIndex = 0;
      break;
    case 0x09:
      outFloat.num = beta[2];
      outFloatIndex = 0;
      break;  
    case 0x0D:
      outFloat.num = (beta[3] * 9.8);
      outFloatIndex = 0;
      break;  
    case 0x11:
      outFloat.num = (beta[4] * 9.8);
      outFloatIndex = 0;
      break;  
    case 0x15:
      outFloat.num = (beta[5] * 9.8);
      outFloatIndex = 0;
      break;  
    default:
      break;
    }
    EEPROM.write(i,outFloat.buffer[outFloatIndex]);
    outFloatIndex++;
  }
  calibrationFlags = EEPROM.read(0x00);
  calibrationFlags |= (1<<ACC_CAL);
  calibrationFlags &= ~(1<<ACC_DEF);
  calibrationFlags &= ~(1<<ALL_DEF);
  EEPROM.write(0x00,calibrationFlags);  
}
