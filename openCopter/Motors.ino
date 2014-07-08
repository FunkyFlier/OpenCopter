void CalibrateESC(){
  delay(50);//wait for new frame
  if (rcCommands.values.throttle > 1900){
    EEPROM.write(0x3E8,0xFF);//clear the handshake flag
    Motor1WriteMicros(2000);//set the output compare value
    Motor2WriteMicros(2000);
    Motor3WriteMicros(2000);
    Motor4WriteMicros(2000);
    Motor5WriteMicros(2000);
    Motor6WriteMicros(2000);
    Motor7WriteMicros(2000);
    Motor8WriteMicros(2000);
    delay(2000);
    Motor1WriteMicros(1000);//set the output compare value
    Motor2WriteMicros(1000);
    Motor3WriteMicros(1000);
    Motor4WriteMicros(1000);
    Motor5WriteMicros(1000);
    Motor6WriteMicros(1000);
    Motor7WriteMicros(1000);
    Motor8WriteMicros(1000);
    while (rcCommands.values.throttle > 1100){
      //delay(1);
    }
    asm volatile ("  jmp 0");  
  }
}

void MotorInit(){
  DDRE |= B00111000;
  DDRH |= B00111000;
  DDRB |= B01100000;


  TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);               
  ICR3 = PERIOD;   

  TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
  TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
  ICR4 = PERIOD;  

  TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR1 = PERIOD;


  /*Motor1WriteMicros(1000);//set the output compare value
  Motor2WriteMicros(1000);
  Motor3WriteMicros(1000);
  Motor4WriteMicros(1000);
  Motor5WriteMicros(1000);
  Motor6WriteMicros(1000);
  Motor7WriteMicros(1000);
  Motor8WriteMicros(1000);*/
}

void MotorHandler(){

  if (rcCommands.values.throttle < 1100){//move this code to a better place
    integrate = false;
    HHState = LAND;
    d.v.flightMode = STABLE;
    enterState = true;
    PitchAngle.reset();
    RollAngle.reset();
    YawAngle.reset();

    PitchRate.reset();
    RollRate.reset();
    YawRate.reset();

    AltHoldPosition.reset();
    AltHoldRate.reset();

    WayPointPosition.reset();
    WayPointRate.reset();

    LoiterXPosition.reset();
    LoiterXRate.reset();

    LoiterYPosition.reset();
    LoiterYRate.reset();

    if (rcCommands.values.rudder > 1700){
      digitalWrite(RED,HIGH);
      digitalWrite(YELLOW,HIGH);
      //digitalWrite(GREEN,LOW);
      //digitalWrite(13,LOW);
      throttleHold = true;
    }
    if (rcCommands.values.rudder < 1300){
      digitalWrite(RED,LOW);
      digitalWrite(YELLOW,LOW);
      //digitalWrite(GREEN,LOW);
      //digitalWrite(13,LOW);
      throttleHold = false;
    }


    Motor1WriteMicros(1000);
    Motor2WriteMicros(1000);
    Motor3WriteMicros(1000);
    Motor4WriteMicros(1000); 
  }
  else{
    if (throttleHold == false){
      motorCommand1 = constrain((throttleCommand + throttleAdjustment + adjustmentX + adjustmentY - adjustmentZ),1000,2000);
      motorCommand2 = constrain((throttleCommand + throttleAdjustment - adjustmentX + adjustmentY + adjustmentZ),1000,2000);
      motorCommand3 = constrain((throttleCommand + throttleAdjustment - adjustmentX - adjustmentY - adjustmentZ),1000,2000);
      motorCommand4 = constrain((throttleCommand + throttleAdjustment + adjustmentX - adjustmentY + adjustmentZ),1000,2000);
      Motor1WriteMicros(motorCommand1);
      Motor2WriteMicros(motorCommand2);
      Motor3WriteMicros(motorCommand3);
      Motor4WriteMicros(motorCommand4);
    }
    else{
      Motor1WriteMicros(1000);
      Motor2WriteMicros(1000);
      Motor3WriteMicros(1000);
      Motor4WriteMicros(1000); 
    }

  }

}














