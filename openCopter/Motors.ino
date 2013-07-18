void MotorInit(){
  DDRE |= B00111000;//set the ports as outputs
  DDRH |= B00001000;

  
  TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);               
  ICR3 = PERIOD;   

  TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
  TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
  ICR4 = PERIOD;  

  TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR1 = PERIOD;
  
  Motor1WriteMicros(1000);//set the output compare value
  Motor2WriteMicros(1000);
  Motor3WriteMicros(1000);
  Motor4WriteMicros(1000);
  
}

void MotorHandler(){
  //this function handles the motors and arms the aircraft
  if (rcCommands.values.throttle < 1100){
    integrate = false;
    Reset();
    
    Motor1WriteMicros(1000);
    Motor2WriteMicros(1000);
    Motor3WriteMicros(1000);
    Motor4WriteMicros(1000); 

    if (rcCommands.values.rudder > 1750){
      digitalWrite(RED,HIGH);
      hold = true;
    }
    if (rcCommands.values.rudder < 1300){
      digitalWrite(RED,LOW);
      hold = false;
    }
  }
  else{
    if (hold == false){
      motorCommand1 = constrain((uint16_t)(rcCommands.values.throttle  + adjustmentX + adjustmentY - adjustmentZ),1000,2000);
      motorCommand2 = constrain((uint16_t)(rcCommands.values.throttle - adjustmentX + adjustmentY + adjustmentZ),1000,2000);
      motorCommand3 = constrain((uint16_t)(rcCommands.values.throttle - adjustmentX - adjustmentY - adjustmentZ),1000,2000);
      motorCommand4 = constrain((uint16_t)(rcCommands.values.throttle + adjustmentX - adjustmentY + adjustmentZ),1000,2000);

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




