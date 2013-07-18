//PID loop control
void Rate(){
  PitchRate.calculate();
  RollRate.calculate();
  YawRate.calculate();
}
void Angle(){
  PitchAngle.calculate();
  RollAngle.calculate();  
}
void Reset(){
  PitchAngle.reset();
  RollAngle.reset();
  PitchRate.reset();
  RollRate.reset();
  YawRate.reset();
}

//this checks to make sure the user is not commanding any throttle after calibration is complete
void SafetyCheck(){
  timer = millis();
  while (rcCommands.values.throttle > 1020){
    
    if (rcType != RC){
      FeedLine();
    }
    
    if (millis() - timer > 500){
      digitalWrite(GREEN,toggle);
      toggle = ~toggle;
      timer = millis();
    }
    if (newRC == true){
      newRC = false;
    }
  }
}
//move the rudder to the right to start calibration
void Arm(){
  //arming procedure
  newRC = false;
  timer = millis();
  while (newRC == false){
    if (rcType == RC){
      delay(100);
    }
    if (rcType != RC){
      FeedLine();
    }
    if (millis() - timer > 1000){//in case it has incorrectly detected serial RC
      rcType = RC;
      DDRB &= 0xE0;
      PORTB |= 0x1F;//turn on pull ups
      PCMSK0 |= 0x1F;//set interrupt mask for all of PORTK
      PCICR |= 1<<0;//enable the pin change interrupt for K
      delay(100);//wait for a few frames
      Center();
      timer = millis();
    }
  }
  newRC = false;

  while (rcCommands.values.rudder < 1750){
    if (rcType == RC){
      delay(100);//wait for a few frames
    }
    if (rcType != RC){
      FeedLine();
    }
    if (newRC == true){
      newRC = false;
    }
  } 

  digitalWrite(RED,LOW);

}

