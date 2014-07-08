void _200HzISRConfig(){
  TCCR5A = (1<<COM5A1);
  TCCR5B = (1<<CS51)|(1<<WGM52);//why WGM52?
  TIMSK5 = (1<<OCIE5A);
  OCR5A = 10000;
}

ISR(TIMER5_COMPA_vect, ISR_NOBLOCK){
  if (watchDogStartCount == true){
    watchDogFailSafeCounter++;
    RCFailSafeCounter++;
  }
  ReadSerialStreams();
  
  if (watchDogFailSafeCounter >=200){
    TIMSK5 = (0<<OCIE5A);
    digitalWrite(13,LOW);
    digitalWrite(RED,LOW);
    digitalWrite(YELLOW,LOW);
    digitalWrite(GREEN,LOW);
    Motor1WriteMicros(1000);//set the output compare value
    Motor2WriteMicros(1000);
    Motor3WriteMicros(1000);
    Motor4WriteMicros(1000);
    Motor5WriteMicros(1000);
    Motor6WriteMicros(1000);
    Motor7WriteMicros(1000);
    Motor8WriteMicros(1000);
    while(1){
      digitalWrite(RED,HIGH);
      digitalWrite(GREEN,LOW);
      delay(500);
      digitalWrite(RED,LOW);
      digitalWrite(GREEN,HIGH);
      delay(500);
    }
  }
}
void ReadSerialStreams(){
  if (rcType != RC){
    FeedLine();
  }


  if (GPSDetected == true){
    while(gpsPort.available() > 0){
      if (gps.encode(gpsPort.read()) == true){
        gpsUpdate = true;
      }
    }
  }

}

























