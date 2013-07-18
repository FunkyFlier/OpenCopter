//motor defines
#define FREQ 400
#define PRESCALE 8
#define PERIOD ((F_CPU/PRESCALE/FREQ) - 1)

#define Motor1WriteMicros(x) OCR3B = x * 2//motor 1 is attached to pin2
#define Motor2WriteMicros(x) OCR3C = x * 2//motor 2 is attached to pin3
#define Motor3WriteMicros(x) OCR3A = x * 2//motor 3 is attached to pin5
#define Motor4WriteMicros(x) OCR4A = x * 2//motor 4 is attached to pin6


void setup(){
  MotorInit();
  Motor1WriteMicros(2000);
  Motor2WriteMicros(2000);
  Motor3WriteMicros(2000);
  Motor4WriteMicros(2000);
  //adjust the delay if your ESC goes into program mode
  delay(2000);
  Motor1WriteMicros(1000);
  Motor2WriteMicros(1000);
  Motor3WriteMicros(1000);
  Motor4WriteMicros(1000);
}
void loop(){

}
void MotorInit(){
  DDRE |= B00111000;//set the ports as outputs
  DDRH |= B00001000;


  // Init PWM Timer 3                                       
  // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTOM
  TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);                 // Prescaler set to 8, that gives us a resolution of 0.5us
  ICR3 = PERIOD;                                // Clock_speed / ( Prescaler * desired_PWM_Frequency) #defined above.  

  TCCR4A = (1<<WGM41)|(1<<COM4A1);
  TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
  ICR4 = PERIOD;


}


