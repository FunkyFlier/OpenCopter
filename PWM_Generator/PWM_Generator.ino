//LED defines
#define RED 38
#define YELLOW 40
#define GREEN 42

//motor defines
#define FREQ 200
#define PRESCALE 8
#define PERIOD ((F_CPU/PRESCALE/FREQ) - 1)

#define Motor1WriteMicros(x) OCR3B = x * 2//motor 1 is attached to pin2
#define Motor2WriteMicros(x) OCR3C = x * 2//motor 2 is attached to pin3
#define Motor3WriteMicros(x) OCR3A = x * 2//motor 3 is attached to pin5
#define Motor4WriteMicros(x) OCR4A = x * 2//motor 4 is attached to pin6

#define Motor5WriteMicros(x) OCR4B = x * 2//motor 1 is attached to pin7
#define Motor6WriteMicros(x) OCR4C = x * 2//motor 2 is attached to pin8
#define Motor7WriteMicros(x) OCR1A = x * 2//motor 3 is attached to pin11
#define Motor8WriteMicros(x) OCR1B = x * 2//motor 4 is attached to pin12

uint16_t command = 1500;

void setup(){
  pinMode(GREEN,OUTPUT);
  pinMode(YELLOW,OUTPUT);
  pinMode(RED,OUTPUT);
  pinMode(13,OUTPUT);
  
  digitalWrite(GREEN,HIGH);
  digitalWrite(YELLOW,HIGH);
  digitalWrite(RED,HIGH);
  digitalWrite(13,HIGH);
  Serial2.begin(115200);
  MotorInit();    
  AllMotors(1500);
}

void loop(){
  
  /*Serial.println(command);
  AllMotors(command);
  delay(1500);
  command += 50;*/
  /*if (Serial2.available() > 0){
    command = Serial2.parseInt();
    Serial2.println(command);
    AllMotors(command);
  }*/
  
  AllMotors(1000);
  delay(1000);
  AllMotors(1250);
  delay(1000);
  AllMotors(1500);
  delay(1000);
  AllMotors(1750);
  delay(1000);
  AllMotors(2000);
  delay(1000);
}


void MotorInit(){
  DDRE |= B00111000;
  DDRH |= B00111000;
  DDRB |= B01100000;

  // Init PWM Timer 3                                       // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM
  TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);                 // Prescaler set to 8, that gives us a resolution of 0.5us
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
  Motor5WriteMicros(1000);//set the output compare value
  Motor6WriteMicros(1000);
  Motor7WriteMicros(1000);
  Motor8WriteMicros(1000);
}

void AllMotors(uint16_t command){
  Motor1WriteMicros(command);//set the output compare value
  Motor2WriteMicros(command);
  Motor3WriteMicros(command);
  Motor4WriteMicros(command);
  Motor5WriteMicros(command);//set the output compare value
  Motor6WriteMicros(command);
  Motor7WriteMicros(command);
  Motor8WriteMicros(command);  
}

