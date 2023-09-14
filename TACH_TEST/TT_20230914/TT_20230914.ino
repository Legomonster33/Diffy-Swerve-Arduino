#include  <avr/interrupt.h>
#include  <avr/io.h>
#include  <stdio.h>
#include  <math.h>
#include  <TimerOne.h>

struct  swerveModule {
  int             sunENCODERwindows;
  unsigned  int   sunPULSEcount;
  unsigned  long  sunPULSEtime;
  unsigned  long  sunPULSElast;
  unsigned  long  sunRPS;
  int             sunMOTORspeedREQ;
  float           sunMOTORspeedOUT;
  unsigned  int   sunMOTORpinOUTPUT;
  int             ringENCODERwindows;
  unsigned  int   ringPULSEcount;
  unsigned  long  ringPULSEtime;
  unsigned  long  ringPULSElast;
  unsigned  long  ringRPS;
  int             ringMOTORspeedREQ;
  float           ringMOTORspeedOUT;
  unsigned  int   ringMOTORpinOUTPUT;
};

volatile  struct  swerveModule  swMOD1;
volatile  struct  swerveModule  swMOD2;

int xVAL;
int yVAL;
int TimePulse;

void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  TimePulse = 1;
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}


void setup() {
  //
  // put your setup code here, to run once:
  //

  // Set the PWM pin assignments for sun and ring motors for each module
  swMOD1.sunMOTORpinOUTPUT = 2;
  swMOD1.ringMOTORpinOUTPUT = 3;
  swMOD2.sunMOTORpinOUTPUT = 6;
  swMOD2.ringMOTORpinOUTPUT = 7;

  // configure the PWM pins to be output for each spark motor controller
  pinMode (swMOD1.sunMOTORpinOUTPUT, OUTPUT);
  pinMode (swMOD1.ringMOTORpinOUTPUT, OUTPUT);
  pinMode (swMOD2.sunMOTORpinOUTPUT, OUTPUT);
  pinMode (swMOD2.ringMOTORpinOUTPUT, OUTPUT);

  // Adjust system timer 3 prescalar from default of 490 hz (0b011 / 0x03) to 122 hz (0b100/ 0x04)
  // (spark motor controller requires PWM from 16 hz to 200 hz, default from system timer is 490hz, too high)
  TCCR3B &= 0b11111000;
  TCCR3B |= 0b00000011;

  // Adjust system timer 4 prescalar from default of 490 hz (0b11 / 0x03) to 122 hz (0b100 / 0x04)
  // (spark motor controller requires PWM from 16 hz to 200 hz, default from system timer is 490hz, too high)
  TCCR4B &= 0b11111000;
  TCCR4B |= 0b00000011;

  Serial.begin(9600);

  Timer1.initialize(100000); // set timer for 0.10sec
  Timer1.attachInterrupt( timerIsr ); // enable the timer
}

void loop() {
  // put your main code here, to run repeatedly:

  xVAL = analogRead(A1);
  yVAL = analogRead(A2);

  swMOD1.sunMOTORspeedOUT = map(xVAL,0,1023,128,240);

  if (TimePulse == 1) {
    Serial.print(xVAL);
    Serial.print(" , ");
    Serial.print(swMOD1.sunMOTORspeedOUT);
    Serial.print(" [");
    Serial.print(swMOD1.sunMOTORpinOUTPUT);
    Serial.println("]");
    TimePulse = 0;
  }

  analogWrite(swMOD1.sunMOTORpinOUTPUT,swMOD1.sunMOTORspeedOUT);

}
