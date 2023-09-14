#include  <avr/interrupt.h>
#include  <avr/io.h>
#include  <stdio.h>
#include  <math.h>
#include  <TimerOne.h>

#define AIRAWMIN  0
#define AIRAWMAX  1023
#define PWM_MIN   64
#define PWM_MAX   254

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

  // ***************************************
  // PRE-SCALAR Adjustment for system timers
  //
  // Timer 1 (TCCR1B)
  // Timer 2 (TCCR2B)
  // Timer 3 (TCCR3B)
  // Timer 4 (TCCR4B)
  // Timer 5 (TCCR5B)
  //
  //  0x01 (0b001) = 31250.00 Hz
  //  0x02 (0b010) =  3906.25 Hz
  //  0x03 (0b011) =   488.28 Hz
  //  0x04 (0b100) =   122.07 Hz
  //  0x05 (0b101) =    30.52 Hz
  //
  //  Example;
  //  change timer 2 to 122.07 Hz
  //
  //  TCCR2B &= 0b11111000;
  //  TCCR2B |= 0b00000100;
  //
  // ***************************************

  TCCR3B &= 0b11111000;
  TCCR3B |= 0b00000011;

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

  swMOD1.sunMOTORspeedOUT = map(xVAL,AIRAWMIN,AIRAWMAX,PWM_MIN,PWM_MAX);

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
