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


void setup() {

  // put your setup code here, to run once:

  // Set the PWM pin assignments for sun and ring motors for each module

  swMOD1.sunMOTORpinOUTPUT = 2;
  swMOD1.ringMOTORpinOUTPUT = 3;
  swMOD2.sunMOTORpinOUTPUT = 6;
  swMOD2.ringMOTORpinOUTPUT = 7;

  // Adjust system timer 3 prescalar from default of 490 hz (0b00000011) to 122 hz (0b00000100)
  // (spark motor controller required PWM from 16 hz to 200 hz, default from system timer is 490hz, too high)

  TCCR3B &= 0b11111000;
  TCCR3B |= 0b00000100;

  // Adjust system timer 3 prescalar from default of 490 hz (0b00000011) to 122 hz (0b00000100)
  // (spark motor controller required PWM from 16 hz to 200 hz, default from system timer is 490hz, too high)

  TCCR4B &= 0b11111000;
  TCCR4B |= 0b00000100;

}

void loop() {
  // put your main code here, to run repeatedly:

}
