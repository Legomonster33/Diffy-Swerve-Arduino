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




}

void loop() {
  // put your main code here, to run repeatedly:

}
