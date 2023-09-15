#include  <avr/interrupt.h>
#include  <avr/io.h>
#include  <stdio.h>
#include  <math.h>
#include  <TimerOne.h>

#define AIRAWMIN  0
#define AIRAWMAX  1023
#define PWM_MIN   63
#define PWM_MAX   254

struct  swerveModule {
  int             sunENCODERwindows;
  int             sunTACHpinINPUT;
  unsigned  int   sunPULSEcount;
  unsigned  long  sunPULSEtime;
  unsigned  long  sunPULSElast;
  unsigned  long  sunRPS;
  int             sunMOTORspeedREQ;
  float           sunMOTORspeedOUT;
  unsigned  int   sunMOTORpinOUTPUT;
  int             ringENCODERwindows;
  int             ringTACHpinINPUT;
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

// This interrupt is triggered once per second
// and causes the current tach pulse counts to
// be converted into a RPM value
void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  TimePulse = 1;
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}

// Interrupt routine to count Mod1 Sun Motor Pulses
void mod1sunTACHpulse(){
  swMOD1.sunPULSEcount++;  
}
// Interrupt routine to count Mod1 Ring Motor Pulses
void mod1ringTACHpulse(){
  swMOD1.ringPULSEcount++;  
}
// Interrupt routine to count Mod2 Sun Motor Pulses
void mod2sunTACHpulse(){
  swMOD2.sunPULSEcount++;  
}
// Interrupt routine to count Mod2 Ring Motor Pulses
void mod2ringTACHpulse(){
  swMOD2.ringPULSEcount++;  
}



void setup() {
  //
  // put your setup code here, to run once:
  //

  // set up the timer interrupt that triggers every 1 second (1000000 ms)
  Timer1.initialize(1000000); // set timer for 1sec
  Timer1.attachInterrupt( timerIsr ); // enable the timer

  // Set the PWM pin assignments for sun and ring motors for each module
  swMOD1.sunMOTORpinOUTPUT = 6;
  swMOD1.ringMOTORpinOUTPUT = 7;
  swMOD2.sunMOTORpinOUTPUT = 44;
  swMOD2.ringMOTORpinOUTPUT = 45;

  // configure the PWM pins to be output for each spark motor controller
  pinMode (swMOD1.sunMOTORpinOUTPUT, OUTPUT);
  pinMode (swMOD1.ringMOTORpinOUTPUT, OUTPUT);
  pinMode (swMOD2.sunMOTORpinOUTPUT, OUTPUT);
  pinMode (swMOD2.ringMOTORpinOUTPUT, OUTPUT);

  // Set the pin assignments for the sun and ring motor TACH inputs
  swMOD1.sunTACHpinINPUT = 2;
  swMOD1.ringTACHpinINPUT = 3;
  swMOD2.sunTACHpinINPUT = 18;
  swMOD2.ringTACHpinINPUT = 19;

  // configure the TACH input pins to have a pull up resistor
  pinMode(swMOD1.sunTACHpinINPUT,INPUT_PULLUP);
  pinMode(swMOD1.ringTACHpinINPUT,INPUT_PULLUP);
  pinMode(swMOD2.sunTACHpinINPUT,INPUT_PULLUP);
  pinMode(swMOD2.ringTACHpinINPUT,INPUT_PULLUP);

  // assign interrupt routines to each of the TACH inputs
  attachInterrupt(digitalPinToInterrupt(swMOD1.sunTACHpinINPUT),mod1sunTACHpulse,RISING);
  attachInterrupt(digitalPinToInterrupt(swMOD1.ringTACHpinINPUT),mod1ringTACHpulse,RISING);
  attachInterrupt(digitalPinToInterrupt(swMOD2.sunTACHpinINPUT),mod2sunTACHpulse,RISING);
  attachInterrupt(digitalPinToInterrupt(swMOD2.ringTACHpinINPUT),mod2ringTACHpulse,RISING);
  
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


}

void loop() {
  // put your main code here, to run repeatedly:

  xVAL = analogRead(A0);
  yVAL = analogRead(A1);


  swMOD1.sunMOTORspeedOUT = map(xVAL,AIRAWMIN,AIRAWMAX,PWM_MIN,PWM_MAX);
  swMOD1.ringMOTORspeedOUT = map(yVAL,AIRAWMIN,AIRAWMAX,PWM_MIN,PWM_MAX);
  swMOD2.sunMOTORspeedOUT = map(xVAL,AIRAWMIN,AIRAWMAX,PWM_MIN,PWM_MAX);
  swMOD2.ringMOTORspeedOUT = map(yVAL,AIRAWMIN,AIRAWMAX,PWM_MIN,PWM_MAX);


  if (TimePulse == 1) {

    // Convert the current pulse count to Rev per sec and then 
    // calculate a new RPS which equates to 80% of the current ready plus 20% of the new reading
    swMOD1.sunRPS = (swMOD1.sunRPS * .8) + ((swMOD1.sunPULSEcount | 3)*.2);
    swMOD1.ringRPS = (swMOD1.ringRPS * .8) + ((swMOD1.ringPULSEcount | 3)*.2);
    swMOD2.sunRPS = (swMOD2.sunRPS * .8) + ((swMOD2.sunPULSEcount | 3)*.2);
    swMOD2.ringRPS = (swMOD2.ringRPS * .8) + ((swMOD2.ringPULSEcount | 3)*.2);

    Serial.print("0,");
    Serial.print(swMOD1.sunRPS);
    Serial.print(",");
    Serial.print(swMOD1.ringRPS);
    Serial.print(",");
    Serial.print(swMOD2.sunRPS);
    Serial.print(",");
    Serial.print(swMOD2.ringRPS);
    Serial.println(",200");



    // Clear the TACH pulse counts each time the Timer Interrupt triggers
    swMOD1.sunPULSEcount = 0;
    swMOD1.ringPULSEcount = 0;
    swMOD2.sunPULSEcount = 0;
    swMOD2.ringPULSEcount = 0;
    
    TimePulse = 0;
  }

  analogWrite(swMOD1.sunMOTORpinOUTPUT,swMOD1.sunMOTORspeedOUT);
  analogWrite(swMOD1.ringMOTORpinOUTPUT,swMOD1.ringMOTORspeedOUT);
  analogWrite(swMOD2.sunMOTORpinOUTPUT,swMOD2.sunMOTORspeedOUT);
  analogWrite(swMOD2.ringMOTORpinOUTPUT,swMOD2.ringMOTORspeedOUT);

}
