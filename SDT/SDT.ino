#include  <avr/interrupt.h>
#include  <avr/io.h>
#include  <stdio.h>
#include  <math.h>
#include  <TimerOne.h>

#define pi  3.141592654

#define AI_SPAN           1023
#define JOY_STICK_SPAN    200
#define JOY_STICK_EGL     100

#define DT_TRACK_WIDTH    100
#define DT_WHEEL_BASE     100

#define MOD1_SUN_MOTOR_OUT  5
#define MOD1_RING_MOTOR_OUT 6
#define MOD2_SUN_MOTOR_OUT  7
#define MOD2_RING_MOTOR_OUT 8

#define JOYSTICK_MOVE_IN    A0
#define JOYSTICK_STRAFE_IN  A1
#define JOYSTICK_ROTATE_IN  A2

bool TimePulse;

struct joystick{
  int     yAXISraw;             // analog input from joystick 1 for forward and reverse
  int     xAXISraw;             // analog input from joystick 1 for left and right
  float   yAXISreq;
  float   xAXISreq;
};

struct  swerveModule {
  unsigned  int   sunPULSEcount;
  unsigned  long  sunPULSEtime;
  unsigned  long  sunPULSElast;
  unsigned  long  sunRPS;
  unsigned  int   ringPULSEcount;
  unsigned  long  ringPULSEtime;
  unsigned  long  ringPULSElast;
  unsigned  long  ringRPS;
};

struct            joystick      joy1;
struct            joystick      joy2;
volatile  struct  swerveModule  swMOD1;
volatile  struct  swerveModule  swMOD2;

bool  firstPASS;

const byte  mod1sunTACHpin = 19;
const byte  mod2ringTACHpin = 18;
int speedREQ = 512;
int speedINC = 0;

void  read_joysticks() {
  
  // read and condition information from each of the control joysticks

  bool  j1_FwdRvs;
  bool  j1_LeftRight;
  bool  j2_Rotate_CW;
  bool  j2_ROtate_CCW;
  char  disp[200];

  joy1.yAXISraw = analogRead(JOYSTICK_MOVE_IN);       // raw analog input for Forward/Reverse
  joy1.xAXISraw = analogRead(JOYSTICK_STRAFE_IN);   // raw analog input for STRAFE

  joy1.yAXISreq = (float(joy1.yAXISraw)/AI_SPAN) * JOY_STICK_SPAN - JOY_STICK_EGL;
  joy1.xAXISreq = (float(joy1.xAXISraw)/AI_SPAN) * JOY_STICK_SPAN - JOY_STICK_EGL;
}

void  calc_module_ctrl(float FWD, float STR, float RCW, float GYRO) {

  // calculate the speed and angle required for each swerve module

  float L;  // the vehicles wheelbase
  float W;  // the vehicles traackwidth;
  float R;  // ratio of wheelbase to trackwidth

  float ws1,ws2,ws3,ws4;
  float wa1,wa2,wa3,wa4;
  float max_ws;

  float xSTR,xFWD;

  float A,B,C,D;  // intermedary variable to calculate speed and direction

  char  disp[200];

  xFWD = FWD * cos(GYRO) + STR * sin(GYRO);
  xSTR = -1 * FWD * sin(GYRO) + STR * cos(GYRO);

  W = DT_TRACK_WIDTH;
  L = DT_WHEEL_BASE;
  R = sqrt((W*W)+(L*L));

  A = STR - RCW * (L/R);
  B = STR + RCW * (L/R);
  C = FWD - RCW * (W/R);
  D = FWD + RCW * (W/R);

  ws1 = sqrt(B*B + C*C);
  ws2 = sqrt(B*B + D*D);
  ws3 = sqrt(A*A + D*D);
  ws4 = sqrt(A*A + C*C);

  wa1 = atan2(B,C) * 180.0/pi -45;
  wa2 = atan2(B,D) * 180.0/pi -45;
  wa3 = atan2(A,D) * 180.0/pi -45;
  wa4 = atan2(A,C) * 180.0/pi -45;

  sprintf(disp,"FWD:%3d STR:%3d Wheel1(%3d,%3d) Wheel2(%3d,%3d) Wheel3(%3d,%3d) Wheel4(%3d,%3d)",int(FWD),int(STR),int(ws1),int(wa1),int(ws2),int(wa2),int(ws3),int(wa3),int(ws4),int(wa4));
  Serial.println(disp);

  max_ws = ws1;
  if (ws2 > max_ws) max_ws = ws2;
  if (ws3 > max_ws) max_ws = ws3;
  if (ws4 > max_ws) max_ws - ws4;

  if (max_ws > 1){
    ws1 /= max_ws;
    ws2 /= max_ws;
    ws3 /= max_ws;
    ws4 /= max_ws;
  }
}

// This is the interrupt routine executed when a TACH pulse is detected for the Ring Gear Motor

void mod1sunTACHpulse(){
  swMOD1.sunPULSEcount++;  
}
void mod2ringTACHpulse(){
  swMOD2.ringPULSEcount++;  
}
void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  TimePulse = 1;
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(5,OUTPUT);
  pinMode(53,INPUT);
  pinMode(30,OUTPUT);
  pinMode(32,OUTPUT);
  pinMode(34,OUTPUT);

  Timer1.initialize(1000000); // set timer for 0.10sec
  Timer1.attachInterrupt( timerIsr ); // enable the timer

  //pinMode(mod2RingTachPin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(mod1sunTACHpin),mod1sunTACHpulse,RISING);
  attachInterrupt(digitalPinToInterrupt(mod2ringTACHpin),mod2ringTACHpulse,RISING);
  firstPASS = 1;
  swMOD2.sunPULSEcount = 0;
  swMOD2.ringPULSEcount = 0;
  TimePulse = 0;
}

float xpot=512;
float ypot=512;

void loop() {
  // put your main code here, to run repeatedly:

  //float tempRPMcalc;
  //int yVAL, xVAL;

//  read_joysticks();
//  calc_module_ctrl(joy1.yAXISreq, joy1.xAXISreq, 0, 0);

  //xVAL = analogRead(A1);

  analogWrite(MOD1_SUN_MOTOR_OUT,512/1023.0*180.0+65);
//  analogWrite(MOD1_RING_MOTOR_OUT,512/1023.0*180.0+65);
//  analogWrite(MOD2_SUN_MOTOR_OUT,512/1023.0*180.0+65);
  analogWrite(MOD2_RING_MOTOR_OUT,512/1023.0*180.0+65);



//xpot=xpot*0.95+0.05*analogRead(A3);
//ypot=ypot*0.95+0.05*analogRead(A4);

//Serial.println(xpot);

//analogWrite(MOD2_RING_MOTOR_OUT,xpot/1023.0*180.0+65);
//analogWrite(MOD2_SUN_MOTOR_OUT,ypot/1023.0*180.0+65);
//analogWrite(MOD1_SUN_MOTOR_OUT,xpot/1023.0*180.0+65);
//analogWrite(MOD1_RING_MOTOR_OUT,ypot/1023.0*180.0+65);

  if (TimePulse == 1){
    swMOD1.sunRPS = swMOD1.sunRPS*.8 + (((swMOD1.sunPULSEcount/6)-swMOD1.sunRPS)*.2);
    Serial.print(0);
    Serial.print(",");
    Serial.print(swMOD1.sunRPS);
    Serial.print(",");
    Serial.print(swMOD1.sunPULSEcount);
    Serial.print(",");
    Serial.println(300);
  
    swMOD1.sunPULSEcount = 0;
    swMOD1.ringPULSEcount = 0;
    swMOD2.sunPULSEcount = 0;
    swMOD2.ringPULSEcount = 0;
    TimePulse = 0;
    speedINC++;
  }

  if (speedINC > 20){
    speedREQ += 50;
    speedINC = 0;
    if (speedREQ >= 1000) speedREQ = 512;
  }

}

