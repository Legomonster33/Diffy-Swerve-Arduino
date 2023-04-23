float AvgMicrosPerPulse[8],
MicrosPerRevolution[8],
MotorOutPut[8],
PotVal[8],
Rpm[8],
TachoDeltaTimePrevious[8],
MicrosPerPulse[8],
TachoTimer[8],
TachoValue[8],
OldTachoValue[8],
TachoDeltaTime[8],
ControlValue[12],
ProcessValue[12],
SetPoint[12],
P[12],
I[12],
D[12],
MaxSummativeError[12],
Error[12],
LastError[12],
ErrorSummative[12],
PreviousTime[12],
DeltaTime[12],
MaxControl[12],
Proportional,
Integral,
Derivative;

void PID(int Idx){
  Error[Idx] = SetPoint[Idx] - ProcessValue[Idx];
  Proportional = P[Idx]*Error[Idx];

  ErrorSummative[Idx] = ErrorSummative[Idx] + Error[Idx];
  if(ErrorSummative[Idx]>MaxSummativeError[Idx]){ErrorSummative[Idx]=MaxSummativeError[Idx];}
  if(ErrorSummative[Idx]<MaxSummativeError[Idx]*-1){ErrorSummative[Idx]=MaxSummativeError[Idx]*-1;}

  DeltaTime[Idx] = micros() - PreviousTime[Idx];

  Integral = DeltaTime[Idx]*I[Idx]*ErrorSummative[Idx];

  Derivative = ((D[Idx]/DeltaTime[Idx])/(Error[Idx]-LastError[Idx]));

  ControlValue[Idx] = Proportional + Integral + Derivative;
  if(ControlValue[Idx]>MaxControl[Idx]){ControlValue[Idx]=MaxControl[Idx];}
  if(ControlValue[Idx]<-1*MaxControl[Idx]){ControlValue[Idx]=-1*MaxControl[Idx];}
}


#define PulsesPerRevolution 6
#define MicrosecondsPerMinute 60000000.0



void ReadRpm(int Idx){
  TachoValue[Idx] = digitalRead(Idx+30);
  //Serial.println(TachoValue[Idx]);
if(TachoValue[Idx] != OldTachoValue[Idx]){
  TachoDeltaTime[Idx] = micros()-TachoTimer[Idx];  
  AvgMicrosPerPulse[Idx] = TachoDeltaTime[Idx]*0.5+TachoDeltaTimePrevious[Idx]*0.5;
  //micros per pulse

  MicrosPerRevolution[Idx] = AvgMicrosPerPulse[Idx]*PulsesPerRevolution;
  //micros per rotation

  Rpm[Idx] = MicrosecondsPerMinute/MicrosPerRevolution[Idx];
  // divide how many microseconds per minute by how many microseconds per revo to get rotations per minute
      
  TachoTimer[Idx] = micros();
  TachoDeltaTimePrevious[Idx] = TachoDeltaTime[Idx];
  //Serial.println(Rpm[Idx]);
  }
  OldTachoValue[Idx] = TachoValue[Idx];
}

void ReadPot(int Pin,int Idx){
  PotVal[Idx] = PotVal[Idx]*0.99+0.01*((((analogRead(Pin)/512.0)-1.0)));
  //Serial.println(PotVal[Idx]);
}

void MotorOutPutCalc(int Idx){
  MotorOutPut[Idx] = MotorOutPut[Idx] + MotorOutPut[Idx]*ControlValue[Idx];
  if(MotorOutPut[Idx]<250){MotorOutPut[Idx]=250;}
  if(MotorOutPut[Idx]>60){MotorOutPut[Idx]=60;}  
  //Serial.println(MotorOutPut[Idx]);
}

void PotRead(int PinNum,int Idx){
  PotVal[Idx] = PotVal[Idx]*0.9+0.1*(((analogRead(PinNum)-512.0)/512.0));
  Serial.println(PotVal[Idx]);
  //Serial.println(analogRead(PinNum));
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  MotorOutPut[0]=160;
  P[0]=1;
  I[0]=0;
  D[0]=0;
  MaxSummativeError[0] = 1;
  MaxControl[0]=1;
  
}

void loop() {
  ReadRpm(0);
  ReadPot(A0,0);
  MotorOutPutCalc(0);
  PotRead(A0,0);
  analogWrite(2, MotorOutPut[0]);
  ProcessValue[0] = Rpm[0]/6000;
  SetPoint[0]=PotVal[0];
}