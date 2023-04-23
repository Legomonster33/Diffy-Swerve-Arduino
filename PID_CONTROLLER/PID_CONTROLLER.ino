float AvgMicrosPerPulse[8],MicrosPerRevolution[8],Speed[8],PotVal[8],Rpm[8],TachoDeltaTimePrevious[8],MicrosPerPulse[8],TachoTimer[8],TachoValue[8],OldTachoValue[8],TachoDeltaTime[8],ControlValue[12],ProcessValue[12],SetPoint[12],P[12],I[12],D[12],MaxSummativeError[12],Error[12],LastError[12],ErrorSummative[12],PreviousTime[12],DeltaTime[12],Proportional,Integral,Derivative;

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
  Serial.println(Rpm[Idx]);
  }
  OldTachoValue[Idx] = TachoValue[Idx];
  if(TachoTimer[Idx]<micros()-500000){TachoTimer[Idx]=micros();}
}

void ReadPot(int Pin,int Idx){
  PotVal[Idx] = PotVal[Idx]*0.99+0.01*((((analogRead(Pin)/512.0)-1.0)));
  //Serial.println(PotVal[Idx]);
}

void SpeedCalc(int Idx){
  Speed[Idx] = (PotVal[Idx]*92.5)+157.5;
  //Serial.println(Speed[Idx]);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
}

void loop() {
  ReadRpm(0);
  ReadPot(A0,0);
  SpeedCalc(0);
  analogWrite(2, Speed[0]);

}