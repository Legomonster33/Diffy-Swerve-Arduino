float Rpm[8],Rpm2[8],TachoTimer[8],TachoValue[8],OldTachoValue[8],Rpm1[8],ControlValue[12],ProcessValue[12],SetPoint[12],P[12],I[12],D[12],MaxSummativeError[12],Error[12],LastError[12],ErrorSummative[12],PreviousTime[12],DeltaTime[12],Proportional,Integral,Derivative;

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

void ReadRpm(int Idx){
  TachoValue[Idx] = digitalRead(Idx+20);
  //Serial.println(TachoValue[Idx]);
if(TachoValue[Idx] != OldTachoValue[Idx]){
  Rpm1[Idx] = 6000000/((micros()-TachoTimer[Idx])/1.5);
  Rpm[Idx] = Rpm1[Idx]*0.5+Rpm2[Idx]*0.5;
  TachoTimer[Idx] = micros();
  Serial.println(Rpm[Idx]+100);
  Rpm2[Idx] = Rpm1[Idx];
  }
  OldTachoValue[Idx] = TachoValue[Idx];
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
}

void loop() {
  ReadRpm(0);
  analogWrite(2, 160+30);

}