#include <PID_v1.h>
#include <DualVNH5019MotorShield.h>
#include "Types.h"

//Define Motor Shield variables and constants
DualVNH5019MotorShield md;
// Left motor == Motor 1
// Right motor == Motor 2
//#define MotorMaxValue 400
#define MotorMaxValue 12
#define BluetoothViewer
#define angularBoost 3
//functions included from motor shield library 

// Define PID regulator parameters and variables
int PIDSampleTime = 100;
long cur_t;
long last_t;


#define voltageToMotorspeed 400/12              //parameter for att konvertera spänningen till input till motorkontrollern.
double aggKp=3.6994, aggKi = 14.57, aggKd=-0.711;
double consKp=2, consKi=4.2, consKd=0;

double L_Input, L_Output = 0, L_Setpoint, L_Gap, R_Input, R_Output = 0, R_Setpoint, R_Gap;


double omega_till_hast = 0.25*0.27/(2*0.045*4); // R*L/(2*r*N)
#define wheelrotationToSphereSpeed 0.05 

#define MaximumVelocity 0.30


//Specify the links and initial tuning parameters
PID L_PID(&L_Input, &L_Output, &L_Setpoint, consKp, consKi, consKd, DIRECT);
PID R_PID(&R_Input, &R_Output , &R_Setpoint, consKp, consKi, consKd, DIRECT);


void PIDSetup(){
  md.init();
  //turn the PID on
  L_PID.SetMode(AUTOMATIC);
  R_PID.SetMode(AUTOMATIC);

  /*L_PID.SetSampleTime(1000);
  R_PID.SetSampleTime(1000);*/
  L_PID.SetOutputLimits(-MotorMaxValue, MotorMaxValue);
  R_PID.SetOutputLimits(-MotorMaxValue, MotorMaxValue);
  
  L_PID.SetSampleTime(PIDSampleTime);
  R_PID.SetSampleTime(PIDSampleTime);

}

void PIDLoop(){
  
  L_Setpoint = (MaximumVelocity * input.velocity);
  R_Setpoint = (MaximumVelocity * input.velocity);
  
  L_Input = getRads().L_wheel * wheelrotationToSphereSpeed;
  R_Input = getRads().R_wheel * wheelrotationToSphereSpeed;

  stopIfFault();

  L_Gap = L_Setpoint - L_Input; // distance away from setpoint
  R_Gap = R_Setpoint - R_Input;

  if(abs(L_Gap) > 0.02 && L_Setpoint)
  {  //we're close to setpoint, use conservative tuning parameters
    L_PID.SetTunings(consKp, consKi, consKd);
  }
  else
  {//we're far from setpoint, use aggressive tuning parameters
     L_PID.SetTunings(aggKp, aggKi, aggKd);
  }

  if(abs(R_Gap) < 0.02 && L_Setpoint) // Byt ut 1 < 10 till gap < 10
  {  //we're close to setpoint, use conservative tuning parameters
    R_PID.SetTunings(consKp, consKi, consKd);
  }
  else
  {//we're far from setpoint, use aggressive tuning parameters
     R_PID.SetTunings(aggKp, aggKi, aggKd);
  }

  L_PID.Compute();
  R_PID.Compute();
  
  setMotorSpeed(L_Output * voltageToMotorspeed + input.omega*angularBoost * voltageToMotorspeed, R_Output * voltageToMotorspeed - angularBoost * input.omega * voltageToMotorspeed);
}

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial2.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial2.println("M2 fault");
    while(1);
  }
}

void setMotorSpeed(int LeftSpeed, int RightSpeed){ //set speed > 400 to keep old motor speed
  LeftSpeed = constrain(LeftSpeed, -400, 400);
  md.setM1Speed(LeftSpeed);
  
  RightSpeed = constrain(RightSpeed, -400, 400);
  md.setM2Speed(RightSpeed);
}

void printPidIO(bool newline){
  #ifdef BluetoothViewer
  Serial2.print(getOutput());
  Serial2.print(" ");
  
  //Serial.print("Input: ");
  Serial2.print(L_Input);
  Serial2.print(" ");
  Serial2.print(R_Input);
  Serial2.print(" ");

  //Serial.print(" Setpoint: ");
  Serial2.print(L_Setpoint);
  Serial2.print(" ");
  Serial2.print(R_Setpoint);
  Serial2.print(" ");

  //Serial.print(" Gap: ");
  Serial2.print(L_Gap);
  Serial2.print(" ");
  Serial2.print(R_Gap);
  Serial2.print(" ");

  //Serial.print(" Output: ");
  Serial2.print(L_Output);
  Serial2.print(" ");
  Serial2.print(R_Output);

  if(newline)
    Serial2.println();
  #else
  Serial.print(getOutput());
  Serial.print(" ");
  
  //Serial.print("Input: ");
  Serial.print(L_Input);
  Serial.print(" ");
  Serial.print(R_Input);
  Serial.print(" ");

  //Serial.print(" Setpoint: ");
  Serial.print(L_Setpoint);
  Serial.print(" ");
  Serial.print(R_Setpoint);
  Serial.print(" ");

  //Serial.print(" Gap: ");
  Serial.print(L_Gap);
  Serial.print(" ");
  Serial.print(R_Gap);
  Serial.print(" ");

  //Serial.print(" Output: ");
  Serial.print(L_Output);
  Serial.print(" ");
  Serial.print(R_Output);

  if(newline)
    Serial.println();
  #endif
}

