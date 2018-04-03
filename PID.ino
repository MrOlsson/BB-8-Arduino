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
//functions included from motor shield library 
/* void init() 
 *    Initialize pinModes and timer1.
 * void setM1Speed(int speed) 
 *    Set speed and direction for motor 1. Speed should be between -400 and 400. 400 corresponds to motor current flowing from M1A to M1B. -400 corresponds to motor current flowing from M1B to M1A. 0 corresponds to full coast.
 * void setM2Speed(int speed) 
 *    Set speed and direction for motor 2. Speed should be between -400 and 400. 400 corresponds to motor current flowing from M2A to M2B. -400 corresponds to motor current flowing from M2B to M2A. 0 corresponds to full coast.
 * void setSpeeds(int m1Speed, int m2Speed) 
 *    Set speed and direction for motor 1 and 2.
 * void setM1Brake(int brake) 
 *    Set brake for motor 1. Brake should be between 0 and 400. 0 corresponds to full coast, and 400 corresponds to full brake.
 * void setM2Brake(int brake) 
 *    Set brake for motor 2. Brake should be between 0 and 400. 0 corresponds to full coast, and 400 corresponds to full brake.
 * void setBrakes(int m1Brake, int m2Brake) 
 *    Set brake for motor 1 and 2.
 * unsigned int getM1CurrentMilliamps() 
 *    Returns current reading from motor 1 in milliamps. See the notes in the "Current readings" section below.
 * unsigned int getM2CurrentMilliamps() 
 *    Returns current reading from motor 2 in milliamps. See the notes in the "Current readings" section below.
 * unsigned char getM1Fault() 
 *    Returns 1 if there is a fault on motor driver 1, 0 if no fault.
 * unsigned char getM2Fault() 
 *    Returns 1 if there is a fault on motor driver 2, 0 if no fault.
 */

// Define PID regulator parameters and variables
int PIDSampleTime = 100;
long cur_t;
long last_t;
//#define constant 200
//Define the aggressive and conservative Tuning Parameters
//double aggKp=4 * constant, aggKi=0.2 * 1, aggKd=1 * constant;
//double consKp=1 * constant, consKi=0.05 * 1, consKd=0.25 * constant;

#define constant 400 / 12
double aggKp=0, aggKi=4.2, aggKd=0;
double consKp=0, consKi=4.2, consKd=0;

double L_Input, L_Output, L_Setpoint, L_Gap, R_Input, R_Output, R_Setpoint, R_Gap;

double omega_till_hast = 0.25*0.27/(2*0.045*4); // R*L/(2*r*N)
#define MaximumVelocity 1 //2
#define MaximumOmega 2

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
  
  L_Setpoint = (MaximumVelocity * input.velocity + omega_till_hast * input.omega * MaximumOmega);
  R_Setpoint = (MaximumVelocity * input.velocity - omega_till_hast * input.omega * MaximumOmega);

  L_Input = getRads().L_wheel;
  R_Input = getRads().R_wheel;

  stopIfFault();
  

  L_Gap = L_Setpoint - L_Input; // distance away from setpoint
  R_Gap = R_Setpoint - R_Input;

  if(abs(L_Gap) < 10)
  {  //we're close to setpoint, use conservative tuning parameters
    L_PID.SetTunings(consKp, consKi, consKd);
  }
  else
  {//we're far from setpoint, use aggressive tuning parameters
     L_PID.SetTunings(aggKp, aggKi, aggKd);
  }

  if(abs(R_Gap) < 10) // Byt ut 1 < 10 till gap < 10
  {  //we're close to setpoint, use conservative tuning parameters
    R_PID.SetTunings(consKp, consKi, consKd);
  }
  else
  {//we're far from setpoint, use aggressive tuning parameters
     R_PID.SetTunings(aggKp, aggKi, aggKd);
  }

  L_PID.Compute();
  R_PID.Compute();

  //setMotorSpeed(L_Output, R_Output);
  setMotorSpeed(L_Output * constant, R_Output * constant);
}

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

void setMotorSpeed(int LeftSpeed, int RightSpeed){ //set speed > 400 to keep old motor speed
  if(LeftSpeed > -400 && LeftSpeed < 400)
    md.setM1Speed(LeftSpeed);
  if(RightSpeed > -400 && RightSpeed < 400)
    md.setM2Speed(RightSpeed);
}

void printPidIO(bool newline){
  #ifdef BluetoothViewer
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

