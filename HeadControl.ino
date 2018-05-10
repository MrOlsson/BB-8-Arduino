#define ROTHEADPIN 46 
#define TURNHEADPIN 47
#define SAMPLE_T 100
#define MAXANGLE 20 * 3.21
#define VTOSETPOINT 63/0.25
#define wheelrotationToSphereSpeed 0.05 
#include <Servo.h>
#include "Types.h"


Servo rotServo;
Servo turnServo;
unsigned long oldRotTime, oldTurnTime;
double HKp=0.2, HKi = 2.245519;
double I = 0, setpoint, output, e, P;

void rotSetup() {
  rotServo.attach(ROTHEADPIN);
  turnServo.attach(TURNHEADPIN);
  turnServo.write(90);
}

void turnLoop(){
  if(millis() > oldTurnTime + SAMPLE_T){
    setpoint = (getRads().L_wheel + getRads().R_wheel)* wheelrotationToSphereSpeed/2.0 * VTOSETPOINT;
    e = setpoint - turnServo.read()+90;
    I = I+HKi*e*SAMPLE_T/1000;
    P = HKp*e;
    output = constrain(I+P, -MAXANGLE, MAXANGLE);
    turnServo.write(90+output);
    Serial2.print(output);
    Serial2.print(" ");
    oldTurnTime = millis();
  }
}


void rotLoop() {
  if(millis() > oldRotTime + SAMPLE_T){
    if(input.head == 1){
       rotateRight();
    }else if(input.head == -1){
      rotateLeft(); 
    }else if(input.head == 0){
      stopRot();
    }else{
      stopRot();  
    }
    oldRotTime = millis();
  }
}

void rotateLeft(){
  short oldVal = rotServo.read();
  if(oldVal != 180){  
    rotServo.write(min(oldVal+20,180));
  }
}

void rotateRight(){
  short oldVal = rotServo.read();
  if(oldVal != 0){
    rotServo.write(max(oldVal-20,0));
  }
}

void stopRot(){
  short oldVal = rotServo.read();
  if(oldVal < 90){
    rotServo.write(min(oldVal+20,90));
  }else if(oldVal > 90){
    rotServo.write(max(oldVal-20,90));
  }
}

void testRotInput(){
  if(input.head == 1){
      digitalWrite(LEDPORT,HIGH);
    }else if(input.head == -1){
      digitalWrite(LEDPORT,HIGH);
      if(n>5){
        digitalWrite(LEDPORT, LOW);
      }if(n>9){
        n=0;
      }
      n++;
    }else{
      digitalWrite(LEDPORT,LOW);
    }
}

void testTurn(){
  if(millis() > oldTurnTime + SAMPLE_T){
    setpoint = input.velocity * 20 * 3.21;
    e = setpoint - turnServo.read()+90;
    I = I+HKi*e*SAMPLE_T/1000;
    P = HKp*e;
    output = constrain(I+P, -MAXANGLE, MAXANGLE);
    turnServo.write(90+output);
    Serial2.print(output);
    Serial2.print(" ");
    oldTurnTime = millis();
  }
}

