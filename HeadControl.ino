#define ROTHEADPIN 46 
#define SAMPLE_T 100
#include <Servo.h>

Servo rotServo;
unsigned long oldRotTime;

void rotSetup() {
  rotServo.attach(ROTHEADPIN);
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

