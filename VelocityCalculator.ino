#include <TimerThree.h>
#include "Types.h"

//Left motor sensor to 18 and 19
//Right motor sensor to 20 and 21

#define irq2pinA 18
#define irq2pinB 19
#define irq1pinA 20
#define irq1pinB 21

volatile long encCount1 = 0;
volatile long encCount2 = 0;

RADS rads;

void TimerIsr(){
  Timer3.detachInterrupt();
  rads.L_wheel = (encCount2 / (64.0 * 50.0)) *100 * 2 * 3.14; // 64 pukter på hall sensorn och gearboxen är på 50:1, 2*pi for radians per secound. Faktor 100 eftersom den mäter 100 ggr/s.
  rads.R_wheel = -(encCount1 / (64.0 * 50.0)) *100 * 2 * 3.14;
  
  //Debug, dont use Serial.print in Interrupts to avoid delays
  /*Serial.print(rads.L_wheel);
  Serial.print(" ");
  Serial.println(rads.R_wheel);*/
  encCount1 = 0;
  encCount2 = 0;
  Timer3.attachInterrupt(TimerIsr);
}

void VelocitySetup() {
  
  // put your setup code here, to run once:
  pinMode(irq1pinA, INPUT);
  pinMode(irq1pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(irq1pinA), enc_irq1, RISING);
  
  pinMode(irq2pinA, INPUT);
  pinMode(irq2pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(irq2pinA), enc_irq2, RISING);
  Timer3.initialize(10000); // 0.01 sec timer (10000 micros)
  Timer3.attachInterrupt(TimerIsr);

  Serial.println("Start");
}

  
void enc_irq1() {
  if(digitalRead(irq1pinB)){
    encCount1 += 1;
  }else{
    encCount1 -= 1;
  }
}

void enc_irq2() {
  
  if(digitalRead(irq2pinB)){
    encCount2 += 1;
  }else{
    encCount2 -= 1;
  }
}


struct RADS getRads(){
  return rads;
}

