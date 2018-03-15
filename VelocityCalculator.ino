#include <TimerThree.h>
#include "Types.h"

//Left motor sensor to 18 and 19
//Right motor sensor to 20 and 21

#define irq2pinA 18
#define irq2pinB 19
#define irq1pinA 20
#define irq1pinB 21

const signed int encoderDirections[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
volatile long encCount1 = 0;
volatile long encCount2 = 0;

double rotation1;
double rotation2;

RADS rads;

void TimerIsr(){
  Timer3.detachInterrupt();
  rads.L_wheel = (encCount1 / (64.0 * 50.0)) * 10 * 2 * 3.14; // 64 pukter på hall sensorn och gearboxen är på 50:1, 2*pi for radians per secound and lastly figure out why 10 is here...
  rads.R_wheel = - (encCount2 / (64.0 * 50.0)) * 10 * 2* 3.14;
  
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
  pinMode(irq1pinA, INPUT_PULLUP);
  pinMode(irq1pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(irq1pinA), enc_irq1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(irq1pinB), enc_irq1, CHANGE);
  
  pinMode(irq2pinA, INPUT_PULLUP);
  pinMode(irq2pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(irq2pinA), enc_irq2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(irq2pinB), enc_irq2, CHANGE);
  Timer3.initialize(10000); // 1 sec timer
  Timer3.attachInterrupt(TimerIsr);

  Serial.println("Start");
}

void enc_irq1() {
  static unsigned int oldEncoderState1 = 0;
  oldEncoderState1 <<= 2;
  oldEncoderState1 |= ((PIND >> 2) & 0x03);
  encCount1 += encoderDirections[(oldEncoderState1 & 0x0F)];
}

void enc_irq2() {
  static unsigned int oldEncoderState2 = 0;
  oldEncoderState2 <<= 2;
  oldEncoderState2 |= ((PIND >> 0) & 0x03);
  encCount2 += encoderDirections[(oldEncoderState2 & 0x0F)];
}

struct RADS getRads(){
  return rads;
}

