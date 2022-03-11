// ServoMotor.c
// Evon and Thomas Lab 6
// 3/8/22

#include "msp.h"
#include <stdint.h>
#include <stdio.h>
#include "Common.h"

void Servo_Init()
{
	// Initialize pins 
	
	P4->SEL0 |= BIT1;
	P4->SEL1 |= BIT1;
	P4->DIR |= BIT1;
  P4->SEL0 |= BIT2;
	P4->SEL1 |= BIT2;
	P4->DIR |= BIT2;
	P4->SEL0 |= BIT3;
	P4->SEL1 |= BIT3;
	P4->DIR |= BIT3;
	P4->SEL0 |= BIT4;
	P4->SEL1 |= BIT4;
	P4->DIR |= BIT4;
	
}
