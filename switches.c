/*
 * File:        switches.c
 * Purpose:     switche Functions
 *
 */
#include "msp.h"
#include "switches.h" 
#include "Common.h"

void Switch1_Init(void){
	// configure PortPin for Switch 1 and Switch2 as port I/O 
     P1->SEL0 &= ~BIT1;
	   P1->SEL1 &= ~BIT1;
	   
	// configure as input
     P1->DIR &= ~BIT1;
	   P1->REN |= BIT1;
              
}

void Switch2_Init(void){
	// Configure port and pins
     P1->SEL0 &= ~BIT4;
	   P1->SEL1 &= ~BIT4;
	// configure as input
     P1->DIR &= ~BIT4;
	   P1->REN |= BIT4;	
}
