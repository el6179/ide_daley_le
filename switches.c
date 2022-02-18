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
	   P1->OUT |= BIT1;
	   P1->REN |= BIT1;
              
}

void Switch2_Init(void){
	// Configure port and pins
     P1->SEL0 &= ~BIT4;
	   P1->SEL1 &= ~BIT4;
	// configure as input
     P1->DIR &= ~BIT4;
	   P1->OUT |= BIT4;
	   P1->REN |= BIT4;	
}

//------------Switch_Input------------
// Read and return the status of Switch1
// Input: none
// return: TRUE if pressed
//         FALSE if not pressed
BOOLEAN Switch1_Pressed(void)
{
	BOOLEAN retVal = FALSE;
	// check if pressed
	if (P1->OUT & BIT1){
		retVal =  TRUE;
	}

	return (retVal);              // return TRUE(pressed) or FALSE(not pressed)
}
//------------Switch_Input------------
// Read and return the status of Switch2
// Input: none
// return: TRUE if pressed
//         FALSE if not pressed
BOOLEAN Switch2_Pressed(void)
{
	BOOLEAN retVal = FALSE;
	// check if pressed
	if (P1->OUT & BIT4){
		retVal = TRUE;
	}

	return (retVal);              // return TRUE(pressed) or FALSE(not pressed)
}
