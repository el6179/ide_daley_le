/*
 * File:        LED.c
 * Purpose:     LED Functions
 *
 */
#include "msp.h"
#include "leds.h" 
#include "Common.h"

void LED1_Init(void)
{
	// configure PortPin for LED1 as port I/O 
  P1->SEL0 &= ~BIT0;
  P1->SEL1 &= ~BIT0;

	// make built-in LED1 LED high drive strength
  P1->DS |= BIT0;

	// make built-in LED1 out	 
  P1->DIR |= BIT0;

	// turn off LED
  P1->OUT &= ~BIT0;
}

void LED2_Init(void){
	// configure PortPin for LED2 as port I/O 
  P2->SEL0 &= ~BIT0;
  P2->SEL1 &= ~BIT0;
  
	// make built-in LED2 LEDs high drive strength
  P2->DS |= BIT0;
  P2->DS |= BIT1;
  P2->DS |= BIT2;

	// make built-in LED2 out	 
  P2->DIR |= BIT0;
  P2->DIR |= BIT1;
  P2->DIR |= BIT2;

	// turn off LED
  P2->OUT &= ~BIT0;
  P2->OUT &= ~BIT1;
  P2->OUT &= ~BIT2;
}

void LED1_On(void){
	// turn LED1 on
	P1->OUT |= BIT0;
}

void LED1_Off(void){
	// turn LED1 off
	P1->OUT &=~BIT0;
}

void LED2_Off(void){
	// turn LED2 off
	P2->OUT &=~BIT0;
  P2->OUT &=~BIT1;
  P2->OUT &=~BIT2;
}

void LED2_On(int color){
	LED2_Off();
	switch(color) {
   case 0:
      //LED1_Off();
      break;
   case 1:
			// LED on, Color Red
      P2->OUT |= BIT0;
      break;
   case 2:
			// LED on, Color Green
      P2->OUT |= BIT1;
      break;
   case 3:
			// LED on, Color Blue
      P2->OUT |= BIT2;
      break;
   case 4:
			// LED on, Color Cyan
      P2->OUT |= BIT1;
      P2->OUT |= BIT2;
      break;
   case 5:
			// LED on, Color Magenta
      P2->OUT |= BIT0;
      P2->OUT |= BIT2;
      break;
   case 6:
			// LED on, Color Yellow
      P2->OUT |= BIT0;
      P2->OUT |= BIT1;
      break;
	 case 7:
			// LED on, Color White
      P2->OUT |= BIT0;
      P2->OUT |= BIT1;
      P2->OUT |= BIT2;
      break;
   default:
     break;
  }
}

BOOLEAN LED1_State(void){
  BOOLEAN state = FALSE;
  if (P1->OUT & BIT0){
    state = TRUE;
  }
  return (state);
}
