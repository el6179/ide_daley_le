/*
* Rochester Institute of Technology
* Department of Computer Engineering
* CMPE 460  Interfacing Digital Electronics
* LJBeato
* 1/14/2021
*
* Filename: main_timer_template.c
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "msp.h"
#include "uart.h"
#include "leds.h"
#include "switches.h"
#include "Timer32.h"
#include "CortexM.h"
#include "Common.h"
#include "ADC14.h"
// The sprintf function seemed to cause a hange in the interrupt service routine.
// I think if we increase the HEAP size, it will work
// change to Heap_Size       EQU     0x00000200 in startup_msp432p401r_uvision.s


BOOLEAN Timer1RunningFlag = FALSE;
BOOLEAN Timer2RunningFlag = FALSE;
extern uint32_t SystemCoreClock;
unsigned long MillisecondCounter = 0;


void PORT1_IRQHandler(void){
    if(P1->IFG & BIT1){
        P1->IFG &= ~BIT1;
        Timer1RunningFlag = !Timer1RunningFlag;
    }
		if(P1->IFG & BIT4){
        P1->IFG &= ~BIT4;
        Timer2RunningFlag = !Timer2RunningFlag;
    }
}
// Interrupt Service Routine for Timer32-1
void Timer32_1_ISR(void){
    char temp[64];
    unsigned int analogIn = ADC_In();
    if (Timer1RunningFlag){
        sprintf(temp,"\r\n\nHex: %x \r\nDec: %u", analogIn, analogIn);
        uart0_put(temp);
    }
}

// Interrupt Service Routine
void Timer32_2_ISR(void){
    char temp[64];
    unsigned int analogIn = ADC_In();
		double volt, mV, dC, dF;
		if (Timer2RunningFlag){
				volt = (analogIn / 16384.0);
				mV = volt * 3300.0;
				dC = (mV - 500.0)/20.0;
				dF = (dC*(9.0/5.0))+32.0;
			sprintf(temp,"\r\nAnalog output: %u; Temp: %.3f C, %.3f F", analogIn, dC, dF);
			//sprintf(temp,"\r\nAnalog output: %u; volt : %f, mV: %f; Temp: %f C", analogIn, volt, mV, dC);
        uart0_put(temp);
    }
}

void Switch1_Interrupt_Init(void){
	// disable interrupts
	DisableInterrupts();
	// initialize the Switch as per previous lab
	Switch1_Init();
	
	//7-0 PxIFG RW 0h Port X interrupt flag
	//0b = No interrupt is pending.
	//1b = Interrupt is pending.
	// clear flag1 (reduce possibility of extra interrupt)	
    P1->IFG &= ~BIT1; 

	//7-0 PxIE RW 0h Port X interrupt enable
	//0b = Corresponding port interrupt disabled
	//1b = Corresponding port interrupt enabled	
	// arm interrupt on  P1.1	
    P1->IE |= BIT1;  

	//7-0 PxIES RW Undefined Port X interrupt edge select
  //0b = PxIFG flag is set with a low-to-high transition.
  //1b = PxIFG flag is set with a high-to-low transition
	// now set the pin to cause falling edge interrupt event
	// P1.1 is falling edge event
    P1->IES |= BIT1; 
	
	// now set the pin to cause falling edge interrupt event
  NVIC_IPR8 = (NVIC_IPR8 & 0x00FFFFFF)|0x40000000; // priority 2
	
	// enable Port 1 - interrupt 35 in NVIC	
  NVIC_ISER1 = 0x00000008;  
	
	// enable interrupts  (// clear the I bit	)
  EnableInterrupts();              
	
}

void Switch2_Interrupt_Init(void)
{
	// disable interrupts
	DisableInterrupts();
	
	// initialize the Switch as per previous lab
	Switch2_Init();
	
	  P1->IFG &= ~BIT4;
	// now set the pin to cause falling edge interrupt event
	// P1.4 is falling edge event
    P1->IES |= BIT4;
  
	// clear flag4 (reduce possibility of extra interrupt)
    //P1->IFG &= ~BIT4; 
  
	// arm interrupt on P1.4 
    P1->IE |= BIT4;     
	
	// now set the pin to cause falling edge interrupt event
  NVIC_IPR8 = (NVIC_IPR8&0x00FFFFFF)|0x40000000; // priority 2
	
	// enable Port 1 - interrupt 35 in NVIC
  NVIC_ISER1 = 0x00000008;         
	
	// enable interrupts  (// clear the I bit	)
  EnableInterrupts();              
	
}

// main
int main(void){
	//initializations
	uart0_init();
	uart0_put("\r\nLab5 ADC demo\r\n");
    // Set the Timer32-1 to 1Hz (0.5 sec between interrupts)
	Timer32_1_Init(&Timer32_1_ISR, SystemCoreClock/2, T32DIV1); // initialize Timer A32-1;
	Timer32_2_Init(&Timer32_2_ISR, SystemCoreClock/2, T32DIV1); // initialize Timer A32-2;

	LED1_Init();
	LED2_Init();
  Switch1_Interrupt_Init();
  Switch2_Interrupt_Init();
	ADC0_InitSWTriggerCh6();
	EnableInterrupts();
    while(1){
        WaitForInterrupt();
    }
}

