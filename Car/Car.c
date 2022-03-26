/*
TI Car
Author : Evon Le
*/
#include <stdio.h>
#include <stdlib.h>

#include "ADC14.h"
#include "Common.h"
#include "ControlPins.h"
#include "CortexM.h"
#include "leds.h"
#include "msp.h"
#include "switches.h"
#include "SysTickTimer.h"
#include "TimerA.h"
#include "uart.h"

extern uint16_t line[128];
extern BOOLEAN g_sendData;

static char str[100];

void INIT_Camera(void)
{
	g_sendData = FALSE;
	ControlPin_SI_Init();
	ControlPin_CLK_Init();
	ADC0_InitSWTriggerCh6();
}

void INIT_Motors(void){
	// Init DC Motor timers
	uint16_t freq = 10000; // Frequency = 10 kHz 
	TIMER_A0_PWM_Init(SystemCoreClock/(freq*4), 0.0, 1);
  TIMER_A0_PWM_Init(SystemCoreClock/(freq*4), 0.0, 2);
	TIMER_A0_PWM_Init(SystemCoreClock/(freq*4), 0.0, 3);
  TIMER_A0_PWM_Init(SystemCoreClock/(freq*4), 0.0, 4);
	// Init Motor Enable
	P3->SEL0 &= ~BIT6;
	P3->SEL1 &= ~BIT6;
  P3->DIR |= BIT6;
	P3->OUT &= ~BIT6;
	
	P3->SEL0 &= ~BIT7;
	P3->SEL1 &= ~BIT7;
  P3->DIR |= BIT7;
	P3->OUT &= ~BIT7;
	
	// Center Servo Motor
	TIMER_A2_PWM_Init(SystemCoreClock/(50*4), 0.42, 1);
}

void INIT(){
	// INIT Camera
	INIT_Camera();
	// INIT Motors
	INIT_Motors();
	
	// Init ADC
	//ADC0_InitSWTriggerCh6();
	
	//UART, LED, SWITCHES INITS
	uart0_init();
	LED1_Init();
	LED2_Init();
	Switch1_Init();
	Switch2_Init();
}

void readCamera(void){
	if (g_sendData == TRUE){
		
	}
}

void myDelay(int del){
	/*
	volatile int j = 0;
	for (j = 0; j < 800000; j++){
		;
	}*/
	
	volatile int i;
	for (i=0; i<del*50000; i++){
		;// Do nothing
	}
}
int main(void) {
	DisableInterrupts();
	INIT(); //Initialize
	EnableInterrupts();
	uart0_put("Init Code\n\r");
	for(;;){
		/*
			TIMER_A2_PWM_DutyCycle(0.28, 1);
			myDelay(50);
			TIMER_A2_PWM_DutyCycle(0.42, 1);
			myDelay(50);
			TIMER_A2_PWM_DutyCycle(0.56, 1);
			myDelay(50);
		*/
	

		if (g_sendData == TRUE) 
		{
			LED1_On();
			// send the array over uart
			sprintf(str,"%i, ",-1); // start value
			uart0_put(str);
			for (int i = 0; i < 128; i++) 
			{
				sprintf(str,"%i, ", line[i]);
				uart0_put(str);
			}
			sprintf(str,"%i\n\r",-2); // end value
			uart0_put(str);
			LED1_Off();
			g_sendData = FALSE;
		}
		// do a small delay
		myDelay(75);
		
		/*
		for (int i=27;i <=50;i++){
			LED2_On(i%7);
			TIMER_A2_PWM_DutyCycle(0.01*i, 1);
			myDelay(75);
		}*/
		/*
		LED2_On(1);
		TIMER_A2_PWM_DutyCycle(0.45, 1);
		myDelay(100);
		LED2_On(2);
		TIMER_A2_PWM_DutyCycle(0.28, 1);
		myDelay(100);
		LED2_On(3);
		TIMER_A2_PWM_DutyCycle(0.61, 1);
		myDelay(100);
		*/
	}
}
