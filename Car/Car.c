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

uint16_t smooth[128];
int map[3];

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
	TIMER_A2_PWM_Init(SystemCoreClock/(50*4), 0.0575, 1);
}

void INIT(){
	// INIT Camera
	INIT_Camera();
	// INIT Motors
	INIT_Motors();
	
	//UART, LED, SWITCHES INITS
	uart0_init();
	LED1_Init();
	LED2_Init();
	Switch1_Init();
	Switch2_Init();
}

void forward(void){
	
	TIMER_A0_PWM_DutyCycle(0.0, 1);	//Forwards Left
	TIMER_A0_PWM_DutyCycle(0.2, 2);
	TIMER_A0_PWM_DutyCycle(0.0, 3);//Forwards Right
	TIMER_A0_PWM_DutyCycle(0.2, 4);
}

void backward(void){
	TIMER_A0_PWM_DutyCycle(0.2, 1);	//Backwards Left
	TIMER_A0_PWM_DutyCycle(0.0, 2);
	TIMER_A0_PWM_DutyCycle(0.2, 3);	//Backwards Right
	TIMER_A0_PWM_DutyCycle(0.0, 4);
}

void stop(void){/*Stop Car*/
	TIMER_A0_PWM_DutyCycle(0.0, 1);
	TIMER_A0_PWM_DutyCycle(0.0, 2);
	TIMER_A0_PWM_DutyCycle(0.0, 3);
	TIMER_A0_PWM_DutyCycle(0.0, 4);
	//Disable Motor
	P3->OUT &= ~BIT6;
	P3->OUT &= ~BIT7;
}

void smoothData(void){
	int max=0;
	if (g_sendData == TRUE){
		for (int i=0;i<128;i++){
			switch(i) {
				case 0:
					smooth[i] = (line[0]+line[1]+line[2])/3;
					break;
				case 1:
					smooth[i] = (line[0]+line[1]+line[2]+line[3])/4;
					break;
				case 126:
					smooth[i] = (line[124]+line[125]+line[126]+line[127])/4;
					break;
				case 127:
					smooth[i] = (line[125]+line[126]+line[127])/3;
					break;
				default:
					smooth[i] = (line[i-2]+line[i-1]+line[i]+line[i+1]+line[i+2])/5;
					break;
			}
			/*if(smooth[i] > max){
				max = smooth[i];
				map[0]=i;
			}
			else if(smooth[i] < max){
				map[1]=i-1;
			}*/
		}
	}
	map[2] = max;
}
//return arr[x1,x2,ymax]
void myDelay(int del){
	volatile int i;
	for (i=0; i<del*50000; i++){
		;// Do nothing
	}
}

int main(void) {
	DisableInterrupts();
	INIT(); //Initialize
	EnableInterrupts();
	LED1_On();
	uart0_put("Init Code\n\r");
	P3->OUT |= BIT6;
	P3->OUT |= BIT7;
	myDelay(50);
	//forward();
	LED1_Off();
	LED2_On(3);
	for(;;){
		smoothData();
		if (g_sendData == TRUE) 
		{
			LED1_On();/*
			// send the array over uart
			sprintf(str,"%i, ",-1); // start value
			uart0_put(str);
			for (int i = 0; i < 128; i++) 
			{
				sprintf(str,"%i, ", smooth[i]);
				uart0_put(str);
			}
			sprintf(str,"%i\n\r",-2); // end value
			uart0_put(str);
			LED1_Off();*/
			sprintf(str,"x1 = %i, x2 = %i, ymax = %i \n\r", map[0], map[1], map[2]);
				uart0_put(str);
			g_sendData = FALSE;
		}
		// do a small delay
		myDelay(25);
		/*
		smoothData();
		if (map[2] < 11000){stop();}
		
		double center = ((map[1]-map[0])/2.0)+map[0];
		double shift;
		if (center < 44){
			LED2_On(1);
			shift = ((center*2.25)/44.0)+3.5;
		}
		else if (center > 84){
			LED2_On(2);
			shift = (((center-84.0)*2.25)/44.0)+5.75;
		}
		TIMER_A2_PWM_DutyCycle(0.01*shift,1);*/
	}
	
}
