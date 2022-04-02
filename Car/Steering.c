/*
TI Car
Author : Evon Le, Thomas Engine Daley
*/
#include <math.h>
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

float el[3], er[3], VLact = 0, VRact = 0, pwml, pwmr, err;

void INIT_Camera(void){
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
	TIMER_A2_PWM_Init(SystemCoreClock/(50*4), 0.05, 1);
}

void INIT(){
	// INIT Camera
	INIT_Camera();
	// INIT Motors
	INIT_Motors();
	
	//UART, LED, SWITCHES INITS
	uart0_init();
	uart2_init();
	LED1_Init();
	LED2_Init();
	Switch1_Init();
	Switch2_Init();
}

void leftWheel(double speed){
	if (speed<0){
		TIMER_A0_PWM_DutyCycle(-speed, 1);
		TIMER_A0_PWM_DutyCycle(0.0, 2);
	}
	else{
		TIMER_A0_PWM_DutyCycle(0.0, 1);
		TIMER_A0_PWM_DutyCycle(speed, 2);
	}
}
void rightWheel (double speed){
	if (speed<0){
		TIMER_A0_PWM_DutyCycle(-speed, 3);
		TIMER_A0_PWM_DutyCycle(0.0, 4);
	}
	else{
		TIMER_A0_PWM_DutyCycle(0.0, 3);
		TIMER_A0_PWM_DutyCycle(speed, 4);
	}
}

float PID(float Vdes, float e[], float *Vact, float *pwm) {
	err = Vdes - *Vact;
	float kp=0.4, ki=0.6, kd=0.1;
	*pwm += kp*e[0] + ki*(e[0]-e[1])/2.0 + kd*(e[0]-2*e[1]+e[2]);
	e[2] = e[1];
	e[1] = e[0];
	e[0] = err;
	*Vact = *pwm;
	return *pwm;
}

void forward(float sl, float sr){
	float leftPID = PID(sl, el, &VLact, &pwml);
	float rightPID = PID(sr, er, &VRact, &pwmr);
	//sprintf(str,"Left Wheel: %.3f  Right Wheel: %.3f \n\r", leftPID, rightPID);
	//uart2_put(str);
	if (leftPID > 30) {
		 leftPID = 30;
	} else if (leftPID < 0) {
		 leftPID = 0;
	}
	if (rightPID > 30) {
		 rightPID = 30;
	} else if (rightPID < 0) {
		 rightPID = 0;
	}
	leftWheel(0.01*leftPID);
	rightWheel(0.01*rightPID);
	
}

void backward(double speed){
	
	leftWheel(-speed);
	rightWheel(-speed);

}

void stop(void){/*Stop Car*/
	forward(0.0, 0.0);/*backward(0.0) should produce same result*/
	//Disable Motor
	P3->OUT &= ~BIT6;
	P3->OUT &= ~BIT7;
}

void smoothData(void){
	int max=0;
	int j=1;
	map[1] = 0;
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
			if(smooth[i] > max){
				max = smooth[i];
				map[0] = i;
			}
		}
		while(map[1] == 0){
			if (smooth[map[0]+j] < max){
				map[1] = map[0]+j;
			}
			j++;
		}
		map[2] = max;
	}
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
	myDelay(100);
	forward(20, 20);
	LED1_Off();
	LED2_On(3);
	for(;;){
		smoothData();
		
		double center = ((map[1]+map[0])/2.0);
		//sprintf(str,"x1 %i, x2 %i, x2-x1 %f \n\r", map[0], map[1], center);
		//uart2_put(str);
		/*
		if ((map[1] - map[0]) < 35){
			LED2_On(7);
			sprintf(str,"x1 = %i, x2 = %i, x2-x1 = %i\n\r", map[0], map[1], map[1] - map[0]);
			uart2_put(str);
			stop();
			//uart2_put("MOTHER HELP!!");
			break;
		}
		else if((map[0] >= 15) && (map[0] < 54)){
			LED2_On(1);
			forward(25, 20); // Left 25 - Right 20
			//rightWheel(0.01*PID(25));
			//leftWheel(0.01*PID(30));
			if ((map[0] >= 15) && (map[0] < 25)){
				TIMER_A2_PWM_DutyCycle(0.035,1);
			}
			else if ((map[0] >= 25) && (map[0] < 54)){
				TIMER_A2_PWM_DutyCycle(0.025,1);
			}
			sprintf(str,"Right : %i\n\r", map[0]);
			uart2_put(str);
			//uart2_put("Turn Right");
		}
		else if((map[1] > 74) && (map[1] <= 115)){
			LED2_On(2);
			forward(20, 25); // Left 20 - Right 25
			//leftWheel(0.01*PID(25));
			//rightWheel(0.01*PID(30));
			if ((map[1] > 74) && (map[1] <= 100) ){
				TIMER_A2_PWM_DutyCycle(0.075,1);
			}
			else if ((map[1] > 100) && (map[1] <= 115)){
				TIMER_A2_PWM_DutyCycle(0.065,1);
			}
			sprintf(str,"Left : %i\n\r", map[1]);
			uart2_put(str);
			//uart2_put("Turn Left");
		}
		else{
			LED2_On(3);
			forward(30, 30);
			TIMER_A2_PWM_DutyCycle(0.05,1);
			//uart2_put("Just Keep Swimming");
		}*/
	  double shift = (-(0.25)*(center-50.0)) + 7.5;
		if (shift > 7.5) {
			 shift = 7.5;
		} else if (shift < 2.5) {
       shift = 2.5;
    }			
		
		if ((map[1] - map[0]) < 35){
			LED2_On(7);
			sprintf(str,"x1 = %i, x2 = %i, x2-x1 = %i\n\r", map[0], map[1], map[1] - map[0]);
			uart2_put(str);
			stop();
			//uart2_put("MOTHER HELP!!");
			break;
		}
		sprintf(str,"shift = %.3f\n\r", shift);
	  uart2_put(str);
		if (shift > 6.5){ // Try 6.25
			forward(20, 25); // Turn Left
			TIMER_A2_PWM_DutyCycle(0.01*shift, 1);
		} else if (shift <= 3.5){ // Try 3.75
			forward(25, 20); // Turn Right
			TIMER_A2_PWM_DutyCycle(0.01*shift, 1);
		} else {
			forward(30, 30); // Onward!
			TIMER_A2_PWM_DutyCycle(0.05, 1);
		}
		
	}
	
}
