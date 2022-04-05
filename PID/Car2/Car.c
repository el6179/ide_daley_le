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
#include "Init.h"
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
/*
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
*/
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

double clip(double pwm){
	float min = 0.0, max = 40.0;
	if (pwm > max) {
		 pwm = max;
	} else if (pwm < min) {
		 pwm = min;
	}
	return pwm;
}

double PID(double Vdes, double e[], double Vact, double pwm) {
	double err = Vdes - Vact;
	double kp=0.4, ki=0.6, kd=0.1;
	pwm += kp*e[0] + ki*(e[0]-e[1])/2.0 + kd*(e[0]-2*e[1]+e[2]);
	e[2] = e[1];
	e[1] = e[0];		
	e[0] = err;
	pwm = clip(pwm);
	return pwm;
}

void forward(double speed){
	leftWheel(speed);
	rightWheel(speed);
}

void backward(double speed){
	leftWheel(-speed);
	rightWheel(-speed);
}

void stop(void){/*Stop Car*/
	forward(0.0);/*backward(0.0) should produce same result*/
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
	double e[3],vact = 0,pwm;
	pwm = PID(20,e,vact,pwm);
	forward(pwm);
	vact = pwm;
	LED1_Off();
	LED2_On(3);
	for(;;){
		sprintf(str,"Speed %f\n\r", vact);
		uart2_put(str);
		smoothData();
		if ((map[1] - map[0]) < 35){
			LED2_On(7);
			stop();
			//uart2_put("MOTHER HELP!!");
			break;
		}
		else if((map[0] >= 15) && (map[0] < 54)){
			LED2_On(1);
			if ((map[0] >= 15) && (map[0] < 25)){
				pwm = PID(24,e,vact,pwm);
				TIMER_A2_PWM_DutyCycle(0.035,1);
			}
			else if ((map[0] >= 25) && (map[0] < 54)){
				pwm = PID(22,e,vact,pwm);
				TIMER_A2_PWM_DutyCycle(0.025,1);
			}
			//sprintf(str,"Right : %i\n\r", map[0]);
			//uart2_put(str);
			//uart2_put("Turn Right");
		}
		else if((map[1] > 74) && (map[1] <= 115)){
			LED2_On(2);
			if ((map[1] > 74) && (map[1] <= 100) ){
				pwm = PID(22,e,vact,pwm);
				TIMER_A2_PWM_DutyCycle(0.075,1);
			}
			else if ((map[1] > 100) && (map[1] <= 115)){
				pwm = PID(24,e,vact,pwm);
				TIMER_A2_PWM_DutyCycle(0.065,1);
			}
			//sprintf(str,"Left : %i\n\r", map[1]);
			//uart2_put(str);
			//uart2_put("Turn Left");
		}
		else{
			LED2_On(3);
			pwm = PID(26,e,vact,pwm);
			TIMER_A2_PWM_DutyCycle(0.05,1);
			//uart2_put("Just Keep Swimming");
		}
		
		rightWheel(0.01*pwm);
		leftWheel(0.01*pwm);
		vact = pwm;
		
	}
	
}
