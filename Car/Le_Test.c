/*
TI Car
Author : Evon Le, Thomas Daley
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
#include "Init.h"

extern uint16_t line[128];
extern BOOLEAN g_sendData;

static char str[100];

float kp = 0.5, ki = 0.1, kd = 0.1;	

uint16_t smooth[128];
int map[3];

void leftWheel(float speed){
	if (speed<0){
		TIMER_A0_PWM_DutyCycle(-speed, 1);
		TIMER_A0_PWM_DutyCycle(0.0, 2);
	}
	else{
		TIMER_A0_PWM_DutyCycle(0.0, 1);
		TIMER_A0_PWM_DutyCycle(speed, 2);
	}
}
void rightWheel (float speed){
	if (speed<0){
		TIMER_A0_PWM_DutyCycle(-speed, 3);
		TIMER_A0_PWM_DutyCycle(0.0, 4);
	}
	else{
		TIMER_A0_PWM_DutyCycle(0.0, 3);
		TIMER_A0_PWM_DutyCycle(speed, 4);
	}
}

void forward(float speedL, float speedR){
	leftWheel(speedL);
	rightWheel(speedR);
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

float PID(float error, float lastErr){
	float correction = (kp * error) + (ki * (error + lastErr)/2) + (kd * (error - lastErr));;
	
	return correction;
}

int main(void) {
	DisableInterrupts();
	INIT(); //Initialize
	EnableInterrupts();
	LED1_On();
	uart0_put("Init Code\n\r");
	P3->OUT |= BIT6;
	P3->OUT |= BIT7;
	LED1_Off();
	LED2_On(1);
	myDelay(25);
	LED2_On(6);
	myDelay(25);
	LED2_On(2);
	//LED2_On(3);
	float lastErr = 0;
	 
	float rightMotorSpeed, leftMotorSpeed, center, speedErr, lastPosErr, shift, baseSpeed = 30;
	float posErr, left, right, leftErr, rightErr;
	
	forward(30, 30);
	
	for(;;){
		smoothData();
		center = ((map[1]+map[0])/2.0);
		/*Carpet Detection*/
		if ((map[1] - map[0]) < 30){
			LED2_Off();
			sprintf(str,"x1 = %i, x2 = %i\n\r", map[0], map[1]);
			uart2_put(str);
			backward(0.25);
			//stop();
			//uart2_put("MOTHER HELP!!");
			continue;
		}
		/*Steering*/
		shift = ((-(1.0/4.0))*(center-54.0)) + 7.5;
		// Max/Min steering
		if (shift > 7.5) {
			 shift = 7.5;
		} else if (shift < 2.5) {
       shift = 2.5;
    }
		sprintf(str,"shift = %.3f\n\r", shift);
	  uart2_put(str);
		// Assign servo position
		TIMER_A2_PWM_DutyCycle(0.01*shift, 1);
		
		
		posErr = (64 - center)*0.0375;
		leftMotorSpeed = baseSpeed - posErr;
		rightMotorSpeed = baseSpeed + posErr;
		
		left = PID(leftMotorSpeed, leftErr);
		right = PID(rightMotorSpeed, rightErr);
    forward(leftMotorSpeed*0.01, rightMotorSpeed*0.01);
		leftErr = left;
		rightErr = right;
		/*
		// Calculate Positional Error
		posErr = 64 - center;
		corErr = PID(posErr);
		speedErr = (kp * posErr) + (ki * (posErr + lastPosErr)/2) + (kd * (posErr - lastPosErr));
		lastPosErr = posErr;
		
		// Apply error calculations
		rightMotorSpeed = baseSpeed + speedErr;
		leftMotorSpeed = baseSpeed - speedErr;
		
		// Set motors to adjusted speeds
    forward(leftMotorSpeed*0.01, rightMotorSpeed*0.01);
		*/
	}
	
}
