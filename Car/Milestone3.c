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
static char prt[10];
static char k;
#define CHAR_COUNT 10

uint16_t smooth[128];
int map[3];

float el[3], er[3], VLact = 0, VRact = 0, pwml, pwmr;

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

float PID(float Vdes, float e[], float *Vact, float *pwm) {
	float err = Vdes - *Vact;
	float kp=0.4, ki=0.6, kd=0.1;
	*pwm += kp*(e[0]-e[1]) + ki*(e[0]+e[1])/2.0 + kd*(e[0]-2*e[1]+e[2]);
	e[2] = e[1];
	e[1] = e[0];
	e[0] = err;
	*Vact = *pwm;
	return *pwm;
}

void forward(float speedL, float speedR){
	
	float leftPID = PID(speedL, el, &VLact, &pwml);
	float rightPID = PID(speedR, er, &VRact, &pwmr);
	
	//sprintf(str,"Left Wheel: %.3f  Right Wheel: %.3f \n\r", leftPID, rightPID);
	//uart2_put(str);
	
	const float max = 50, min = 0;
	
	if (leftPID > max) {
		 leftPID = max;
	} else if (leftPID < min) {
		 leftPID = min;
	}
	
	if (rightPID > max) {
		 rightPID = max;
	} else if (rightPID < min) {
		 rightPID = min;
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

void PORT1_IRQHandler(void){
    if(P1->IFG & BIT1){
        P1->IFG &= ~BIT1;
				P3->OUT |= BIT6;
				P3->OUT |= BIT7;
				myDelay(50);
				forward(20,20);
				LED2_On(3);
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
	LED1_Off();
	LED2_On(1);
	myDelay(50);
	LED2_On(6);
	myDelay(50);
	LED2_On(2);
	//LED2_On(3);
	
	 
	float rightMotorSpeed, leftMotorSpeed, center, speedErr, posErr, lastPosErr, shift, baseSpeed = 30;
	// baseSpeed = 27, kp = 0.375, ki = 0.35, kd = 0.45
	// baseSpeed = 28, kp = 0.375, ki = 0.19, kd = 0.5
	// experimental bs = 30, kp = 0.375, ki = 0, kd = 0.5
  float kp = 0.375, ki = 0.24, kd = 0.5;	
	
	//sprintf(str,"p = %.2f, i = %.2f, d = %.2f\n\r", kp,ki,kd);
	//uart2_put(str);
	forward(30, 30);
	
  int pc = 0;
	for(;;){
		/*
		if (Switch1_Pressed() == TRUE){
			LED2_On(6);
			TIMER_A2_PWM_DutyCycle(0.05, 1);
			myDelay(50);
			forward(30, 30);
			P3->OUT |= BIT6;
			P3->OUT |= BIT7;
		}
		*/
		if (uart2_dataAvailable()==TRUE){
			// Retrieve the character
			char pch = uart2_getchar();// get the character
      if (pch == 0x20){
				char* num = prt + 1;
				switch(k) {
					case 'P':
						kp=atof(num);
					break;
					case 'I':
						ki=atof(num);
					break;
					case 'D':
						kd=atof(num);
					break;
				}
				pc = 0;
				sprintf(str,"p = %.2f, i = %.2f, d = %.2f\n\r", kp,ki,kd);
				uart2_put(str);
      }
      else{
        //Add char to phone string like normal
        if (pc < CHAR_COUNT){
					if (pc == 0){
						k = pch;
					}
					else{
						prt[pc] = pch;
					}
          pc++;
        }
      }
		}
		smoothData();
		
		center = ((map[1]+map[0])/2.0);
		
		if ((map[1] - map[0]) < 30){
			LED2_Off();
			sprintf(str,"x1 = %i, x2 = %i, x2-x1 = %i\n\r", map[0], map[1], map[1] - map[0]);
			uart2_put(str);
			stop();
			//uart2_put("MOTHER HELP!!");
			break;
		}
		
		shift = ((-(1.0/4.0))*(center-54.0)) + 7.5;
		
		// Max/Min steering
		if (shift > 7.5) {
			 shift = 7.5;
		} else if (shift < 2.5) {
       shift = 2.5;
    }
		
		// Changing motor speed
		/*if (shift < 5.2 && shift > 4.8) {
			baseSpeed = 30;
		} 
		else {
			baseSpeed = 27;
		}*/
		
		//sprintf(str,"shift = %.3f\n\r", shift);
	  //uart2_put(str);
		
		// Assign servo position
		TIMER_A2_PWM_DutyCycle(0.01*shift, 1);
		
		// Calculate Positional Error
		posErr = 64 - center;
		speedErr = (kp * posErr) + (ki * (posErr + lastPosErr)/2) + (kd * (posErr - lastPosErr));
		lastPosErr = posErr;
		
		// Apply error calculations
		rightMotorSpeed = baseSpeed + speedErr;
		leftMotorSpeed = baseSpeed - speedErr;
		
		// Set motors to adjusted speeds
    forward(leftMotorSpeed, rightMotorSpeed);
		
		//sprintf(str, "RM Speed = %.3f\n\rLM Speed = %.3F\n\r", rightMotorSpeed, leftMotorSpeed);
		//uart2_put(str);
		
	}
	
}
