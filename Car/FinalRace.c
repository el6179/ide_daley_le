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
#define CHAR_COUNT 10
#define CENTER_SHIFT 7.2
// lower = 4.7, upper = 9.7
uint16_t smooth[128];
int map[3];
int setMode = 1;

float el[3], er[3], VLact = 0, VRact = 0, pwml, pwmr, lastPosErr;
//float centerErr[5];
//float kp=0.4, ki=0.6, kd=0.1;
//kp = 0.29 ki = 0.08 kd = 0.485
float kp = 0.29, ki = 0.05, kd = 0.22;
float baseSpeed;

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
void rightWheel(float speed){
	if (speed<0){
		TIMER_A0_PWM_DutyCycle(-speed, 3);
		TIMER_A0_PWM_DutyCycle(0.0, 4);
	}
	else{
		TIMER_A0_PWM_DutyCycle(0.0, 3);
		TIMER_A0_PWM_DutyCycle(speed, 4);
	}
}


void PID(float Vdes, float e[], float *Vact, float *pwm) {
	float err = Vdes - *Vact;
	*pwm += (kp*(e[0]-e[1])) + (ki*(e[0]+e[1])/2.0) + (kd*(e[0]-2*e[1]+e[2]));
	e[2] = e[1];
	e[1] = e[0];
	e[0] = err;
	
	const float max = 55, min = 0;
	if (*pwm > max){
		*pwm = max;
	} 
	else if (*pwm < min){
		*pwm = min;
	}
	
	*Vact = *pwm;
}

void forward(float speedL, float speedR, float shift, float posErr){
  
	float x = fabs(posErr); // fPosErr
	float multiplier;
	// Best run: 0.0065 - 0.038 + 0.1455 (0.75 max)
	if (x > 5) {
	  //multiplier = (0.01275*x*x) + (-0.123*x) + 0.34425; // max 0.9
		//multiplier = (0.0025*x*x) + (0.02*x) + 0.0675;
    multiplier = (0.00416667*x*x) + (0.00833333*x) + 0.1875;
	} 
	else if (x >= 13) {
		multiplier = 1;
	}
	else {
	  multiplier = 0;
	}

	float speedErr = posErr * multiplier;
	
	PID(speedL - speedErr, el, &VLact, &pwml);
	PID(speedR + speedErr, er, &VRact, &pwmr);
  //float leftPID = pwml - speedErr;
	//float rightPID = pwmr + speedErr;
	
	int max = 55, min = 0;
	
	/*if (leftPID > max) {
		LED2_On(3);
		leftPID = max;
	} 
	else if (leftPID < min) {
		leftPID = min;
	} 
	else if (rightPID > max) {
		LED2_On(3);
    rightPID = max;
	}		 
	else if (rightPID < min) {
		rightPID = min;
	}*/
	
	leftWheel(0.01*pwml);
	rightWheel(0.01*pwmr);
	
	//sprintf(str,"Left Wheel: %.3f  Right Wheel: %.3f \n\r", leftPID, rightPID);
	//uart2_put(str);
	
}

void backward(double speed){
	TIMER_A2_PWM_DutyCycle(0.01*CENTER_SHIFT, 1);
	leftWheel(-speed);
	rightWheel(-speed);

}

void stop(void){/*Stop Car*/
	forward(0.0, 0.0, 5.0, 64);/*backward(0.0) should produce same result*/
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
void myDelay(int del){
	volatile int i;
	for (i=0; i<del*50000; i++){
		;// Do nothing
	}
}

void PORT1_IRQHandler(void){
    if(P1->IFG & BIT1){
			  setMode = 0;
        P1->IFG &= ~BIT1;
			  baseSpeed = 30;
				LED2_On(5);
    }
		else if(P1->IFG & BIT4){
			  setMode = 0;
        P1->IFG &= ~BIT4;
			  baseSpeed = 33;
				LED2_On(4);
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
	
	while(setMode){
		WaitForInterrupt();
	}
	
	LED1_Off();
	LED2_On(1);
	myDelay(50);
	LED2_On(6);
	myDelay(50);
	LED2_On(2);
	 
	float center, shift, speed;
	// baseSpeed = 27, kp = 0.375, ki = 0.35, kd = 0.45
	// baseSpeed = 28, kp = 0.375, ki = 0.19, kd = 0.5
	// baseSpeed = 30, kp = 0.375, ki = 0.24, kd = 0.5
  //float kp = 0.375, ki = 0.24, kd = 0.5;	
	
	//sprintf(str,"p = %.2f, i = %.2f, d = %.2f\n\r", kp,ki,kd);
	//uart2_put(str);
	//forward(baseSpeed, baseSpeed, 5.0, 64);
	
	
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

		smoothData();
		
		center = ((map[1]+map[0])/2.0);
		
		if ((map[1] - map[0]) < 5){
			LED2_Off();
			sprintf(str,"x1 = %i, x2 = %i, x2-x1 = %i\n\r", map[0], map[1], map[1] - map[0]);
			uart2_put(str);
			backward(0.25);
			//stop();
			continue;
			//break;
		}
		if (center >= 60 && center <= 68){
			shift = ((-(1.0/5.0))*(center-51.5)) + (CENTER_SHIFT+2.5);
		}
		else {
			shift = ((-(1.0/4.0))*(center-54.0)) + (CENTER_SHIFT+2.5);
		}
		// Max/Min steering
		if (shift > CENTER_SHIFT+2.5) {
			 //LED2_On(2);
			 shift = CENTER_SHIFT+2.5;
		} else if (shift < CENTER_SHIFT-2.5) {
			 //LED2_On(3);
       shift = CENTER_SHIFT-2.5;
    }
		
		// Changing motor speed
		if (shift < CENTER_SHIFT+0.35 && shift > CENTER_SHIFT-0.35) {
			speed = baseSpeed;
      ki = 0.05;
		} 
		else {
			LED2_Off();
			speed = 20;
      ki = 0.0;
		}
		
		
		//baseSpeed = (-(1.0/4.0))*((center-54.0)*(center-54.0)) + 40;
		
		//sprintf(str,"SPEED = %.3f\n\r", speed);
	  //uart2_put(str);
		
		// Assign servo position
		if (shift > CENTER_SHIFT-0.25 && shift < CENTER_SHIFT+0.25){
		    shift = CENTER_SHIFT;  
		} 
	
		TIMER_A2_PWM_DutyCycle(0.01*shift, 1);
		
		// Calculate Positional Error
		
		float posErr = (64 - center);
		
		//speedErr = (kp * posErr) + (ki * (posErr + lastPosErr)/2) + (kd * (posErr - lastPosErr));
		//lastPosErr = posErr;
		
		// Apply error calculations
		// rightMotorSpeed = baseSpeed + speedErr;
		// leftMotorSpeed = baseSpeed - speedErr;
		
		// Set motors to adjusted speeds
		forward(speed, speed, shift, posErr);
		//sprintf(str, "RM Speed = %.3f\n\rLM Speed = %.3F\n\r", rightMotorSpeed, leftMotorSpeed);
		//uart2_put(str);
		
	}
	
}
