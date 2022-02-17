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
#include <string.h>
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

unsigned long MillisecondCounter = 0;

char DecToHex(int x){
  char h;
  if (x > 9){
    switch(x) {
     case 10:
        h = 'A';
        break;
     case 11:
        h = 'B';
        break;
     case 12:
        h = 'C';
        break;
     case 13:
        h = 'D';
        break;
     case 14:
        h = 'E';
        break;
     case 15:
        h = 'F';
        break;
     default:
       break;
    }
  }
  else
    h = x + '0';
  
  return (h);
}

// Interrupt Service Routine for Timer32-1
void Timer32_1_ISR(void){

}
// Interrupt Service Routine
void Timer32_2_ISR(void){

}

void ADC_MEM0_Print(void){
  //wait for MEM0 to haveconversion result
  while(!(ADC14IFGR0 & BIT0)){}
  int dVal = 0;
  int dc = 0;
  int dMask = 0x1;
  int n;
  char hPrint[4];
  int hc = 4;
  if (ADC14IFGR0 & BIT0){
    for (int x = 0; x<16; x++){
      n = (dMask<<x);
      if (ADC14MEM0 & n){
        dc += pow(2,x);
      }
      if (dc%4 == 3){
        dVal += dc;
        hPrint[hc] = DecToHex(dc);
      }
    }
  }
	int dSize = log10(dVal) + 1;
  char dPrint[dSize];
	for (int i = dSize-1; i >=0; --i) {
		dPrint[i]= (dVal%10) + '0';
		dVal /= 10;
	}
	
  uart0_put("\r\nHex: ");
  uart0_put(hPrint);
  uart0_put("\r\nDec: ");
  uart0_put(dPrint);
}

// main
int main(void){
	//char temp[64];
	//unsigned int analogIn = 0;
	//initializations
	uart0_init();
	uart0_put("\r\nLab5 ADC demo\r\n");

	

	LED1_Init();
	LED2_Init();
	Switch2_Init();
	ADC0_InitSWTriggerCh6();
	EnableInterrupts();
  while(1){
		ADC_MEM0_Print();
  }
}

