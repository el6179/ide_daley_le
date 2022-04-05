#ifndef _LEDSH_
#define _LEDSH_

typedef unsigned char BOOLEAN;
#define FALSE 0
#define TRUE  !FALSE
typedef unsigned char BYTE;

void LED1_Init(void);
void LED2_Init(void);
void LED1_On(void);
void LED1_Off(void);

void LED2_Off(void);
void LED2_On(int);

BOOLEAN LED1_State(void);

#endif
