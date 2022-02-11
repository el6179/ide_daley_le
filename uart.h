#ifndef _UARTH_
#define _UARTH_

typedef unsigned char BOOLEAN;
#define FALSE 0
#define TRUE  !FALSE
typedef unsigned char BYTE;

#define CLSR              (uint16_t)(0xC0)  //CLOCK SOURCE
#define CMFS              (uint16_t)(0xFFF1)  //CLEAR FIRST AND SECOND MODULE
#define DSIN              (uint16_t)(0x000F)  //DISABLE INTERRUPTS

void uart0_init(void);
BYTE uart0_getchar(void);
void uart0_putchar(char ch);
void uart0_put(char *ptr_str);

void uart2_init(void);
BYTE uart2_getchar(void);
void uart2_putchar(char ch);
void uart2_put(char *ptr_str);

BOOLEAN uart0_dataAvailable(void);
BOOLEAN uart2_dataAvailable(void);

#endif
