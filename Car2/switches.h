#ifndef _SWITCHESH_
#define _SWITCHESH_

typedef unsigned char BOOLEAN;
#define FALSE 0
#define TRUE  !FALSE
typedef unsigned char BYTE;

void Switch1_Init(void);
void Switch2_Init(void);

BOOLEAN Switch1_Pressed(void);
BOOLEAN Switch2_Pressed(void);

#endif
