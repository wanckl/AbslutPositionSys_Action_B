#ifndef __LED_H
#define __LED_H
#include "sys.h"

#define LED0 PCout(6)	// DS0
#define LED1 PCout(7)	// DS1

#define R0 PCout(8)	// DS0
#define R1 PCout(9)	// DS1

void LED_Init(void);	//LED≥ı ºªØ
void Heat_Res_Init(void);
#endif
