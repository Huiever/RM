#ifndef _LED_H_
#define _LED_H_

#include "sys.h"

#define LED_ON  0
#define LED_OFF 1

#define LED1 PGout(1)
#define LED2 PGout(2)
#define LED3 PGout(3)
#define LED4 PGout(4)
#define LED5 PGout(5)
#define LED6 PGout(6)
#define LED7 PGout(7)
#define LED_Red   PEout(11)
#define LED_Green PFout(14)

void LED_Configuration(void);
void LED_Init(void);

#endif
