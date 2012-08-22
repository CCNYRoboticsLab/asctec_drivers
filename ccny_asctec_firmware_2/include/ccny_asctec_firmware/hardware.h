#ifndef HARDWARE_H
#define HARDWARE_H

#include "LPC214x.h"
#include "main.h"
#include "system.h"
#include "uart.h"
#include "irq.h"

#define EXT_NCS 7   //CS outputs on P0
#define LL_nCS  20	
#define CTS_RADIO 22

//I/Os on P1
#define CAMERA_FET	16	

void LED(unsigned char, unsigned char);

void beeper(unsigned char offon);

#endif // HARDWARE_H


