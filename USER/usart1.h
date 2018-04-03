#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
#include <stdio.h>

#define MY_PRINTF


void USART1_Config(void);
void NVIC_Configuration(void);
int fputc(int ch, FILE *f);
void myPrintf(const char *fmt, ...);
#endif /* __USART1_H */
