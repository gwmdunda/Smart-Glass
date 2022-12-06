#ifndef __HCSR04_H
#define __HCSR04_H


#include "stm32f10x.h"
#include "stm32f10x_gpio.h"	  
#include "stm32f10x_tim.h"	  
#include "stm32f10x_exti.h"  
#include "stm32f10x_exti.h"

extern volatile uint16_t distance_value;

void hcsr04_init(void);
void EXTI0_IRQHandler(void);

#endif /* __HCSR04_H */ 
