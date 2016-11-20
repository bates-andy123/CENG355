/*
 * pwm_in.c
 *
 *  Created on: Nov 18, 2016
 *      Author: asus-andy
 */
#include "stm32f0xx.h"
#include "stm32f0xx_misc.h"

extern void timer2_IT_config(){
	  /* GPIOC clock enable */
	  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	  TIM2->ARR = 12000000;

	  TIM2->CR1 |= TIM_CR1_CEN; /* (5) */
}

extern void DAC_init(){
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	DAC->CR |= DAC_CR_TSEL1  | DAC_CR_EN1;
	DAC->DHR12R1 = 0x000;
}
