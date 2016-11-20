//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
//#include "diag/Trace.h"
//#include "cmsis/cmsis_device.h"
#include "stm32f0xx_spi.h"
#include "ADC_example.h"
#include "pwm_in.h"

static int freq_cycles = 0;
static int trigger_cnt = 0;

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


void configurePA();
void configurePB();
void configurePD();
void configureSPI1();
void configureLCD();


void config_exti_A0_A1();
void EXTI0_1_IRQHandler();

void writeLCD();
void changeAddress(int data);
void writeData(uint8_t);

int r;


int main(int argc, char* argv[]) {

	configurePA();
	configurePB();
	configurePD();
	configureSPI1();
	configureLCD();

	//enable ADC
	SetClockForADC();
	SetClockForADC();
	CalibrateADC();
	EnableADC();
	ConfigureADC();
	ConfigureTIM15();
	ADC1->CR |= ADC_CR_ADSTART;

	DAC_init();
	timer2_IT_config();

	config_exti_A0_A1();

	int i = 0;
	while(i < 10000)
		i++;

	writeLCD();

    // Infinite loop
	while (1) {

		unsigned int adc = ADC1->DR;

		/* Calculate resistance */
		r = (int)(adc * (5000.0/4095.0));
		DAC->DHR12R1 = adc;
		writeLCD();
	}
}

/*
 * Configure the PA registers.
 */

void send_cmd(uint8_t data){
	//high bits
	writeData((data & 0xF0)>>4);
	writeData(0x80 | ((data & 0xF0)>>4));
	writeData((data & 0xF0)>>4);

	//low bits
	writeData(0x0F & data);
	writeData(0x80 | (0x0F & data));
	writeData(0x0F & data);
}

extern void send_char(uint8_t data){
	//high bits
	writeData(0x40 | ((data & 0xF0)>>4));
	writeData(0xC0 | ((data & 0xF0)>>4));
	writeData(0x40 | ((data & 0xF0)>>4));

	//low bits
	writeData(0x40 | (0x0F & data));
	writeData(0xC0 | (0x0F & data));
	writeData(0x40 | (0x0F & data));
}

void configurePA() {
	/* Enable clock for GPIOA peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/****** PA0 *********/

	/* Configure PA0 as analog */
	GPIOA->MODER |= GPIO_MODER_MODER0;
	/* Ensure no pull-up/pull-down for PA0 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);

	/****** PA1 **********/

	/* Configure PA1 as input */
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);
	/* Ensure no pull-up/pull-down for PA1 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);


	/****** PA4 *********/

	/* Configure PA4 as analog */
	GPIOA->MODER |= GPIO_MODER_MODER4;
	/* Ensure no pull-up/pull-down for PA4 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

}

/*
 * Configure the PB registers.
 */
void configurePB() {
	/* Enable clock for GPIOB peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	/* Configure PB3 as alternate function */
	GPIOB->MODER |= GPIO_MODER_MODER3_1;

	/* No pull-up/down for PB3 */
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);

	/* Configure PB5 as alternate function */
	GPIOB->MODER |= GPIO_MODER_MODER5_1;

	/* No pull-up/down for PB5 */
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);


	GPIO_InitTypeDef GPIO_InitStructure;

	//Select port 3 to be our toggle port
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* Enable SPI1 on APB Bus */
}

/*
 * Configure the PD registers.
 */
void configurePD() {
	/* Enable clock for GPIOD peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIODEN;

	/* Configure PD2 as output */
	GPIOD->MODER |= GPIO_MODER_MODER2_0;

	/* No pull-up/down for PD2 */
	GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
}

/*
 * Configure the SPI registers.
 */
void configureSPI1() {
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	/* Use CMSIS function calls here to init */
	SPI_InitTypeDef SPI_InitStructInfo;
	SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256 ;
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, SPI_InitStruct);

	SPI_Cmd(SPI1, ENABLE);
}

/*
 * Set up LCD for desired display settings.
 */
void configureLCD() {
	/* Reconfigure and setup in 4 bit mode */
	send_cmd(0x33); //special function set command
	send_cmd(0x32); //special function set command and 4 bit mode

	// LCD options

	/* DDRAM using 4-bit interface; two lines of eight characters */
	send_cmd(0x28); //DL= 0 N=1 F=0
	/* Display on, cursor off and not blinking */
	send_cmd(0x0C); //D = 1, C=0 B =0
	/* Auto-increment DDRAM address; don't shift display */
	send_cmd(0x06); // I/D=1 S=0

	send_cmd(0x1); //clear display
}

extern void config_exti_A0_A1(){//eleven
	  // Enable the peripheral clock of GPIOA
	  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	  // Select mode
	  // Select input mode (00) on PA0
	  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER1 | GPIO_MODER_MODER0));
	  //Select Pull down for A0 and A1
	  GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR1_1);
	  // Configure Syscfg, exti and nvic for pushbutton PA0
	  // (1) PA0 as source input
	  // (2) unmask port 0
	  // (3) Rising edge
	  // (4) Set priority
	  // (5) Enable EXTI0_1_IRQn
	  SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~(SYSCFG_EXTICR1_EXTI1 | SYSCFG_EXTICR1_EXTI0)) | SYSCFG_EXTICR1_EXTI1_PA | SYSCFG_EXTICR1_EXTI0_PA; // (1)
	  EXTI->IMR |= (EXTI_IMR_MR1 | EXTI_IMR_MR0); // (2)
	  EXTI->RTSR |= (EXTI_RTSR_TR1 | EXTI_RTSR_TR0); // (3)
	  NVIC_SetPriority(EXTI0_1_IRQn, 0); // (4)
	  NVIC_EnableIRQ(EXTI0_1_IRQn); // (5)
}

/* 
 * This handler is declared in system/src/cmsis/vectors_stm32f0xx.c 
 *
 * Calculates frequency when rising edge event occurs. Writes to LCD.
 */

void EXTI0_1_IRQHandler(void)
{
	static int if_done_once = 0;

		/* Check if EXTI1 interrupt pending flag is indeed set */
		if ((EXTI->PR & EXTI_PR_PR1) != 0)
		{

			if(if_done_once == 0){
				TIM2->CNT = 0;
				TIM2->CR1 |= TIM_CR1_CEN;
				if_done_once = 1;
			}else{
				if_done_once = 0;
				freq_cycles = TIM2->CNT;

				TIM2->CR1 &= ~(TIM_CR1_CEN);
				TIM2->CNT = 0;
				TIM2->CR1 |= TIM_CR1_CEN;
			}
			EXTI->PR |= 0b10;
		}
}

/*
 * Displays current frequency and resistance on LCD.
 */
void writeLCD() {

	if(freq_cycles < 1000 )
		freq_cycles = 1000;
	int frequency = ((double)48000000.0)/((double)freq_cycles);

	//write freq
	changeAddress(0x0);
	send_char('F');
	send_char(':');
	send_char((frequency/1000) + 48);
	send_char('.');
	send_char((frequency/100)%10 + 48);
	send_char((frequency%10) + 48);
	send_char('k');
	send_char('H');

	//write Resistance
	changeAddress(0x40);
	send_char('R');
	send_char(':');
	send_char((r/1000) + 48);
	send_char((r%1000)/100 + 48);
	send_char((r%100)/10 + 48);
	send_char((r%10) + 48);
	send_char('O');
	send_char('h');
}

/*
 * Sends data in the format of an address to LCD.
 */
void changeAddress(int data) {
	//high bits
	writeData(0x8 | (data & 0xF0)>>4);
	writeData((0x80 | 0x8) | ((data & 0xF0)>>4));
	writeData(0x8 | (data & 0xF0)>>4);

	//low bits
	writeData(0x0F & data);
	writeData(0x80 | (0x0F & data));
	writeData(0x0F & data);
}

/*
 * Writes data to LCD when SPI is ready.
 */
void writeData(uint8_t data) {

	/* reset LCK signal to 0 by forcing PD2 to 0 */
	GPIOD->BSRR = GPIO_BSRR_BR_2;

	/* Wait until SPI1 is ready */
	while (SPI1->SR & SPI_SR_BSY);

	SPI_SendData8(SPI1, data);

	/* Wait until SPI1 is not busy */
	while (SPI1->SR & SPI_SR_BSY);

	/* Force LCK signal to 1 by forcing PD2 to 1 */
	GPIOD->BSRR = GPIO_BSRR_BS_2;
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
