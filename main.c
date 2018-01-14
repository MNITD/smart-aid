#include "stm32f0xx.h"
#include "string.h"
#include "semihosting.h"
#include <stdio.h>
//#include "stm32f0xx_rcc.h"
//#include "tm_stm32f4_keypad.h"

#include <stm32f0xx_rcc.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_misc.h>
#include "stm32f0xx_keypad_d1md1m.h"
//#include <stm32f0xx_dma.h>
#include "delay/delay.h"
#include <stm32f0xx_i2c.h>
#include "i2c_lcd.h" // LCD =====================
#include "adc_photoresistor.h"
#include "APS.h" // ========= SERVO ============
extern uint16_t prescaler_us;
extern uint16_t prescaler_ms;

// LCD ==============================



//ADC_InitTypeDef A;
GPIO_InitTypeDef G;
NVIC_InitTypeDef N;
//DMA_InitTypeDef D;

//#define ROW1 GPIOA->GPIO_Pin_9


/* Rows */
/* Row 1 default */
#ifndef KEYPAD_ROW_1_PIN
#define KEYPAD_ROW_1_PORT			GPIOC
#define KEYPAD_ROW_1_PIN			GPIO_Pin_6
#endif
/* Row 2 default */
#ifndef KEYPAD_ROW_2_PIN
#define KEYPAD_ROW_2_PORT			GPIOC
#define KEYPAD_ROW_2_PIN			GPIO_Pin_7
#endif
/* Row 3 default */
#ifndef KEYPAD_ROW_3_PIN
#define KEYPAD_ROW_3_PORT			GPIOC
#define KEYPAD_ROW_3_PIN			GPIO_Pin_8
#endif
/* Row 4 default */
#ifndef KEYPAD_ROW_4_PIN
#define KEYPAD_ROW_4_PORT			GPIOC
#define KEYPAD_ROW_4_PIN			GPIO_Pin_9
#endif

/* Columns */
/* Column 1 default */
#ifndef KEYPAD_COLUMN_1_PIN
#define KEYPAD_COLUMN_1_PORT		GPIOA
#define KEYPAD_COLUMN_1_PIN			GPIO_Pin_8
#endif
/* Column 2 default */
#ifndef KEYPAD_COLUMN_2_PIN
#define KEYPAD_COLUMN_2_PORT		GPIOA
#define KEYPAD_COLUMN_2_PIN			GPIO_Pin_9
#endif
/* Column 3 default */
#ifndef KEYPAD_COLUMN_3_PIN
#define KEYPAD_COLUMN_3_PORT		GPIOA
#define KEYPAD_COLUMN_3_PIN			GPIO_Pin_10
#endif
/* Column 4 default */
#ifndef KEYPAD_COLUMN_4_PIN
#define KEYPAD_COLUMN_4_PORT		GPIOA
#define KEYPAD_COLUMN_4_PIN			GPIO_Pin_15
#endif

typedef struct {
	GPIO_TypeDef * Port;
	uint16_t Pin;
} KeypadControl;

KeypadControl KeyCol1, KeyCol2, KeyCol3, KeyCol4,    KeyRow1, KeyRow2, KeyRow3, KeyRow4;

//KeypadControl cols[] = {KeyCol1, KeyCol2,KeyCol3,KeyCol4};
//KeypadControl rows[] = {KeyRow1, KeyRow2, KeyRow3, KeyRow4};



void initgpio_keyboard()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 ;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void keypadColHight(GPIO_TypeDef * PORT, uint16_t PIN){
	PORT->BSRR = PIN;
}
void keypadColLow(GPIO_TypeDef * PORT, uint16_t PIN){
	PORT->BRR = PIN;
}

int read_keypad_cols(){
	for (int i = 1; i <= 4; i++ ){

	}
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)) {
		printf("Active col 1 \n\r");
		return 1;
	}
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9)) {
		printf("Active col 1 \n\r");
		return 2;
	}
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10)) {
		printf("Active col 1 \n\r");
		return 3;
	}
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15)) {
		printf("Active col 1 \n\r");
		return 4;
	}
	return 0;
}


void check_keypad(){

			{
			GPIOC->BSRR = GPIO_Pin_6;//set bit as high
			GPIOC->BRR = GPIO_Pin_7;//set bit as low
			GPIOC->BRR = GPIO_Pin_8;//set bit as low
			GPIOC->BRR = GPIO_Pin_9;//set bit as low

			{
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8))//read input bit PB12
				printf("The value is 8 \n\r");
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9)) //read input bit PB11
				printf("The value is 9 \n\r");
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10)) //read input bit PB10
				printf("The value is 10 \n\r");
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15)) //read input bit PB2
				printf("The value is 11 \n\r");
			}}
}


//==================SERVO ===================
GPIO_InitTypeDef G;

//Timekeeping functions
volatile uint32_t MSec = 0;
//int count = 10;
int pill = 0;

void SysTick_Handler(void){
	MSec++;
}

void Servo_init(void){
	//Enable GPIO clock
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

		//Intialize AnyPinServo library
		APS_Init();

		//Set PA0 and PA1 as outputs
		G.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
		G.GPIO_Mode = GPIO_Mode_OUT;
		G.GPIO_OType = GPIO_OType_PP;
		G.GPIO_PuPd = GPIO_PuPd_NOPULL;
		G.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOA, &G);
}

//Delay function (milliseconds)
void Delay(uint32_t T){
	uint32_t MSS = MSec;
	while((MSec-MSS)<T) __NOP;
}

int servo_45_0_45_test_loop(void){
				// SERVO =========================================
			uint8_t res;
				res = APS_AddPin(GPIOA, GPIO_Pin_0, APS_SERVOMIDDLE);
				if(res != AE_SUCCESS){
					//Do something with result
				}

				res = APS_AddPin(GPIOA, GPIO_Pin_1, APS_SERVOMIDDLE);
				if(res != AE_SUCCESS){
					//Do something with result
				}

				//Initialize millisecond counter
				SysTick_Config(SystemCoreClock/1000);
				int start_pos = 0;
				int left = 45;
				int right = -45;
				int count = 3;
				int phres = 1;
				while (1){
					if(count > 0) {
						int cur_pos = start_pos;
						APS_SetPositionDegree(GPIOA, GPIO_Pin_0, cur_pos);
						APS_WaitForUpdate();
						Delay(2000);	// 2 sec

						while(!pill) {		// pill not caught yet
							cur_pos += left;
							APS_SetPositionDegree(GPIOA, GPIO_Pin_0, cur_pos);
							APS_WaitForUpdate();
							Delay(2000);	// 2 sec

							cur_pos += right;
							APS_SetPositionDegree(GPIOA, GPIO_Pin_0, cur_pos);
							APS_WaitForUpdate();
							Delay(2000);	// 2 sec

							if (phres == 1) {	// check "pill caught"
								cur_pos += right;
								APS_SetPositionDegree(GPIOA, GPIO_Pin_0, cur_pos);
								APS_WaitForUpdate();
								Delay(2000);	// 2 sec
								pill = 1;		// pill caught
							}

						}
						count--;
						pill = 0;
					}
				}
				// SERVO =========================================
			}


//==================SERVO ===================


int main (void) {
	prescaler_ms = SystemCoreClock / 1000;
	prescaler_us = SystemCoreClock / 1000000;
	SystemInit();
//	uint32_t  i = 0;
	initgpio_keyboard();
	Servo_init();


//	LCD_launch() ;
//	LCD_test_run() ;






//	SystemInit();


	launch_photoresistor();


			int i ;
			while(1){
				 ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
				    while ((ADC1->ISR & ADC_ISR_EOC) == 0); /* wait end of conversion */
				    printf("Light is %d\n\r", ADC1->DR);
			}

		//TODO make Enum Strings ?
//		servo_45_0_45_test_loop(void) ////////////SERVO _____T E S T ________



	// =============== KEYBOARD ====================
//	TM_KEYPAD_Button_t pressedBtn;
//	while(1){
//		while(1)
//		{
//		GPIOC->BSRR = GPIO_Pin_6;//set bit as high
//		GPIOC->BRR = GPIO_Pin_7;//set bit as low
//		GPIOC->BRR = GPIO_Pin_8;//set bit as low
//		GPIOC->BRR = GPIO_Pin_9;//set bit as low
//
//		{
//		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8))//read input bit PB12
//			printf("The value is 8 \n\r");
//		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9)) //read input bit PB11
//			printf("The value is 9 \n\r");
//		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10)) //read input bit PB10
//			printf("The value is 10 \n\r");
//		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15)) //read input bit PB2
//			printf("The value is 11 \n\r");
//		}}
//
//
////		pressedBtn = TM_KEYPAD_Read();
////		printf("test: %d\n\r", GPIOA->IDR & GPIO_Pin_10 );
////		printf("The value is %s\n\r", pressedBtn);
//
//	}
	// ==================== KEYBOARD ===========================
}


