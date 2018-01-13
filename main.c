#include "stm32f0xx.h"
#include "string.h"
#include "semihosting.h"
#include <stdio.h>
#include "stm32f0xx_rcc.h"
//#include "tm_stm32f4_keypad.h"

#include <stm32f0xx_rcc.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_misc.h>
#include "stm32f0xx_keypad_d1md1m.h"
//#include <stm32f0xx_dma.h>

#include "APS.h" // ========= SERVO ============


//ADC_InitTypeDef A;
GPIO_InitTypeDef G;
NVIC_InitTypeDef N;
//DMA_InitTypeDef D;

//#define ROW1 GPIOA->GPIO_Pin_9


void initgpio()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ;
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
//==================SERVO ===================




int main (void) {
	SystemInit();
	uint32_t  i = 0;
	initgpio();
	Servo_init();

		//TODO make Enum Strings ?


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


