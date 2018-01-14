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
	InitDelayTIM6();

	launch_photoresistor();

//			int i ;
//			while(1){
//				 ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
//				    while ((ADC1->ISR & ADC_ISR_EOC) == 0); /* wait end of conversion */
//				    printf("Light is %d\n\r", ADC1->DR);
//			}



		//TODO make Enum Strings ?
//		servo_45_0_45_test_loop(void) ////////////SERVO _____T E S T ________



//	 =============== KEYBOARD ====================
//	TM_KEYPAD_Button_t pressedBtn;
//	InitKeypad();
//	int a = ;
//	printf("%d", a);
//	printf("port %d\n\r", (*key_cols[0].Port));
//				printf("DEF %d\n\r", KEYPAD_COLUMN_1_PORT);
//				printf("GPIO %d\n\r", GPIOA);
	int key;
	while(1){
		while(1)
		{
			key = read_keypad_key();
			if (key){
				printf("key_col %c \n\r", key);
				TIM6delay_ms(1000); // Debounce and several ckicks ommit
			}

//			check_keypad();

		}


//		pressedBtn = TM_KEYPAD_Read();
//		printf("test: %d\n\r", GPIOA->IDR & GPIO_Pin_10 );
//		printf("The value is %s\n\r", pressedBtn);

	}
//	 ==================== KEYBOARD ===========================
}


