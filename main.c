#include "stm32f0xx.h"
#include "string.h"
#include "semihosting.h"
#include <stdio.h>
//#include "stm32f0xx_rcc.h"
//#include "tm_stm32f4_keypad.h"
//#include "stm32f0xx_gpio_init.h"
#include <stm32f0xx_usart.h>
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
//
void UART_Init(void) {
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  USART_StructInit(&USART_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

  //TX                                GPIO_AF_1 = USART1
  gpio_pinSetup_AF(GPIOA, GPIO_Pin_9, GPIO_AF_1, GPIO_OType_PP, GPIO_PuPd_UP,
      GPIO_Speed_50MHz);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  //RX
  gpio_pinSetup_AF(GPIOA, GPIO_Pin_10, GPIO_AF_1, GPIO_OType_PP, GPIO_PuPd_UP,
      GPIO_Speed_50MHz);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  USART_InitStructure.USART_BaudRate = 19200;

  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);
  //USART_Init(USART2, &USART_InitStructure);
  //USART_Init(USART3, &USART_InitStructure);

  // UNCOMMENT IF RX INTERRUPT IS NEEDED
  // set according NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  //NVIC_SetPriority(USART1_IRQn, 3);    // ? don't know if it is neccessary
  NVIC_EnableIRQ(USART1_IRQn);

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  //NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  //NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;

  NVIC_InitStructure.NVIC_IRQChannelPriority = 1; //SET PRIORITY!!!

  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //enable USART1 interrupt - Receive Data register not empty interrupt.
  //USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  //enable USART1 interrupt - Receive Data register not empty interrupt.
  //USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);  //enable USART1 interrupt - Receive Data register not empty interrupt.

  USART_Cmd(USART1, ENABLE);
  //USART_Cmd(USART2, ENABLE);
  //USART_Cmd(USART3, ENABLE);
}






int main (void) {
	LR_RED_active = 0;
	LR_RED_passive = 0;
	LR_BLUE_active = 0;
	LR_BLUE_passive= 0 ;
	prescaler_ms = SystemCoreClock / 1000;
	prescaler_us = SystemCoreClock / 1000000;
	SystemInit();
//	uint32_t  i = 0;
//	initgpio_keyboard();
//	Servo_init();


//	LCD_launch() ;
//	LCD_test_run() ;



	SystemInit();
	InitDelayTIM6();


	launch_photoresistor();
	int resistor_number = 0;
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);

	config_light_resistor_pin(LR_RED_PORT, LR_RED_Pin);
	config_light_resistor_pin(LR_BLUE_PORT, LR_BLUE_Pin);
	config_light_resistor_pin(LR_RED_DIOD_PORT, LR_RED_DIOD_Pin);
	config_light_resistor_pin(LR_BLUE_DIOD_PORT, LR_BLUE_DIOD_Pin);
	listenLR(LR_RED_PORT, LR_RED_Pin);
	listenLR(LR_BLUE_PORT, LR_BLUE_Pin);

	int LR_data ;

	calibrate_light_resistor(LR_RED_PORT, LR_RED_Pin);
	printf("RED_ACTIVE = %d, RED_PASSIVE = %d\n\r" ,LR_RED_active ,LR_RED_passive );
	calibrate_light_resistor(LR_BLUE_PORT, LR_BLUE_Pin);
	printf("BLUE_ACTIVE = %d, BLUE_PASSIVE = %d\n\r" ,LR_BLUE_active ,LR_BLUE_passive );

	printf("LR calibration OK \n\r");

//			int i ;
//			while(1){ // PHOTORESISTOR!!!
//
//				if (resistor_number){
//					unlistenLR(LR_BLUE_PORT, LR_BLUE_Pin);
//					listenLR(LR_RED_PORT, LR_RED_Pin);
//					LR_data = read_from_LR(LR_RED_PORT, LR_RED_Pin);
//					printf("LR RED is %d\n\r", LR_data);
//					resistor_number ^= 1;
//				}else{
//					unlistenLR(LR_RED_PORT, LR_RED_Pin);
//					listenLR(LR_BLUE_PORT, LR_BLUE_Pin);
//					LR_data = read_from_LR(LR_BLUE_PORT, LR_BLUE_Pin);
//					printf("LR BLUE is %d\n\r", LR_data);
//					resistor_number ^= 1;
//				}
//				TIM6delay_ms(500);
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
//	UART_Init();
	while(1){
//		puts("test\n");
//		TIM6delay_ms(1000); // Debounce and several ckicks ommit



//		while(1)
//		{
////			key = read_keypad_key();
////			if (key){
////				printf("key_col %c \n\r", key);
////				TIM6delay_ms(1000); // Debounce and several ckicks ommit
////			}
//
////			check_keypad();
//
//		}


//		pressedBtn = TM_KEYPAD_Read();
//		printf("test: %d\n\r", GPIOA->IDR & GPIO_Pin_10 );
//		printf("The value is %s\n\r", pressedBtn);

	}
//	 ==================== KEYBOARD ===========================
}


