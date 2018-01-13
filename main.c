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


// ================== PHOTO ============

void ADC_init(void){
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->CR2 |= RCC_CR2_HSI14ON;
	while( (RCC->CR2 & RCC_CR2_HSI14RDY) == 0 ); //waiting while oscilator is ready
}


void ADC_enable(void){
	//Enable ADC:
		do
		{
			ADC1->CR |= ADC_CR_ADEN;
		} while ( (ADC1->ISR & ADC_ISR_ADRDY) == 0 ); //whait while ADC ready
}

void ADC_calibrate(void){
	if ( (ADC1->CR & ADC_CR_ADEN) != 0 ) {
		ADC1->CR &= (uint32_t) (~ADC_CR_ADCAL) ;
	}
	ADC1->CR |= ADC_CR_ADCAL; // launch calibration
	while ( (ADC1->CR & ADC_CR_ADCAL) != 0  );
	ADC_enable();
}

void ADC_disable(void){
	if ( (ADC1->CR & ADC_CR_ADSTART) != 0 ) // ensure no conversion is going
	{
//		ADC_CR_ADCSTART
		ADC1->CR |= ADC_CR_ADSTP; // stop any ongoing conversion
	}
	while ((ADC1->CR & ADC_CR_ADSTP)!=0);//wait ADSTP is reset by hardware
	ADC1->CR |= ADC_CR_ADDIS; // disable ADC
	while( (ADC1->CR & ADC_CR_ADEN) != 0 ); // until the ADC is fully disabled
}

int ADC_single_conversion_sequenceSWtrigger (){
//	ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; //select HSI14 by writing  00 in CKMODE
//	ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL_9 | ADC_CHSELR_CHSEL10 | ADC_CHSELR_CHSEL17 ;// for temp measure
//	ADC1->
}

//void photoresistor_pin_config(){
//	/* GPIOC Periph clock enable */
//	  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
//}

void  ConfigureGPIO(void)
{
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8|GPIO_MODER_MODER9|GPIO_MODER_MODER4)) \
               | (GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0 | GPIO_MODER_MODER4);
}

//GPIO_InitTypeDef G;

//void GPIO_analog_init(void){
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
//
//	G.GPIO_Pin = GPIO_Pin_4;
//	G.GPIO_Mode = GPIO_Mode_AN;
////	G.GPIO_OType = GPIO_OType_PP;
////	G.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	G.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_Init(GPIOC, &G);
//}

void SetClockForADC(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  RCC->CR2 |= RCC_CR2_HSI14ON;
  while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0);

}

void  CalibrateADC(void)
{
  if ((ADC1->CR & ADC_CR_ADEN) != 0)
  {
    ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);
  }
  ADC1->CR |= ADC_CR_ADCAL;
  while ((ADC1->CR & ADC_CR_ADCAL) != 0) ;
}

void EnableADC(void)
{
  do
  {
		ADC1->CR |= ADC_CR_ADEN;
  }
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);
}

void ConfigureADC(void)
{
  //ADC1->CFGR1 |= ADC_CFGR1_AUTOFF;
  ADC1->CHSELR = ADC_CHSELR_CHSEL14;
  //ADC1->SMPR |= ADC_SMPR1_SMPR_0 | ADC_SMPR1_SMPR_1 | ADC_SMPR1_SMPR_2;
  //ADC->CCR |= ADC_CCR_VREFEN;
}

// =================== PHOTO===========+



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


// ================== PHOTORESISTOR ==============
uint32_t prescaler_ms;
int n = 0;




// ================== PHOTORESISTOR ==============

int main (void) {
	SystemInit();
//	uint32_t  i = 0;
	initgpio();
	Servo_init();
//	SystemInit();

	ConfigureGPIO();
			SetClockForADC();
			  CalibrateADC();
			  EnableADC();
			  ConfigureADC();

			int i ;
	//		ADC1->CHSELR = ADC_CHSELR_CHSEL11;
	//		ADC1->SMPR = ADC_SMPR1_SMPR_0 | ADC_SMPR1_SMPR_1 | ADC_SMPR1_SMPR_2 ;
	//		ADC1->CR |= ADC_CR_ADSTART;
//		   	GPIOC->ODR ^= 1<<8;
//		   	delay_init();
//		   	uint16_t ii;
			while(1){
	//			ADC1->CR |= ADC_CR_ADSTART;
	//			while( (ADC1->ISR & ADC_ISR_EOC) == 0 );
	//			i = ADC1->DR;
	//			printf("Light: %d us\n\r", i);
				 ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
				    while ((ADC1->ISR & ADC_ISR_EOC) == 0); /* wait end of conversion */
				    printf("Light is %d\n\r", ADC1->DR);
//				    ii = ADC1->DR;
//				    if ((ii > 2000) /*&& (ADC1->DR > vrefint_low)*/)
//				    {
//	//			      error |= WARNING_MEASURE; /* warning as the measure is out of the range */
//				    	GPIOC->ODR ^= 1<<8;
//				    }
//				    delay_ms(200);
			}

		//TODO make Enum Strings ?


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


