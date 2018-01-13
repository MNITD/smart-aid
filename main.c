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
uint32_t prescaler_us;
uint32_t prescaler_ms;

// LCD ==============================

int TIM6delay_us(uint16_t value){
	TIM6->PSC = prescaler_us; // apply current prescaler
	TIM6->ARR = value;  // countdown limitation
	TIM6->CNT = 0; // assure countdown  set to 0
	TIM6->CR1 |= TIM_CR1_CEN;  //enable timer (automatic start)
		while((TIM6->SR & TIM_SR_UIF)==0){}  // check for interuptions upon timer owerflow
		// till counting is over
		TIM6->SR &=~TIM_SR_UIF; // reset flag
	return 0;
}
int TIM6delay_ms(uint16_t value){
	TIM6->PSC = prescaler_ms; // apply current prescaler
	TIM6->ARR = value;  // countdown limitation
	TIM6->CNT = 0; // assure countdown  set to 0
	TIM6->CR1 |= TIM_CR1_CEN;  //enable timer (automatic start)
		while((TIM6->SR & TIM_SR_UIF)==0){}  // check for interuptions upon timer owerflow
		// till counting is over
		TIM6->SR &=~TIM_SR_UIF; // reset flag
	return 0;
}

void InitDelayTIM6(void){
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // enable TIM6 timer
}
#include "i2c_lcd.h"

#define I2C1_OWN_ADDRESS (0x3F) // extra important ! Real address of LCD device




/*void gpioInit(void);
void i2cInit(void);
void delay(uint32_t t);
*/

//const uint8_t mes[] = "STM32F4 + I2C + LCD";
const uint8_t mes[] = "TMW";
const uint8_t mes1[] = "AAA";

uint8_t str [20];

void gpioInit(void) {
	//GPIO_InitTypeDef gpio;
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	/* Select output mode (01) on PC8 and PC9 */
	/*gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		gpio.GPIO_Mode = GPIO_Mode_AF;*/
	  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7)) \
					 | (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);

	 //gpio.GPIO_Speed = GPIO_Speed_50MHz;
	  //GPIOB->OSPEEDR = 0xFFFF;

	 // gpio.GPIO_PuPd = GPIO_PuPd_UP;
	  //GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0;

	  GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;

	/*
	gpio.GPIO_OType = GPIO_OType_OD;
	*/
	//GPIO_Init(GPIOB, &gpio);



	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

	GPIOB->AFR[0] = (GPIOB ->AFR[0] &~ (GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7))\
		                  | (1 << (6 * 4)) | (1 << (7 * 4));


}

void i2cInit(void) {
	//I2C_InitTypeDef i2c;

	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	  /* Configure I2C2, master */
	  /* (1) Timing register value is computed with the AN4235 xls file,
	   fast Mode @400kHz with I2CCLK = 48MHz, rise time = 140ns, fall time = 40ns */
	  /* (2) Periph enable */
	  /* (3) Slave address = 0x5A, write transfer, 1 byte to transmit, autoend */
	I2C1->CR1 &= ~(I2C_CR1_PE) /*| I2C_CR1_GCEN | I2C_CR1_WUPEN*/; /* (2) */

	I2C1->TIMINGR = (uint32_t)0x00E0D3FF /*0x205078C1 *//*0x108065BE*//*0x60201B6B*//* 0x40422631*/; /* (1) */

	  I2C1->CR1 |= I2C_CR1_PE /*| I2C_CR1_GCEN | I2C_CR1_WUPEN*/; /* (2) */

	  I2C1->CR2 |=  I2C_CR2_AUTOEND | (1<<16) | (I2C1_OWN_ADDRESS<<1); /* (3) */




	/*i2c.I2C_ClockSpeed = 50000;
	i2c.I2C_Mode = I2C_Mode_I2C;
	i2c.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c.I2C_OwnAddress1 = 0x00;
	i2c.I2C_Ack = I2C_Ack_Enable;
	i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &i2c);

	I2C_Cmd(I2C1, ENABLE);*/
}
/*
void iii2c_send(uint8_t data){
	// start I2C master transmission sequence
	while(!(I2C1->ISR & I2C_ISR_TXE) == (I2C_ISR_TXE)) // Check Tx empty
	{
	  I2C1->TXDR = data; // Byte to send
	  I2C1->CR2 |= I2C_CR2_START; // Go
}
}*/
// LCD  ==================================


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
	prescaler_ms = SystemCoreClock / 1000;
	prescaler_us = SystemCoreClock / 1000000;
	SystemInit();
//	uint32_t  i = 0;
	initgpio();
	Servo_init();

	// LCD ====================================================
	GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7)) \
					 | (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);


		gpioInit();
		i2cInit();
		InitDelayTIM6();
//		delay_init();
		TIM6delay_ms(100);
		lcd_Init();
		/*TIM6delay_ms(10);
		lcd_Init();*/

		lcd_Command(0xC0);
		sprintf(str, mes1);
		lcd_PrintC(str);


		lcd_Command(0x80);
		sprintf(str, mes);
		lcd_PrintC(str);

		TIM6delay_ms(1000);
		lcd_Command(0x01);
		lcd_Command(0x80);

		sprintf(str, "ABO");
		lcd_PrintC(str);
		sprintf(str, "ABO");
		lcd_PrintC(str);

		TIM6delay_ms(1000);
		lcd_Command(0x01);
		lcd_Command(0xC0);

		/*lcd_PrintC(mes);
		lcd_PrintC(mes);*/

	//	lcd_Goto(1, 0);

		sprintf(str, "Value of Pi");
		lcd_PrintC(str);
	// LCD ===================================================
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


