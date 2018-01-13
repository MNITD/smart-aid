//
////#include "system_stm32f0xx.h"
//#include <stdio.h>
//#include "semihosting.h"
//#include "stm32f0xx.h"
//#include "stm32f0xx_i2c.h"
//#include "i2c_lcd.h"
//#include "string.h"
////#include "stm32f0xx_adc.h"#
////#include "stm32f0xx_hal.h"
//
//
//uint32_t prescaler_us;
//uint32_t prescaler_ms;
//int n = 0;
//
//
////void InitTIM15(void)
////{
////	prescaler_ms = SystemCoreClock / 1000;
////	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
////	TIM15->CR2 |= TIM_CR2_MMS_1; //output on a rising edge
////	TIM15->PSC |= 959; //Select prescaler /960
////	TIM15->ARR = (uint16_t)50000; //event uppdate each second
////	TIM15->CR1 |= TIM_CR1_CEN;// enable TIM15
////}
//
//void ADC_init(void){
//	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
//	RCC->CR2 |= RCC_CR2_HSI14ON;
//	while( (RCC->CR2 & RCC_CR2_HSI14RDY) == 0 ); //waiting while oscilator is ready
//}
//
//
//void ADC_enable(void){
//	//Enable ADC:
//		do
//		{
//			ADC1->CR |= ADC_CR_ADEN;
//		} while ( (ADC1->ISR & ADC_ISR_ADRDY) == 0 ); //whait while ADC ready
//}
//
//void ADC_calibrate(void){
//	if ( (ADC1->CR & ADC_CR_ADEN) != 0 ) {
//		ADC1->CR &= (uint32_t) (~ADC_CR_ADCAL) ;
//	}
//	ADC1->CR |= ADC_CR_ADCAL; // launch calibration
//	while ( (ADC1->CR & ADC_CR_ADCAL) != 0  );
//	ADC_enable();
//}
//
//void ADC_disable(void){
//	if ( (ADC1->CR & ADC_CR_ADSTART) != 0 ) // ensure no conversion is going
//	{
////		ADC_CR_ADCSTART
//		ADC1->CR |= ADC_CR_ADSTP; // stop any ongoing conversion
//	}
//	while ((ADC1->CR & ADC_CR_ADSTP)!=0);//wait ADSTP is reset by hardware
//	ADC1->CR |= ADC_CR_ADDIS; // disable ADC
//	while( (ADC1->CR & ADC_CR_ADEN) != 0 ); // until the ADC is fully disabled
//}
//
//int ADC_single_conversion_sequenceSWtrigger (){
////	ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; //select HSI14 by writing  00 in CKMODE
////	ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL_9 | ADC_CHSELR_CHSEL10 | ADC_CHSELR_CHSEL17 ;// for temp measure
////	ADC1->
//}
//
////void photoresistor_pin_config(){
////	/* GPIOC Periph clock enable */
////	  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
////}
//
//void  ConfigureGPIO(void)
//{
//  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
//  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER8|GPIO_MODER_MODER9|GPIO_MODER_MODER4)) \
//               | (GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0 | GPIO_MODER_MODER4);
//}
//
////GPIO_InitTypeDef G;
//
////void GPIO_analog_init(void){
////	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
////	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
////
////	G.GPIO_Pin = GPIO_Pin_4;
////	G.GPIO_Mode = GPIO_Mode_AN;
//////	G.GPIO_OType = GPIO_OType_PP;
//////	G.GPIO_PuPd = GPIO_PuPd_NOPULL;
////	G.GPIO_Speed = GPIO_Speed_2MHz;
////	GPIO_Init(GPIOC, &G);
////}
//
//void SetClockForADC(void)
//{
//  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
//  RCC->CR2 |= RCC_CR2_HSI14ON;
//  while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0);
//
//}
//
//void  CalibrateADC(void)
//{
//  if ((ADC1->CR & ADC_CR_ADEN) != 0)
//  {
//    ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);
//  }
//  ADC1->CR |= ADC_CR_ADCAL;
//  while ((ADC1->CR & ADC_CR_ADCAL) != 0) ;
//}
//
//void EnableADC(void)
//{
//  do
//  {
//		ADC1->CR |= ADC_CR_ADEN;
//  }
//  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);
//}
//
//void ConfigureADC(void)
//{
//  //ADC1->CFGR1 |= ADC_CFGR1_AUTOFF;
//  ADC1->CHSELR = ADC_CHSELR_CHSEL14;
//  //ADC1->SMPR |= ADC_SMPR1_SMPR_0 | ADC_SMPR1_SMPR_1 | ADC_SMPR1_SMPR_2;
//  //ADC->CCR |= ADC_CCR_VREFEN;
//}
//
//int TIM6delay_ms(uint16_t value){
//	TIM6->PSC = prescaler_ms; // apply current prescaler
//	TIM6->ARR = value;  // countdown limitation
//	TIM6->CNT = 0; // assure countdown  set to 0
//	TIM6->CR1 |= TIM_CR1_CEN;  //enable timer (automatic start)
//		while((TIM6->SR & TIM_SR_UIF)==0){}  // check for interuptions upon timer owerflow
//		// till counting is over
//		TIM6->SR &=~TIM_SR_UIF; // reset flag
//	return 0;
//}
//int TIM6delay_us(uint16_t value){
//	TIM6->PSC = prescaler_us; // apply current prescaler
//	TIM6->ARR = value;  // countdown limitation
//	TIM6->CNT = 0; // assure countdown  set to 0
//	TIM6->CR1 |= TIM_CR1_CEN;  //enable timer (automatic start)
//		while((TIM6->SR & TIM_SR_UIF)==0){}  // check for interuptions upon timer owerflow
//		// till counting is over
//		TIM6->SR &=~TIM_SR_UIF; // reset flag
//	return 0;
//}
//int TIM6delay(uint16_t value){
//	TIM6->PSC = prescaler_ms; // apply current prescaler
//	TIM6->ARR = value;  // countdown limitation
//	TIM6->CNT = 0; // assure countdown  set to 0
//	TIM6->CR1 |= TIM_CR1_CEN;  //enable timer (automatic start)
//		while((TIM6->SR & TIM_SR_UIF)==0){}  // check for interuptions upon timer owerflow
//		// till counting is over
//		TIM6->SR &=~TIM_SR_UIF; // reset flag
//	return 0;
//}
//
//void InitDelayTIM6(void){
//	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // enable TIM6 timer
//}
//
//void I2C2_init(void)
//{
//	I2C2->TIMINGR = (uint32_t)0x00B01A4B; /* (1) */
//	I2C2->CR1 = I2C_CR1_PE | I2C_CR1_RXIE; /* (2) */
//	I2C2->CR2 = I2C_CR2_AUTOEND | (1<<16) | I2C_CR2_RD_WRN | (LCD_ADDR << 1); /* (3) */
//}
//
//void ConfigureGPIOI2C2 (void){
//	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
//	GPIOB->OTYPER |= GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11;
//	GPIOB->AFR[1] = (GPIOB->AFR[1] &~ (GPIO_AFRH_AFRH2 | GPIO_AFRH_AFRH3)) | (1 << (2*4)) | (1 << (3*4));
//	GPIOB->MODER = (GPIOB->MODER & ~ (GPIO_MODER_MODER10 | GPIO_MODER_MODER11) ) | (GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1);
//}
//void ConfigureMASTERI2C2 (void){
//	RCC->AHBENR |= RCC_APB1ENR_I2C2EN;
//	I2C2->TIMINGR = (uint32_t)0x00B01A4B;
//	I2C2->CR1 = I2C_CR1_PE | I2C_CR1_RXIE;
//	I2C2->CR2 = I2C_CR2_AUTOEND | (1<<16) | I2C_CR2_RD_WRN | (0x05<<1);
//	NVIC_SetPriority(I2C2_IRQn, 0);
//	NVIC_EnableIRQ(I2C2_IRQn);
//}
//
//
//
//void I2C_master_transmision(void){
//
//
//	if ( (I2C2->ISR & I2C_ISR_TXE) == (I2C_ISR_TXE) ) // Check Tx empty
//	{
////		I2C2->TXDR = I2C_BYTE_TO_SEND;
//		I2C2->CR2 |= I2C_CR2_START; // START REQUEST
//	}
//}
//
//
//
//
//	int main(void){
//		SystemInit();
//		ConfigureGPIO();
//		SetClockForADC();
//		CalibrateADC();
//		EnableADC();
//		ConfigureADC();
//
//		prescaler_ms = SystemCoreClock / 1000;
//		prescaler_us = SystemCoreClock / 1000000;
//		int i ;
//
//		ConfigureGPIOI2C2();
//		ConfigureMASTERI2C2();
//		I2C2_init();
//
//
//		const uint8_t mes[] = "Message1";
//		const uint8_t mes1[] = "Message1";
//
//
//		uint8_t str [20];
//		//....................
//
//
//			lcd_Init();
//
//
//			lcd_Command(0xC0);
//			sprintf(str, mes1);
//			lcd_PrintC(str);
//
//
//			lcd_Command(0x80);
//			sprintf(str, mes);
//			lcd_PrintC(str);
//
//			TIM6delay_ms(1000);
//			lcd_Command(0x01);
//			lcd_Command(0x80);
//
//			sprintf(str, "123");
//			lcd_PrintC(str);
//
//
//			TIM6delay_ms(1000);
//			lcd_Command(0x01);
//			lcd_Command(0xC0);
//
//
//			sprintf(str, "Pi = %.2f", 3.14);
//			lcd_PrintC(str);
//
//
////		I2C_master_transmision();
//
////		ADC1->CHSELR = ADC_CHSELR_CHSEL11;
////		ADC1->SMPR = ADC_SMPR1_SMPR_0 | ADC_SMPR1_SMPR_1 | ADC_SMPR1_SMPR_2 ;
////		ADC1->CR |= ADC_CR_ADSTART;
//	   	GPIOC->ODR ^= 1<<8;
//	   	InitDelayTIM6();
//	   	uint16_t ii;
//		while(1){
////			printf("%d \n\r", SystemCoreClock);
////			ADC1->CR |= ADC_CR_ADSTART;
////			while( (ADC1->ISR & ADC_ISR_EOC) == 0 );
////			i = ADC1->DR;
////			printf("Light: %d us\n\r", i);
//			 ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
//			    while ((ADC1->ISR & ADC_ISR_EOC) == 0); /* wait end of conversion */
//
//			    ii = ADC1->DR;
//			    printf("Light: %d us\n\r", ii);
////			    if ((ii > 2000) /*&& (ADC1->DR > vrefint_low)*/)
////			    {
//////			      error |= WARNING_MEASURE; /* warning as the measure is out of the range */
////			    	GPIOC->ODR ^= 1<<8;
////			    }
////			    TIM6delay(2000);
//		}
//
//
//	}
