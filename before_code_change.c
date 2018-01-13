//#include <stdio.h>
//#include "semihosting.h"
//#include "system_stm32f0xx.h"
//#include "stm32f0xx.h"
//#include "stm32f0xx_gpio.h"
//#include "stm32f0xx_syscfg.h"
//#include "stm32f0xx_usart.h"
//#include "stm32f0xx_adc.h"
//#include "stm32f0xx_rcc.h"
//#include "stm32f0xx_exti.h"
//#include "stm32f0xx_dac.h"
//#include "tm_stm32f4_keypad.h"
//#include "string.h"
////#include "stm32f0xx_adc.h"#
////#include "stm32f0xx_hal.h"
//
//
//uint32_t prescaler_ms;
//int n = 0;
//
//
//void InitTIM15(void)
//{
//	prescaler_ms = SystemCoreClock / 1000;
//	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
//	TIM15->CR2 |= TIM_CR2_MMS_1; //output on a rising edge
//	TIM15->PSC |= 959; //Select prescaler /960
//	TIM15->ARR = (uint16_t)50000; //event uppdate each second
//	TIM15->CR1 |= TIM_CR1_CEN;// enable TIM15
//}
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
//void photoresistor_pin_config(){
//	/* GPIOC Periph clock enable */
//	  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
//}
//
//GPIO_InitTypeDef G;
//
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
//
//int TIM6delay(uint16_t value){
//	uint32_t prescaler_ms; // milisecs prescaler
//	uint32_t prescaler_us; // nanosecs prescaler
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
//void I2C_init(void){
//
//}
//
//	int main(void){
//		SystemInit();
//		GPIO_analog_init();
//		InitTIM15();
//		ADC_init();
//		ADC_calibrate();
//		ADC_enable();
//		int i ;
//		ADC1->CHSELR = ADC_CHSELR_CHSEL11;
//		ADC1->SMPR = ADC_SMPR1_SMPR_0 | ADC_SMPR1_SMPR_1 | ADC_SMPR1_SMPR_2 ;
////		ADC1->CR |= ADC_CR_ADSTART;
//		while(1){
//			ADC1->CR |= ADC_CR_ADSTART;
//			while( (ADC1->ISR & ADC_ISR_EOC) == 0 );
//			i = ADC1->DR;
//			printf("Light: %d us\n\r", i);
//	        }
//	}
