//
////#include "system_stm32f0xx.h"
//#include "stm32f0xx.h"
//
////#include "stm32f0xx_adc.h"#
////#include "stm32f0xx_hal.h"
//
//
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
//
//	int main(void){
//		SystemInit();
//		ConfigureGPIO();
//		SetClockForADC();
//		  CalibrateADC();
//		  EnableADC();
//		  ConfigureADC();
//
//		int i ;
////		ADC1->CHSELR = ADC_CHSELR_CHSEL11;
////		ADC1->SMPR = ADC_SMPR1_SMPR_0 | ADC_SMPR1_SMPR_1 | ADC_SMPR1_SMPR_2 ;
////		ADC1->CR |= ADC_CR_ADSTART;
//	   	GPIOC->ODR ^= 1<<8;
//	   	delay_init();
//	   	uint16_t ii;
//		while(1){
////			ADC1->CR |= ADC_CR_ADSTART;
////			while( (ADC1->ISR & ADC_ISR_EOC) == 0 );
////			i = ADC1->DR;
////			printf("Light: %d us\n\r", i);
//			 ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
//			    while ((ADC1->ISR & ADC_ISR_EOC) == 0); /* wait end of conversion */
//
//			    ii = ADC1->DR;
//			    if ((ii > 2000) /*&& (ADC1->DR > vrefint_low)*/)
//			    {
////			      error |= WARNING_MEASURE; /* warning as the measure is out of the range */
//			    	GPIOC->ODR ^= 1<<8;
//			    }
//			    delay_ms(200);
//		}
//
//
//	}
