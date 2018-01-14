#include "adc_photoresistor.h"




// ================== PHOTO ============

void  ConfigureGPIO_ADC(void)
{
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER4)) \
               | ( GPIO_MODER_MODER4);
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

int launch_photoresistor(void){
				 ConfigureGPIO_ADC();
				 			SetClockForADC();
				 			  CalibrateADC();
				 			  EnableADC();
				 			  ConfigureADC();
			 }
// =================== PHOTO===========+
