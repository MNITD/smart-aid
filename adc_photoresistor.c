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


void calibrate_light_resistor(GPIO_TypeDef* LR_PORT, uint16_t LR_PIN){ //USE IN DIOD FACING POSITION
	GPIO_TypeDef* thisLR_DIOD_PORT;
	uint16_t thisLR_DIOD_PIN;
	TIM6delay_ms(2000);
	if ((LR_PORT == LR_RED_PORT) && (LR_RED_Pin == LR_PIN) ){
		thisLR_DIOD_PORT = LR_RED_DIOD_PORT;
		thisLR_DIOD_PIN = LR_RED_DIOD_Pin;
		}else{
			thisLR_DIOD_PORT = LR_BLUE_DIOD_PORT;
			thisLR_DIOD_PIN = LR_BLUE_DIOD_Pin;
//			thisLR_PIN_hight = &LR_BLUE_active;
//			thisLR_PIN_low =  &LR_BLUE_passive;
		}

	turnON_LR_DIOD(thisLR_DIOD_PORT, thisLR_DIOD_PIN);
	listenLR(LR_PORT, LR_PIN);
	TIM6delay_ms(2000);
	if ((LR_PORT == LR_RED_PORT) && (LR_RED_Pin == LR_PIN) ){
		LR_RED_active= read_from_LR(LR_PORT, LR_PIN);
		printf("got H = %d\n\r", LR_RED_active);
	}else{
		LR_BLUE_active = read_from_LR(LR_PORT, LR_PIN);
		printf("got H = %d\n\r", LR_RED_active);
	}
	turnOFF_LR_DIOD(thisLR_DIOD_PORT, thisLR_DIOD_PIN);

	TIM6delay_ms(2000);
	if ((LR_PORT == LR_RED_PORT) && (LR_RED_Pin == LR_PIN) ){
			LR_RED_passive= read_from_LR(LR_PORT, LR_PIN);
			printf("got L = %d\n\r", LR_RED_passive);
		}else{
			LR_BLUE_passive = read_from_LR(LR_PORT, LR_PIN);
			printf("got L = %d\n\r", LR_BLUE_passive);
		}

	unlistenLR(LR_PORT, LR_PIN);
}

int check_if_pill(GPIO_TypeDef* LR_PORT, uint16_t LR_PIN, GPIO_TypeDef* LR_DIOD_PORT, uint16_t LR_DIOD_PIN){
	TIM6delay_ms(200);
	int LR_value, dif_low, dif_hight;
	turnON_LR_DIOD(LR_DIOD_PORT,LR_DIOD_PIN);
	listenLR(LR_PORT, LR_PIN);
	TIM6delay_ms(1000);
	LR_value = read_from_LR(LR_DIOD_PORT,LR_DIOD_PIN);
	unlistenLR(LR_PORT, LR_PIN);
	turnOFF_LR_DIOD(LR_DIOD_PORT,LR_DIOD_PIN);
	if ((LR_PORT == LR_RED_PORT) && (LR_RED_Pin == LR_PIN) ){
		dif_low = (LR_RED_passive - LR_value) * (LR_RED_passive - LR_value);
		dif_hight = (LR_RED_active - LR_value) * (LR_RED_active - LR_value);
	}
	else{
		dif_low = (LR_BLUE_passive - LR_value) * (LR_BLUE_passive - LR_value);
		dif_hight = (LR_BLUE_active - LR_value) * (LR_BLUE_active - LR_value);
	}
	return (dif_low <= dif_hight);
}


void config_light_resistor_pin(GPIO_TypeDef* PORT, uint16_t Pin){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = Pin;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_Init(PORT, &GPIO_InitStructure);
};

void turnON_LR_DIOD(GPIO_TypeDef* PORT, uint16_t PIN){
	PORT->BSRR = PIN;
}
void turnOFF_LR_DIOD(GPIO_TypeDef* PORT, uint16_t PIN){
	PORT->BRR = PIN;
}
void listenLR(GPIO_TypeDef* PORT, uint16_t PIN){
	PORT->BSRR = PIN;
}
void unlistenLR(GPIO_TypeDef* PORT, uint16_t PIN){
	PORT->BRR = PIN;
}

int  read_from_LR(GPIO_TypeDef* PORT, uint16_t PIN){
	ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
	while ((ADC1->ISR & ADC_ISR_EOC) == 0); /* wait end of conversion */
	return ADC1->DR;
}
// =================== PHOTO===========+
